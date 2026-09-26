"""Gate contract for score_ros.base_link_xyz. No bags."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "eval"))
from geo import to_frame  # noqa: E402
from score_ros import BASE_FRAC, BASELINE_M, base_link_xyz  # noqa: E402

LAT0, LON0, H0 = 55.8, 37.42, 160.0
START = (LAT0, LON0, H0)


def fail(msg):
    print("FAIL", msg)
    raise SystemExit(1)


def lon_for_east(east_m, alt):
    lo, hi = LON0, LON0 + 0.02
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        x, y, _ = to_frame(np.array([LAT0]), np.array([mid]), np.array([alt]), START, "enu")
        if float(np.hypot(x[0], y[0])) < east_m:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def pair(east_m, alt_r, baseline_tol=2.0, height_tol=1.0):
    lon = LON0 if east_m == 0.0 else lon_for_east(east_m, alt_r)
    master = np.array([[0.0, 10.0, LAT0, LON0, H0, 2.0]])
    rover = np.array([[0.0, 10.0, LAT0, lon, alt_r, 2.0]])
    return base_link_xyz(master, rover, "enu", START, baseline_tol=baseline_tol, height_tol=height_tol)


def main():
    _, _, _, ok = pair(BASELINE_M, H0)
    if not ok[0]:
        fail("rigid 12.436 m baseline was rejected")
    _, _, _, ok = pair(BASELINE_M + 2.0, H0)
    if not ok[0]:
        fail("baseline exactly 2 m outside 12.436 was rejected")
    _, _, _, ok = pair(BASELINE_M + 2.05, H0)
    if ok[0]:
        fail("baseline 2.05 m outside 12.436 was kept")
    _, _, _, ok = pair(80.0, H0)
    if ok[0]:
        fail("an 80 m chord was kept")
    _, _, _, ok = pair(BASELINE_M, H0 + 0.4)
    if not ok[0]:
        fail("a 0.4 m height split, inside the clean-bag maximum, was rejected")
    _, _, _, ok = pair(BASELINE_M, H0 + 3.0)
    if ok[0]:
        fail("a 3 m height split was kept")
    _, _, _, ok = pair(BASELINE_M, H0, baseline_tol=-1.0)
    if ok[0]:
        fail("a negative baseline tolerance kept a sample")
    # Exact hit: w must be 1, so the chord uses this rover, not the previous one.
    lon = lon_for_east(BASELINE_M, H0)
    master = np.array([[0.0, 10.0, LAT0, LON0, H0, 2.0]])
    rover = np.array([
        [0.0, 9.0, LAT0, LON0, H0, 2.0],
        [0.0, 10.0, LAT0, lon, H0, 2.0],
    ])
    x, y, z, ok = base_link_xyz(master, rover, "enu", START)
    if not ok[0]:
        fail("exact rover stamp was not paired")
    mx, my, mz = to_frame(np.array([LAT0]), np.array([LON0]), np.array([H0]), START, "enu")
    rx = float(mx[0] + (x[0] - mx[0]) / BASE_FRAC)
    ry = float(my[0] + (y[0] - my[0]) / BASE_FRAC)
    span = float(np.hypot(rx - mx[0], ry - my[0]))
    if abs(span - BASELINE_M) > 0.05:
        fail(f"exact hit used the wrong rover sample, span {span:.3f}")
    # No bracket: rover only after a 1 s gap.
    rover_late = np.array([[0.0, 11.0, LAT0, lon, H0, 2.0]])
    _, _, _, ok = base_link_xyz(master, rover_late, "enu", START)
    if ok[0]:
        fail("a rover 1 s away was interpolated")
    from score_ros import output_rate_hz
    from judge import pair as pair_estimates
    hz, gap = output_rate_hz([0.0, 0.0, 0.1, 0.1, 0.2])
    if not (abs(hz - 10.0) < 1e-9 and abs(gap - 0.1) < 1e-9):
        fail(f"duplicate stamps changed the rate, hz={hz} gap={gap}")
    if np.isfinite(output_rate_hz([1.0, 1.0])[0]):
        fail("identical stamps produced a finite rate")
    from score_ros import output_rate_report, source_rates
    # Three callbacks, 0.1 ms apart, for 10 s at 10 Hz. Exact duplicates are
    # absent, so the unique-stamp median is the 0.1 ms gap.
    near = []
    for i in range(100):
        base = i / 10.0
        near.extend((base, base + 1e-4, base + 2e-4))
    near_rep = output_rate_report(near, near)
    if not (near_rep["rate_hz"] > 1000.0):
        fail(f"near-duplicate median did not inflate, hz={near_rep['rate_hz']}")
    if not (29.0 < near_rep["rate_record_hz"] < 31.0):
        fail(f"record-duration rate left the publish rate, hz={near_rep['rate_record_hz']}")
    if near_rep["n_unique"] != 300 or near_rep["n_duplicate"] != 0:
        fail("near-duplicates were counted as exact copies")
    if abs(near_rep["rate_wall_hz"] - near_rep["rate_record_hz"]) > 1e-9:
        fail("wall rate disagreed with the record when log time matches the header")
    exact = np.repeat(np.arange(100) / 10.0, 3)
    exact_rep = output_rate_report(exact)
    if abs(exact_rep["rate_hz"] - 10.0) > 1e-6:
        fail(f"exact duplicates changed the unique rate, hz={exact_rep['rate_hz']}")
    if exact_rep["n_unique"] != 100 or exact_rep["n_duplicate"] != 200:
        fail("exact duplicate count is wrong")
    if not (29.0 < exact_rep["rate_record_hz"] < 31.0):
        fail("exact duplicates did not raise the message rate over the record")
    back = output_rate_report([0.0, 0.2, 0.1, 0.3])
    if back["n_regressed"] != 1:
        fail("a backward step was not counted")
    src = source_rates({"front": 1000, "rear": 1000, "cmd": 500}, 100.0)
    if abs(src["front"] - 10.0) > 1e-9 or abs(src["cmd"] - 5.0) > 1e-9:
        fail("per-source rate is not the count over the span")
    if np.isfinite(source_rates({"front": None}, 100.0)["front"]):
        fail("a missing source produced a finite rate")
    t_est = np.arange(0.0, 1.01, 0.2)
    t_ref = np.arange(0.0, 1.01, 0.05)
    idx = pair_estimates(t_est, t_ref, 0.05)
    used = idx[idx >= 0]
    coverage = len(used) / len(t_ref)
    if coverage <= len(t_est) / len(t_ref) + 1e-9:
        fail("one estimate did not cover several references")
    if len(np.unique(used)) != len(t_est):
        fail("estimate reuse test did not use every output sample")
    from judge import compare_pairings, pair_one_to_one, score
    idx_oto = pair_one_to_one(t_est, t_ref, 0.05)
    used_oto = idx_oto[idx_oto >= 0]
    if len(used_oto) != len(np.unique(used_oto)):
        fail("one-to-one reused an estimate")
    if len(used_oto) > len(t_est):
        fail("one-to-one matched more references than estimates")
    prev_e = -1e99
    for i in np.argsort(t_ref, kind="mergesort"):
        j = int(idx_oto[i])
        if j < 0:
            continue
        if t_est[j] + 1e-12 < prev_e:
            fail("one-to-one match crossed in time")
        prev_e = t_est[j]
    # Two references sit on the estimate at 0.04. Legacy uses it twice.
    # One-to-one keeps the earlier estimate for the first reference.
    cross_e = np.array([0.00, 0.04])
    cross_r = np.array([0.03, 0.045])
    legacy_idx = pair_estimates(cross_e, cross_r, 0.05)
    oto_idx = pair_one_to_one(cross_e, cross_r, 0.05)
    if len(np.unique(legacy_idx)) != 1:
        fail("the legacy matcher did not reuse the nearer estimate")
    if list(oto_idx) != [0, 1]:
        fail(f"one-to-one did not walk forward, idx={list(oto_idx)}")
    est = {
        "t": np.array([0.0, 1.0]),
        "x": np.array([10.0, 0.0]),
        "y": np.zeros(2),
        "z": np.zeros(2),
        "v": np.zeros(2),
    }
    ref = {
        "t": np.array([0.0, 0.04, 0.05, 1.0]),
        "x": np.zeros(4),
        "y": np.zeros(4),
        "z": np.zeros(4),
        "v": np.zeros(4),
    }
    both = compare_pairings(est, ref)
    legacy = score(est, ref)
    if abs(both["rmse_3d_nearest_legacy"] - legacy["rmse_3d"]) > 1e-12:
        fail("the legacy alias moved the published RMSE")
    if abs(both["coverage_nearest_legacy"] - legacy["coverage"]) > 1e-12:
        fail("the legacy alias moved the published coverage")
    if abs(both["rmse_3d_nearest_legacy"] - (75.0 ** 0.5)) > 1e-9:
        fail(f"legacy RMSE changed, {both['rmse_3d_nearest_legacy']}")
    if abs(both["rmse_3d_one_to_one"] - (50.0 ** 0.5)) > 1e-9:
        fail(f"one-to-one RMSE changed, {both['rmse_3d_one_to_one']}")
    if abs(both["coverage_nearest_legacy"] - 1.0) > 1e-12 or abs(both["coverage_one_to_one"] - 0.5) > 1e-12:
        fail(f"coverage pair changed, {both}")
    if abs(exact_rep["rate_all_messages_hz"] - exact_rep["rate_record_hz"]) > 1e-9:
        fail("all-message rate left the record rate")
    if abs(exact_rep["rate_unique_stamp_hz"] - exact_rep["rate_unique_hz"]) > 1e-9:
        fail("unique-stamp rate left the unique rate")
    if abs(exact_rep["duplicate_stamp_fraction"] - 200.0 / 300.0) > 1e-12:
        fail("duplicate fraction is not the exact-copy share")
    if exact_rep["regressed_stamp_count"] != 0 or back["regressed_stamp_count"] != 1:
        fail("regressed stamp count disagreed with n_regressed")
    if abs(exact_rep["p50_gap"] - 0.1) > 1e-9 or abs(exact_rep["max_gap"] - exact_rep["max_gap_s"]) > 1e-12:
        fail("unique-stamp gaps moved the old max gap")
    if not (near_rep["p50_gap"] < 1e-3 and near_rep["p95_gap"] > near_rep["p50_gap"]):
        fail("near-duplicate gaps did not separate the median from the tail")
    from reference import along_track_speed, horizontal_speed
    vx = np.array([10.0, -10.0, 10.0, 0.0])
    vy = np.array([0.0, 0.0, 3.0, 4.0])
    tx = np.ones(4)
    ty = np.zeros(4)
    mod = horizontal_speed(vx, vy)
    par = along_track_speed(vx, vy, tx, ty)
    if np.any(mod < 0.0):
        fail("the horizontal module went negative")
    if abs(mod[0] - mod[1]) > 1e-12:
        fail("forward and reverse did not share one module")
    if abs(par[0] - 10.0) > 1e-12 or abs(par[1] + 10.0) > 1e-12:
        fail("the projection did not keep the sign")
    if mod[2] <= par[2] + 0.4:
        fail("a cross-track component did not raise the module")
    if abs(par[3]) > 1e-12 or abs(mod[3] - 4.0) > 1e-12:
        fail("pure cross-track speed still had an along-track part")
    from score_ros import retention_parts
    lon_ok = lon_for_east(BASELINE_M, H0)
    lon_far = lon_for_east(80.0, H0)
    masters = np.array([
        [0.0, 10.0, LAT0, LON0, H0, 2.0],
        [0.0, 11.0, LAT0, LON0, H0, 2.0],
        [0.0, 12.0, LAT0, LON0, H0, 2.0],
        [0.0, 13.0, LAT0, LON0, H0, 2.0],
    ])
    rovers = np.array([
        [0.0, 10.0, LAT0, lon_ok, H0, 2.0],
        [0.0, 12.0, LAT0, lon_far, H0, 2.0],
        [0.0, 13.0, LAT0, lon_ok, H0 + 3.0, 2.0],
    ])
    _, _, _, ok_time = base_link_xyz(masters, rovers, "enu", START, baseline_tol=1e9, height_tol=1e9)
    _, _, _, ok_base = base_link_xyz(masters, rovers, "enu", START, baseline_tol=2.0, height_tol=1e9)
    _, _, _, ok_all = base_link_xyz(masters, rovers, "enu", START, baseline_tol=2.0, height_tol=1.0)
    n_master = len(masters)
    n_paired = int(ok_time.sum())
    n_kept = int(ok_all.sum())
    parts = retention_parts(n_master, n_paired, n_kept)
    if n_paired != 3 or n_kept != 1:
        fail(f"the four causes were not separated, paired={n_paired} fix={n_kept}")
    if int((ok_time & ~ok_base).sum()) != 1 or int((ok_base & ~ok_all).sum()) != 1:
        fail("baseline and height rejects were not one each")
    if abs(parts["time_pair_retention"] - 0.75) > 1e-12:
        fail("time-pair retention is not n_paired/n_master")
    if abs(parts["rigid_body_retention"] - 1.0 / 3.0) > 1e-12:
        fail("rigid-body retention is not n_fix/n_paired")
    if abs(parts["total_retention"] - 0.25) > 1e-12:
        fail("total retention is not n_fix/n_master")
    if abs(parts["time_pair_retention"] * parts["rigid_body_retention"] - parts["total_retention"]) > 1e-12:
        fail("the three ratios do not multiply")
    # rmse_3d_raw keeps this call: gates off, still a paired base_link chord.
    lon80 = lon_for_east(80.0, H0)
    one = np.array([[0.0, 10.0, LAT0, LON0, H0, 2.0]])
    far = np.array([[0.0, 10.0, LAT0, lon80, H0, 2.0]])
    x, y, z, ok = base_link_xyz(one, far, "enu", START, baseline_tol=1e9, height_tol=1e9)
    if not ok[0]:
        fail("the ungated path dropped a time-paired rover")
    _, _, _, gated = base_link_xyz(one, far, "enu", START, baseline_tol=2.0, height_tol=1.0)
    if gated[0]:
        fail("the gated path kept an 80 m chord")
    mx, my, mz = to_frame(np.array([LAT0]), np.array([LON0]), np.array([H0]), START, "enu")
    off = float(np.hypot(x[0] - mx[0], y[0] - my[0]))
    if off < 1.0:
        fail("the ungated chord sat on the master antenna")
    if abs(off / BASE_FRAC - 80.0) > 2.0:
        fail(f"the ungated point is not the antenna fraction, offset {off:.2f}")
    if abs(float(z[0] - mz[0]) + 3.0) > 0.05:
        fail("the ungated height is not the antenna point 3 m down")
    _, _, _, alone = base_link_xyz(
        np.array([[0.0, 20.0, LAT0, LON0, H0, 2.0]]), far, "enu", START,
        baseline_tol=1e9, height_tol=1e9)
    if alone[0]:
        fail("the ungated path kept a master with no rover pair")
    lon40 = lon_for_east(40.0, H0)
    bracket = np.array([
        [0.0, 9.9, LAT0, lon40, H0, 2.0],
        [0.0, 10.1, LAT0, lon80, H0, 2.0],
    ])
    xi, yi, _, oki = base_link_xyz(one, bracket, "enu", START, baseline_tol=1e9, height_tol=1e9)
    if not oki[0]:
        fail("the ungated path did not interpolate a rover inside 0.25 s")
    midi = float(np.hypot(xi[0] - mx[0], yi[0] - my[0]))
    if not (off * 0.4 < midi < off * 0.8):
        fail(f"the interpolated chord is not between the two rovers, offset {midi:.2f}")

    from score_ros import keep_status
    status = np.array([-1, 0, 1, 2])
    if list(np.flatnonzero(keep_status(status, 0))) != [1, 2, 3]:
        fail("status >= 0 did not drop a negative status")
    if list(np.flatnonzero(keep_status(status, 2))) != [3]:
        fail("status >= 2 did not keep only RTK")

    # Every master is time-paired and every chord fails the baseline gate.
    masters = np.array([[0.0, 10.0 + i, LAT0, LON0, H0, 2.0] for i in range(4)])
    far_lon = lon_for_east(80.0, H0)
    rovers = np.array([[0.0, 10.0 + i, LAT0, far_lon, H0, 2.0] for i in range(4)])
    _, _, _, ok_time = base_link_xyz(masters, rovers, "enu", START, baseline_tol=1e9, height_tol=1e9)
    _, _, _, ok_gate = base_link_xyz(masters, rovers, "enu", START, baseline_tol=2.0, height_tol=1.0)
    if int(ok_time.sum()) != 4 or int(ok_gate.sum()) != 0:
        fail("a fully rejected baseline was not 4 time pairs and 0 gated fixes")
    if int(ok_gate.sum()) != 0:
        fail("the gated reference was not empty")

    # Same chords, two gates: raw keeps the long chord, the 2 m gate does not.
    edges = (BASELINE_M, BASELINE_M + 2.0, BASELINE_M + 2.05, 80.0)
    raw_keep = []
    gated_keep = []
    for east in edges:
        _, _, _, ok_raw = base_link_xyz(
            master, np.array([[0.0, 10.0, LAT0, lon_for_east(east, H0), H0, 2.0]]),
            "enu", START, baseline_tol=1e9, height_tol=1e9)
        _, _, _, ok_g = base_link_xyz(
            master, np.array([[0.0, 10.0, LAT0, lon_for_east(east, H0), H0, 2.0]]),
            "enu", START, baseline_tol=2.0, height_tol=1.0)
        raw_keep.append(bool(ok_raw[0]))
        gated_keep.append(bool(ok_g[0]))
    if raw_keep != [True, True, True, True] or gated_keep != [True, True, False, False]:
        fail(f"raw/gated sensitivity {raw_keep} {gated_keep}")

    from score_ros import prepare_gnss

    def fix(t, lat=LAT0, lon=LON0, alt=H0, status=0):
        return [0.0, t, lat, lon, alt, status]

    masters = np.array([fix(2.5), fix(4.0), fix(6.0)])
    rovers = np.array([fix(0.0), fix(0.5)])
    prep, err = prepare_gnss(masters, rovers, 3.0, 0)
    if prep is None or abs(prep["t_open"] - 0.0) > 1e-9 or abs(prep["window_end"] - 3.0) > 1e-9:
        fail(f"rover-first window did not open at the rover, {err} {prep}")
    after = prep["master"][prep["master"][:, 1] > prep["window_end"], 1]
    if not np.array_equal(after, np.array([4.0, 6.0])):
        fail(f"the excluded interval is not the node's first 3 s, after={after}")
    shared, err = prepare_gnss(masters, np.array([fix(2.5), fix(2.6)]), 3.0, 0)
    if shared is None or abs(shared["t_open"] - 2.5) > 1e-9:
        fail("a shared first stamp did not stay on that stamp")

    empty, err = prepare_gnss(np.zeros((0, 6)), rovers, 3.0, 0)
    if empty is not None or "master" not in err:
        fail(f"empty master did not name the master, {err}")
    below = np.array([fix(1.0, status=-1), fix(2.0, status=-1)])
    none_m, err = prepare_gnss(below, rovers, 3.0, 0)
    if none_m is not None or "master" not in err:
        fail(f"masters below status 0 did not fail cleanly, {err}")
    none_r, err = prepare_gnss(masters, below, 3.0, 0)
    if none_r is not None or "rover" not in err:
        fail(f"rovers below status 0 did not fail cleanly, {err}")
    missing, err = prepare_gnss(None, None, 3.0, 0)
    if missing is not None or "master" not in err:
        fail(f"missing arrays did not fail cleanly, {err}")

    poisoned = np.array([
        fix(0.0, lat=float("nan")),
        fix(1.0, alt=float("nan")),
        fix(2.0, lon=float("nan")),
        fix(2.2, lat=91.0),
        fix(4.0),
    ])
    rover_h = np.array([fix(0.0, alt=160.0), fix(0.4, alt=162.0)])
    kept, err = prepare_gnss(poisoned, rover_h, 3.0, 0)
    if kept is None or abs(kept["t_open"]) > 1e-9:
        fail(f"a NaN master opened the window or dropped the later fix, {err}")
    if not np.isfinite(kept["start"][2]) or abs(kept["start"][2] - 161.0) > 1e-6:
        fail(f"NaN height entered the origin, {kept['start']}")
    only_nan, err = prepare_gnss(poisoned[:4], rover_h, 3.0, 0)
    if only_nan is not None or "master" not in err:
        fail(f"an all-NaN master did not fail cleanly, {err}")

    mixed = np.array([fix(2.5, alt=100.0, status=0), fix(4.0, alt=200.0, status=2)])
    strict, err = prepare_gnss(mixed, rovers, 3.0, 2)
    if strict is None or abs(strict["t_open"]) > 1e-9:
        fail(f"min-status moved the window, {err}")
    if list(strict["master"][:, 5].astype(int)) != [2]:
        fail("min-status did not drop the non-RTK master")
    if abs(strict["start"][2] - 100.0) > 1e-6:
        fail("the frame origin left the in-window master")

    _, _, _, ok_empty_m = base_link_xyz(np.zeros((0, 6)), rovers, "enu", START)
    _, _, _, ok_empty_r = base_link_xyz(masters, np.zeros((0, 6)), "enu", START)
    if len(ok_empty_m) != 0 or bool(ok_empty_r.any()):
        fail("an empty antenna raised or kept a sample")
    print("ok")


if __name__ == "__main__":
    main()

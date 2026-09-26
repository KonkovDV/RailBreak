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
    t_est = np.arange(0.0, 1.01, 0.2)
    t_ref = np.arange(0.0, 1.01, 0.05)
    idx = pair_estimates(t_est, t_ref, 0.05)
    used = idx[idx >= 0]
    coverage = len(used) / len(t_ref)
    if coverage <= len(t_est) / len(t_ref) + 1e-9:
        fail("one estimate did not cover several references")
    if len(np.unique(used)) != len(t_est):
        fail("estimate reuse test did not use every output sample")
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
    print("ok")


if __name__ == "__main__":
    main()

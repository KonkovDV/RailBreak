"""Score a recorded /result/* bag against base_link, not the master antenna.

The organisers' tf in base_link (first-bogie yaw axis, wheel-rail contact):
master (−9.873, 0, 3), rover (2.563, 0, 3). base_link is the point 0.794 of
the way from master to rover, then 3 m down. rmse_3d_raw is the same chord
with the baseline and height gates turned off; it is still time-paired and
status-filtered, not the master antenna. A master fix is kept in rmse_3d only when
the rover lies on that rigid body: planar baseline within 2 m of 12.436 m,
and the two antenna heights within 1 m. On a clean recording the baseline
residual is centimetres (max 0.1 m) and the height split stays under 0.5 m;
a rover tens of metres away is not the antenna. Bogie pitch 7.55 m is not
this baseline.
Default frame is absolute MGRS, matching the node. Pairs by header stamp,
tolerance 0.05 s.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "eval"))
from geo import to_frame  # noqa: E402
from judge import score  # noqa: E402
from rosbag2_io import decode_message, iter_messages  # noqa: E402

# master→base_link is 9.873 of the 12.436 m master→rover baseline.
BASE_FRAC = 9.873 / 12.436
BASELINE_M = 12.436
BASELINE_TOL_M = 2.0
# Clean recording 30618_e9a34502: |dh| max 0.5 m. A 40 permille grade over
# 12.436 m is 0.5 m. 1 m is outside that body, short of a bad altitude.
HEIGHT_TOL_M = 1.0
ANTENNA_UP_M = 3.0


def output_rate_hz(stamps):
    """Hz from unique finite stamps. Duplicate header stamps do not make dt zero.

    Near-duplicates still shrink the median gap. rate_record_hz in
    output_rate_report is the message count over the header span.
    """
    report = output_rate_report(stamps)
    return report["rate_hz"], report["max_gap_s"]


def output_rate_report(stamps, log_stamps=None) -> dict:
    """Separate the quantities a single median dt mixes together.

    rate_hz is 1/median(diff(unique header stamps)). Exact duplicates do not
    make it infinite. Stamps a fraction of a millisecond apart still do.
    rate_record_hz is (N-1) over the header span. rate_wall_hz is the same
    over bag log time. n_duplicate counts exact repeats. n_regressed counts
    steps that go backwards in arrival order.
    """
    t = np.asarray(stamps, float)
    finite = t[np.isfinite(t)]
    n = int(finite.size)
    uniq = np.unique(finite)
    n_unique = int(uniq.size)
    dt = np.diff(uniq)
    dt = dt[dt > 0]
    span = float(uniq[-1] - uniq[0]) if n_unique >= 2 else float("nan")
    wall = float("nan")
    if log_stamps is not None:
        w = np.asarray(log_stamps, float)
        w = w[np.isfinite(w)]
        if w.size >= 2 and float(w[-1] - w[0]) > 0.0:
            wall = float((w.size - 1) / (w[-1] - w[0]))
    return {
        "n_stamp": n,
        "n_unique": n_unique,
        "n_duplicate": n - n_unique,
        "n_regressed": int(np.sum(np.diff(finite) < 0)) if n > 1 else 0,
        "rate_hz": float(1.0 / np.median(dt)) if dt.size else float("nan"),
        "max_gap_s": float(dt.max()) if dt.size else float("nan"),
        "rate_record_hz": float((n - 1) / span) if span > 0.0 else float("nan"),
        "rate_unique_hz": float((n_unique - 1) / span) if span > 0.0 else float("nan"),
        "rate_wall_hz": wall,
        "span_s": span,
    }


def _diag_count(diag: dict, key: str):
    raw = diag.get(key, "")
    if raw == "" or raw is None:
        return None
    return int(float(raw))


def source_rates(counts: dict, span_s: float) -> dict:
    """Publishes of one callback divided by the output header span."""
    out = {}
    for key, n in counts.items():
        if n is None or not (span_s > 0.0):
            out[key] = float("nan")
        else:
            out[key] = float(n) / float(span_s)
    return out


def load_source(path: Path) -> dict:
    """GNSS from an extracted npz or straight from the original rosbag2."""
    if path.suffix == ".npz":
        with np.load(path) as zz:
            return {k: zz[k] for k in zz.files}
    fix, vel, rover = [], [], []
    for ts, topic, typ, blob in iter_messages(path):
        r = decode_message(typ, blob)
        if not r:
            continue
        if topic == "/sensing/gnss/master/fix":
            fix.append((ts * 1e-9, r["stamp_s"], r["lat"], r["lon"], r["alt"], r["status"]))
        elif topic == "/sensing/gnss/master/vel":
            vel.append((ts * 1e-9, r["stamp_s"], r["v"], r["vy"], r["vz"]))
        elif topic == "/sensing/gnss/rover/fix":
            rover.append((ts * 1e-9, r["stamp_s"], r["lat"], r["lon"], r["alt"], r["status"]))
    out = {"sensing_gnss_master_fix": np.array(fix, float)}
    if vel:
        out["sensing_gnss_master_vel"] = np.array(vel, float)
    if rover:
        out["sensing_gnss_rover_fix"] = np.array(rover, float)
    return out


def base_link_xyz(master: np.ndarray, rover: np.ndarray, frame: str, start,
                  baseline_tol: float = BASELINE_TOL_M,
                  height_tol: float = HEIGHT_TOL_M):
    """master, rover rows: stamp, lat, lon, alt, ... Interpolate rover onto master.

    Drop a sample whose rover is not on the rigid antenna pair. Planar length
    catches a fix tens of metres off the tram. Height is separate: a 3 m
    altitude split changes the 3D length by 0.36 m, so a length
    gate alone would keep it.
    """
    rover = rover[np.argsort(rover[:, 1])]
    rt = rover[:, 1]
    t = master[:, 1]
    n = len(rt)
    i = np.searchsorted(rt, t, side="left")
    on_sample = (i < n) & (np.abs(t - rt[np.clip(i, 0, n - 1)]) <= 1e-3)
    bracket = (i > 0) & (i < n)
    ok = on_sample | bracket
    left = np.clip(i - 1, 0, n - 1)
    right = np.clip(i, 0, n - 1)
    # An exact hit uses that sample. A bracket uses the two neighbours when they are close.
    gap = rt[right] - rt[left]
    ok = on_sample | (bracket & (gap > 0) & (gap <= 0.25))
    w = np.zeros(len(t))
    use = ok & (gap > 0)
    w[use] = (t[use] - rt[left[use]]) / gap[use]
    def col(j):
        return rover[left, j] + w * (rover[right, j] - rover[left, j])
    mx, my, mz = to_frame(master[:, 2], master[:, 3], master[:, 4], start, frame)
    rx, ry, rz = to_frame(col(2), col(3), col(4), start, frame)
    x = mx + BASE_FRAC * (rx - mx)
    y = my + BASE_FRAC * (ry - my)
    z = mz + BASE_FRAC * (rz - mz) - ANTENNA_UP_M
    span = np.hypot(rx - mx, ry - my)
    ok = ok & (np.abs(span - BASELINE_M) <= baseline_tol)
    ok = ok & (np.abs(rz - mz) <= height_tol)
    return x, y, z, ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("result_bag", type=Path)
    ap.add_argument("source_npz", type=Path, help="original rosbag2 directory or extracted .npz")
    ap.add_argument("--window", type=float, default=3.0)
    ap.add_argument("--min-status", type=int, default=0,
                    help="keep master fixes with status >= this (2 = RTK only)")
    ap.add_argument("--frame", choices=("mkrs_start", "mkrs", "mgrs", "grid_start", "enu"),
                    default="mgrs",
                    help="must match the node's output_frame")
    ap.add_argument("--baseline-tol", type=float, default=BASELINE_TOL_M,
                    help="drop a reference sample when the planar master–rover "
                         "distance is outside 12.436 ± this many metres")
    ap.add_argument("--height-tol", type=float, default=HEIGHT_TOL_M,
                    help="drop a reference sample when the antenna heights differ "
                         "by more than this many metres")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    pos_t, px, py, pz, pv = [], [], [], [], []
    diag_last: dict[str, str] = {}
    vel_t, vel, vel_log = [], [], []
    for ts, topic, typ, blob in iter_messages(args.result_bag):
        rec = decode_message(typ, blob)
        if rec is None:
            continue
        if topic == "/result/position":
            pos_t.append(rec["stamp_s"]); px.append(rec["s"]); py.append(rec["y"]); pz.append(rec["z"]); pv.append(rec["v"])
        elif topic == "/result/velocity":
            vel_t.append(rec["stamp_s"]); vel.append(rec["velocity"]); vel_log.append(ts * 1e-9)
        elif topic == "/result/diagnostics":
            vals = rec.get("values", {})
            if "callback_max_us" in vals:
                diag_last = vals
    z = load_source(args.source_npz)
    g_all = z["sensing_gnss_master_fix"][np.argsort(z["sensing_gnss_master_fix"][:, 1])]
    status_counts = {int(s): int(n) for s, n in zip(*np.unique(g_all[:, 5].astype(int), return_counts=True))}
    # Origin is the node's start window: every fix with status >= 0. The scored
    # set can be stricter (RTK only) without moving the frame.
    g0 = g_all[g_all[:, 5] >= 0]
    t0 = g0[0, 1]
    w = g0[g0[:, 1] <= t0 + args.window]
    lat0, lon0, h0 = float(np.median(w[:, 2])), float(np.median(w[:, 3])), float(np.median(w[:, 4]))
    g = g_all[g_all[:, 5] >= args.min_status]
    rover = z.get("sensing_gnss_rover_fix")
    if rover is None or len(rover) < 2:
        print("no rover fixes: base_link is the master–rover segment, not the master antenna", file=sys.stderr)
        return 2
    rover = rover[rover[:, 5] >= 0] if rover.shape[1] > 5 else rover
    n_master = int(len(g))
    start = (lat0, lon0, h0)
    rx_raw, ry_raw, rz_raw, ok_time = base_link_xyz(
        g, rover, args.frame, start, baseline_tol=1e9, height_tol=1e9)
    _, _, _, ok_base = base_link_xyz(
        g, rover, args.frame, start, baseline_tol=args.baseline_tol, height_tol=1e9)
    rx, ry, rz, ok = base_link_xyz(
        g, rover, args.frame, start, baseline_tol=args.baseline_tol, height_tol=args.height_tol)
    n_paired = int(ok_time.sum())
    n_baseline_reject = int((ok_time & ~ok_base).sum())
    n_height_reject = int((ok_base & ~ok).sum())
    g_raw, rx_raw, ry_raw, rz_raw = g[ok_time], rx_raw[ok_time], ry_raw[ok_time], rz_raw[ok_time]
    g = g[ok]
    rx, ry, rz = rx[ok], ry[ok], rz[ok]
    if len(g) < 2:
        print("rover does not cover the master fixes inside the 12.436 m baseline", file=sys.stderr)
        return 2
    ref_v = np.full(len(g), np.nan)
    if "sensing_gnss_master_vel" in z:
        mv = z["sensing_gnss_master_vel"][np.argsort(z["sensing_gnss_master_vel"][:, 1])]
        # Horizontal module, not v · t(s). speed_reference.py measures the gap.
        ref_v = np.interp(g[:, 1], mv[:, 1], np.hypot(mv[:, 2], mv[:, 3]), left=np.nan, right=np.nan)
    after = g[:, 1] > t0 + args.window
    ref = {"t": g[after, 1], "x": rx[after], "y": ry[after], "z": rz[after], "v": ref_v[after]}
    est = {"t": np.array(pos_t), "x": np.array(px), "y": np.array(py), "z": np.array(pz), "v": np.array(pv)}
    pos = score(est, ref)
    raw_3d = float("nan")
    raw_coverage = float("nan")
    if len(g_raw) >= 2:
        after_raw = g_raw[:, 1] > t0 + args.window
        v_raw = np.full(len(g_raw), np.nan)
        if "sensing_gnss_master_vel" in z:
            v_raw = np.interp(g_raw[:, 1], mv[:, 1], np.hypot(mv[:, 2], mv[:, 3]), left=np.nan, right=np.nan)
        ref_raw = {"t": g_raw[after_raw, 1], "x": rx_raw[after_raw], "y": ry_raw[after_raw],
                   "z": rz_raw[after_raw], "v": v_raw[after_raw]}
        if len(ref_raw["t"]) >= 2:
            pos_raw = score(est, ref_raw)
            raw_3d = pos_raw["rmse_3d"]
            raw_coverage = pos_raw["coverage"]
    vt = np.array(vel_t)
    vest = {"t": vt, "x": np.zeros(len(vt)), "y": np.zeros(len(vt)), "z": np.zeros(len(vt)), "v": np.array(vel)}
    vs = score(vest, ref)
    rates = output_rate_report(vt, vel_log)
    rate_hz, max_gap_s = rates["rate_hz"], rates["max_gap_s"]
    src = source_rates(
        {
            "front": _diag_count(diag_last, "n_pub_front"),
            "rear": _diag_count(diag_last, "n_pub_rear"),
            "cmd": _diag_count(diag_last, "n_pub_cmd"),
        },
        rates["span_s"],
    )
    # end drift: 3D error at the last paired reference sample over path length
    from judge import pair
    idx = pair(est["t"], ref["t"])
    m = np.flatnonzero(idx >= 0)
    end_err = end_x = end_y = end_z = float("nan")
    path = float(np.sum(np.hypot(np.diff(ref["x"]), np.diff(ref["y"]))[np.hypot(np.diff(ref["x"]), np.diff(ref["y"])) < 30]))
    if len(m):
        j = m[-1]
        e = idx[j]
        end_x = float(est["x"][e] - ref["x"][j])
        end_y = float(est["y"][e] - ref["y"][j])
        end_z = float(est["z"][e] - ref["z"][j])
        end_err = float(math.sqrt(end_x ** 2 + end_y ** 2 + end_z ** 2))
    out = {
        "n_position": len(pos_t),
        "n_velocity": len(vel_t),
        "rate_hz": rate_hz,
        "max_gap_s": max_gap_s,
        "n_unique": rates["n_unique"],
        "n_duplicate": rates["n_duplicate"],
        "n_regressed": rates["n_regressed"],
        "rate_record_hz": rates["rate_record_hz"],
        "rate_unique_hz": rates["rate_unique_hz"],
        "rate_wall_hz": rates["rate_wall_hz"],
        "coverage": pos["coverage"],
        "rmse_3d": pos["rmse_3d"],
        "rmse_3d_raw": raw_3d,
        "coverage_raw": raw_coverage,
        "rmse_x": pos["rmse_x"], "rmse_y": pos["rmse_y"], "rmse_z": pos["rmse_z"],
        "rmse_v_result_velocity": vs["rmse_v"],
        "rmse_v_odometry_twist": pos["rmse_v"],
        "frame": args.frame,
        "reference": "base_link",
        "base_frac": BASE_FRAC,
        "baseline_m": BASELINE_M,
        "baseline_tol_m": args.baseline_tol,
        "height_tol_m": args.height_tol,
        "n_master": n_master,
        "n_paired": n_paired,
        "n_baseline_reject": n_baseline_reject,
        "n_height_reject": n_height_reject,
        "time_pair_retention": (float(n_paired) / n_master) if n_master else float("nan"),
        "rigid_body_retention": (float(ok.sum()) / n_paired) if n_paired else float("nan"),
        "reference_retention": (float(ok.sum()) / n_master) if n_master else float("nan"),
        "min_status": args.min_status,
        "n_fix": int(len(g)),
        "status_counts": status_counts,
        "end_err_m": end_err,
        "end_dx": end_x, "end_dy": end_y, "end_dz": end_z,
        "path_m": path,
        "end_drift_pct": 100.0 * end_err / path if path > 0 else float("nan"),
        "callback_max_us": float(diag_last.get("callback_max_us", "nan")),
        "n_pub_front": _diag_count(diag_last, "n_pub_front"),
        "n_pub_rear": _diag_count(diag_last, "n_pub_rear"),
        "n_pub_cmd": _diag_count(diag_last, "n_pub_cmd"),
        "n_dup_out": _diag_count(diag_last, "n_dup_out"),
        "n_behind_out": _diag_count(diag_last, "n_behind_out"),
        "rate_front_hz": src["front"],
        "rate_rear_hz": src["rear"],
        "rate_cmd_hz": src["cmd"],
        "n_anchor": int(float(diag_last.get("n_anchor", "0"))),
        "gnss_state": diag_last.get("gnss", ""),
        "gnss_note": diag_last.get("gnss_note", ""),
    }
    text = json.dumps(out, indent=1)
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

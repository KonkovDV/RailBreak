"""Score a recorded /result/* bag against base_link, not the master antenna.

The organisers' tf: master is 9.873 m behind base_link, rover is
2.563 m ahead, both 3.0 m up. base_link is the point 0.794 of the way from
master to rover, then 3 m down. Default frame is absolute MGRS, matching
the node. Pairs by header stamp, tolerance 0.05 s.
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
ANTENNA_UP_M = 3.0


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


def base_link_xyz(master: np.ndarray, rover: np.ndarray, frame: str, start):
    """master, rover rows: stamp, lat, lon, alt, ... Interpolate rover onto master."""
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
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    pos_t, px, py, pz, pv = [], [], [], [], []
    diag_last: dict[str, str] = {}
    vel_t, vel = [], []
    for _ts, topic, typ, blob in iter_messages(args.result_bag):
        rec = decode_message(typ, blob)
        if rec is None:
            continue
        if topic == "/result/position":
            pos_t.append(rec["stamp_s"]); px.append(rec["s"]); py.append(rec["y"]); pz.append(rec["z"]); pv.append(rec["v"])
        elif topic == "/result/velocity":
            vel_t.append(rec["stamp_s"]); vel.append(rec["velocity"])
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
    rx, ry, rz, ok = base_link_xyz(g, rover, args.frame, (lat0, lon0, h0))
    g = g[ok]
    rx, ry, rz = rx[ok], ry[ok], rz[ok]
    if len(g) < 2:
        print("rover does not cover the master fixes", file=sys.stderr)
        return 2
    ref_v = np.full(len(g), np.nan)
    if "sensing_gnss_master_vel" in z:
        mv = z["sensing_gnss_master_vel"][np.argsort(z["sensing_gnss_master_vel"][:, 1])]
        ref_v = np.interp(g[:, 1], mv[:, 1], np.hypot(mv[:, 2], mv[:, 3]), left=np.nan, right=np.nan)
    after = g[:, 1] > t0 + args.window
    ref = {"t": g[after, 1], "x": rx[after], "y": ry[after], "z": rz[after], "v": ref_v[after]}
    est = {"t": np.array(pos_t), "x": np.array(px), "y": np.array(py), "z": np.array(pz), "v": np.array(pv)}
    pos = score(est, ref)
    vt = np.array(vel_t)
    vest = {"t": vt, "x": np.zeros(len(vt)), "y": np.zeros(len(vt)), "z": np.zeros(len(vt)), "v": np.array(vel)}
    vs = score(vest, ref)
    dt_out = np.diff(np.sort(vt))
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
        "rate_hz": float(1.0 / np.median(dt_out)) if len(dt_out) else float("nan"),
        "max_gap_s": float(dt_out.max()) if len(dt_out) else float("nan"),
        "coverage": pos["coverage"],
        "rmse_3d": pos["rmse_3d"],
        "rmse_x": pos["rmse_x"], "rmse_y": pos["rmse_y"], "rmse_z": pos["rmse_z"],
        "rmse_v_result_velocity": vs["rmse_v"],
        "rmse_v_odometry_twist": pos["rmse_v"],
        "frame": args.frame,
        "reference": "base_link",
        "base_frac": BASE_FRAC,
        "min_status": args.min_status,
        "n_fix": int(len(g)),
        "status_counts": status_counts,
        "end_err_m": end_err,
        "end_dx": end_x, "end_dy": end_y, "end_dz": end_z,
        "path_m": path,
        "end_drift_pct": 100.0 * end_err / path if path > 0 else float("nan"),
        "callback_max_us": float(diag_last.get("callback_max_us", "nan")),
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

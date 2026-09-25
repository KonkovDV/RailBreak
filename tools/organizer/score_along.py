"""Along-track wheel odometry on the train centerline.

GNSS is used once, at the first fix, to choose the branch and the origin of s.
After that only bogie speed is integrated. This is the init rule in the README,
not a GNSS filter.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_map import SHCH, KUL, _nearest  # noqa: E402
from judge import score  # noqa: E402
from reference import enu  # noqa: E402

FRONT = "vehicle_front_bogie_velocity"
REAR = "vehicle_rear_bogie_velocity"
KMH = 1.0 / 3.6


def _wheels(z):
    f = z[FRONT]
    r = z[REAR]
    f = f[np.argsort(f[:, 1])]
    r = r[np.argsort(r[:, 1])]
    rear = np.interp(f[:, 1], r[:, 1], r[:, 2], left=np.nan, right=np.nan)
    both = np.isfinite(rear)
    v = np.where(both, 0.5 * (f[:, 2] + rear), f[:, 2]) * KMH
    return f[:, 1], v


def _branch(x0, y0, lat0, lon0) -> str:
    kx, ky = enu(np.array([KUL[0]]), np.array([KUL[1]]), lat0, lon0)
    sx, sy = enu(np.array([SHCH[0]]), np.array([SHCH[1]]), lat0, lon0)
    dk = float(np.hypot(x0 - kx[0], y0 - ky[0]))
    ds = float(np.hypot(x0 - sx[0], y0 - sy[0]))
    return "SK" if ds <= dk else "KS"


def project_s(s, x, y, px, py) -> float:
    j, _ = _nearest(x, y, np.array([px]), np.array([py]))
    return float(s[int(j[0])])


def one(path: Path, center) -> dict | None:
    z = np.load(path)
    if "sensing_gnss_master_fix" not in z.files:
        return None
    g = z["sensing_gnss_master_fix"]
    g = g[np.argsort(g[:, 1])]
    g = g[g[:, 5] >= 0]
    if len(g) < 20 or FRONT not in z.files:
        return None
    lat0, lon0 = float(center["lat0"]), float(center["lon0"])
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    key = _branch(float(x[0]), float(y[0]), lat0, lon0)
    s = center[f"{key}_s"]
    cx, cy = center[f"{key}_x"], center[f"{key}_y"]
    h = center[f"{key}_h"]
    s0 = project_s(s, cx, cy, float(x[0]), float(y[0]))
    tw, vw = _wheels(z)
    dt = np.diff(tw)
    step = np.where((dt > 0) & (dt < 0.5), 0.5 * (vw[1:] + vw[:-1]) * dt, 0.0)
    sw = s0 + np.r_[0.0, np.cumsum(step)]
    sw = np.clip(sw, float(s[0]), float(s[-1]))
    # Jury frame: ENU of the reference's own first fix, not the map origin.
    lat_r, lon_r, alt_r = float(g[0, 2]), float(g[0, 3]), float(g[0, 4])
    rx, ry = enu(g[:, 2], g[:, 3], lat_r, lon_r)
    est_x = np.interp(sw, s, cx)
    est_y = np.interp(sw, s, cy)
    est_h = np.interp(sw, s, h)
    # Map coordinates are ENU about the map origin. The jury frame here is
    # ENU about the first fix, so subtract that origin expressed in the map.
    map_of_ref = enu(np.array([lat_r]), np.array([lon_r]), lat0, lon0)
    ex = np.interp(g[:, 1], tw, est_x) - float(map_of_ref[0][0])
    ey = np.interp(g[:, 1], tw, est_y) - float(map_of_ref[1][0])
    ez = np.interp(g[:, 1], tw, est_h) - alt_r
    ev = np.interp(g[:, 1], tw, vw, left=np.nan, right=np.nan)
    rv = np.full(len(g), np.nan)
    if "sensing_gnss_master_vel" in z.files:
        mv = z["sensing_gnss_master_vel"]
        mv = mv[np.argsort(mv[:, 1])]
        rv = np.interp(g[:, 1], mv[:, 1], np.hypot(mv[:, 2], mv[:, 3]), left=np.nan, right=np.nan)
    got = score(
        {"t": g[:, 1], "x": ex, "y": ey, "z": ez, "v": ev},
        {"t": g[:, 1], "x": rx, "y": ry, "z": g[:, 4] - alt_r, "v": rv},
    )
    j, dist = _nearest(cx, cy, x[::10], y[::10])
    return {
        "bag": path.stem,
        "branch": key,
        "rmse_3d": got["rmse_3d"],
        "rmse_v": got["rmse_v"],
        "xt_p50": float(np.median(dist)),
        "xt_p95": float(np.percentile(dist, 95)),
        "s0": s0,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("org", type=Path)
    ap.add_argument("splits", type=Path)
    ap.add_argument("center", type=Path)
    ap.add_argument("--split", default="val")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    center = np.load(args.center)
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    rows = [r for n in names if (r := one(args.org / f"{n}.npz", center))]
    rmse = np.array([r["rmse_3d"] for r in rows])
    xt = np.array([r["xt_p95"] for r in rows])
    summary = {
        "split": args.split,
        "n": len(rows),
        "rmse_3d_median": float(np.median(rmse)),
        "rmse_3d_p95": float(np.percentile(rmse, 95)),
        "xt_p95_median": float(np.median(xt)),
        "rows": rows,
    }
    if args.out:
        args.out.write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print(
        f"{args.split} n={len(rows)} rmse_3d med={summary['rmse_3d_median']:.2f} "
        f"p95={summary['rmse_3d_p95']:.2f} xt_p95 med={summary['xt_p95_median']:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

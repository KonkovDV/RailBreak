"""Compare the published GNSS speed reference with the along-track projection.

score_ros and eval_odometer use hypot(vx, vy). That module is non-negative,
folds in the cross-track component, and does not change sign on reverse.
The longitudinal speed on a known trajectory is v · t(s). This script measures
the gap on one split. It does not replace the published reference.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_odometer import MFIX, MVEL, build_ring, ring_init, track_reference  # noqa: E402
from reference import along_track_speed, enu, horizontal_speed  # noqa: E402


def tangent_at(ring, s_query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    s, x, y = ring["s"], ring["x"], ring["y"]
    j = np.searchsorted(s, np.asarray(s_query, float))
    j = np.clip(j, 5, len(s) - 6)
    return x[j + 5] - x[j - 5], y[j + 5] - y[j - 5]


def one(path: Path, cl, window: float) -> dict | None:
    z = np.load(path)
    if MFIX not in z.files or MVEL not in z.files or len(z[MVEL]) < 50:
        return None
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    ring = build_ring(cl, {"SK": [], "KS": []})
    init = ring_init(z, window, ring, lat0, lon0)
    if init is None:
        return None
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    s_ref, dist = track_reference(ring, x, y, init["s0"], t=g[:, 1])
    m = (dist < 3.0) & (g[:, 1] > init["t0"] + window)
    if m.sum() < 100:
        return None
    mv = z[MVEL][np.argsort(z[MVEL][:, 1])]
    vx = np.interp(g[m, 1], mv[:, 1], mv[:, 2], left=np.nan, right=np.nan)
    vy = np.interp(g[m, 1], mv[:, 1], mv[:, 3], left=np.nan, right=np.nan)
    ok = np.isfinite(vx) & np.isfinite(vy)
    if ok.sum() < 100:
        return None
    tx, ty = tangent_at(ring, s_ref[m][ok])
    mod = horizontal_speed(vx[ok], vy[ok])
    par = along_track_speed(vx[ok], vy[ok], tx, ty)
    gap = mod - par
    return {
        "n": int(ok.sum()),
        "gap_rmse": float(np.sqrt(np.mean(gap * gap))),
        "gap_med": float(np.median(np.abs(gap))),
        "gap_p95": float(np.percentile(np.abs(gap), 95)),
        "excess_med": float(np.median(mod - np.abs(par))),
        "reverse": float(np.mean(par < -0.11)),
        "large": float(np.mean(np.abs(gap) > 0.5)),
        "align": float(np.median((vx[ok] * tx + vy[ok] * ty) / (np.hypot(vx[ok], vy[ok]) * np.hypot(tx, ty) + 1e-9))),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org", type=Path, default=Path("local/org"))
    ap.add_argument("--splits", type=Path, default=Path("local/splits.json"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--split", default="val")
    ap.add_argument("--gnss-window", type=float, default=3.0)
    args = ap.parse_args()
    cl = np.load(args.map)
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    rows = []
    for n in names:
        got = one(args.org / f"{n}.npz", cl, args.gnss_window)
        if got is None:
            print(f"  {n} skipped")
            continue
        rows.append(got)
        print(
            f"  {n} n={got['n']} gap_rmse={got['gap_rmse']:.4f} "
            f"|gap|_med={got['gap_med']:.4f} p95={got['gap_p95']:.4f} "
            f"excess_med={got['excess_med']:.4f} reverse={got['reverse']:.4f} "
            f"large={got['large']:.4f} align={got['align']:.3f}"
        )
    if not rows:
        print("no recordings")
        return 1
    def col(k):
        return np.array([r[k] for r in rows], float)
    print(f"\n{args.split} recordings={len(rows)}")
    print(f"gap_rmse median={np.median(col('gap_rmse')):.4f} m/s  mean={col('gap_rmse').mean():.4f}")
    print(f"|gap| med median={np.median(col('gap_med')):.4f} m/s  p95 median={np.median(col('gap_p95')):.4f}")
    print(f"excess_med median={np.median(col('excess_med')):.4f} m/s")
    print(f"reverse fraction median={np.median(col('reverse')):.4f} max={col('reverse').max():.4f}")
    print(f"|gap|>0.5 fraction median={np.median(col('large')):.5f} max={col('large').max():.5f}")
    print(f"align median={np.median(col('align')):.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

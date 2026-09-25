"""Score two baselines on a split. GNSS is the reference, not a filter input.

speed: mean of the two bogies, divided by 3.6, against |GNSS velocity|.
along: integrate that speed and compare path length to the GNSS chord-free
path. This is not the jury 3D metric; it is the drift the map has to remove.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from judge import score  # noqa: E402
from reference import master_reference  # noqa: E402

FRONT = "vehicle_front_bogie_velocity"
REAR = "vehicle_rear_bogie_velocity"
KMH = 1.0 / 3.6


def _series(z, key):
    a = z[key]
    o = np.argsort(a[:, 1])
    return a[o, 1], a[o, 2]


def wheel_mps(z) -> tuple[np.ndarray, np.ndarray] | None:
    if FRONT not in z.files or REAR not in z.files:
        return None
    tf, vf = _series(z, FRONT)
    tr, vr = _series(z, REAR)
    if len(tf) < 5:
        return None
    rear = np.interp(tf, tr, vr, left=np.nan, right=np.nan)
    both = np.isfinite(rear)
    v = np.where(both, 0.5 * (vf + rear), vf) * KMH
    return tf, v


def along_error(t, v, ref) -> float | None:
    dt = np.diff(t)
    ok = (dt > 0) & (dt < 0.5)
    if ok.sum() < 10:
        return None
    s = np.r_[0.0, np.cumsum(np.where(ok, 0.5 * (v[1:] + v[:-1]) * dt, 0.0))]
    x, y = ref["x"], ref["y"]
    step = np.hypot(np.diff(x), np.diff(y))
    step = np.where(step < 30.0, step, 0.0)
    path = float(np.r_[0.0, np.cumsum(step)][-1])
    return float(s[-1] - path)


def one(path: Path) -> dict | None:
    z = np.load(path)
    ref = master_reference(z)
    wheels = wheel_mps(z)
    if ref is None or wheels is None:
        return None
    t, v = wheels
    vi = np.interp(ref["t"], t, v, left=np.nan, right=np.nan)
    est = {
        "t": ref["t"],
        "x": ref["x"],
        "y": ref["y"],
        "z": ref["z"],
        "v": vi,
    }
    # Position copied from the reference: this row is a speed score only.
    spd = score(est, ref)
    return {
        "bag": path.stem,
        "rmse_v": spd["rmse_v"],
        "coverage": spd["coverage"],
        "along_m": along_error(t, v, ref),
        "path_m": float(np.hypot(ref["x"][-1] - ref["x"][0], ref["y"][-1] - ref["y"][0])),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("org", type=Path)
    ap.add_argument("splits", type=Path)
    ap.add_argument("--split", default="val")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    rows = []
    for name in names:
        rec = one(args.org / f"{name}.npz")
        if rec:
            rows.append(rec)
    v = np.array([r["rmse_v"] for r in rows], float)
    a = np.array([r["along_m"] for r in rows if r["along_m"] is not None], float)
    summary = {
        "split": args.split,
        "n": len(rows),
        "rmse_v_median": float(np.nanmedian(v)) if len(v) else None,
        "rmse_v_p95": float(np.nanpercentile(v, 95)) if len(v) else None,
        "along_m_median": float(np.median(a)) if len(a) else None,
        "along_m_p95": float(np.percentile(np.abs(a), 95)) if len(a) else None,
        "rows": rows,
    }
    text = json.dumps(summary, indent=1)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(
        f"{args.split} n={summary['n']} rmse_v med={summary['rmse_v_median']:.4f} "
        f"p95={summary['rmse_v_p95']:.4f} along_m med={summary['along_m_median']:.2f} "
        f"|along| p95={summary['along_m_p95']:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

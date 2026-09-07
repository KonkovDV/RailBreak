"""Guess drive jerk limit and PT1 lag from the first accel after a dwell.

Does not import UKF. Fit on a dry departure (notch step, low v), not during slip.

Usage:
  python tools/eval/identify_jerk.py synth/runs/jagged_notch/run.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams  # noqa: E402

DT = 0.02


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _v_wheels(row: dict, r0: float) -> float:
    ws = []
    for k in ("w0", "w1", "w2", "w3"):
        if k in row and row[k] not in (None, ""):
            w = float(row[k])
            if math.isfinite(w):
                ws.append(w)
    if not ws:
        return 0.0
    return r0 * sum(ws) / len(ws)


def identify(rows: list[dict], *, dt_s: float = DT, p: PlantParams | None = None) -> dict:
    """First notch step after |v|<0.3: a_wheels vs time → j_max, tau.

    Algebraic F=F* has a jump. A ramp of duration T at a_ss gives j_max≈a_ss/T.
    A PT1 has 63% rise at t=τ. We report both; YAML defaults stay 0 until the bag.
    """
    p = p or PlantParams()
    out = {
        "n_samples": 0,
        "j_max_mps3": 0.0,
        "tau_drv_s": 0.0,
        "a_ss_mps2": None,
        "t_10_90_s": None,
        "note": "first accel after dwell; paste j_max_mps3 / tau_drv_s. Not a filter update.",
    }
    v_prev = None
    dwell = True
    capturing = False
    acc: list[tuple[float, float]] = []  # (t_rel, a)
    t_rel = 0.0
    for row in rows:
        notch = float(row.get("notch") or 0.0)
        brake = float(row.get("brake") or 0.0)
        v = _v_wheels(row, p.r0_m)
        if row.get("gt_v") not in (None, ""):
            try:
                v = float(row["gt_v"])
            except (TypeError, ValueError):
                pass
        if v_prev is None:
            v_prev = v
            continue
        a = (v - v_prev) / dt_s
        v_prev = v
        if not capturing:
            if v < 0.3 and abs(notch) < 0.05 and brake < 0.15:
                dwell = True
            if dwell and notch > 0.4 and brake < 0.05:
                capturing = True
                t_rel = 0.0
                acc = []
            continue
        t_rel += dt_s
        if t_rel > 4.0 or brake > 0.15 or notch < 0.2:
            break
        acc.append((t_rel, a))
    out["n_samples"] = len(acc)
    if len(acc) < 8:
        out["note"] = "no first-accel window; keep j_max_mps3: 0 and tau_drv_s: 0"
        return out
    a_vals = [x[1] for x in acc]
    a_ss = max(a_vals)
    out["a_ss_mps2"] = a_ss
    if a_ss < 0.2:
        out["note"] = "peak a too small; not a traction ramp"
        return out
    lo, hi = 0.1 * a_ss, 0.9 * a_ss
    t_lo = next((t for t, a in acc if a >= lo), None)
    t_hi = next((t for t, a in acc if a >= hi), None)
    if t_lo is None or t_hi is None or t_hi <= t_lo:
        out["note"] = "could not bracket 10–90% rise"
        return out
    t_rise = t_hi - t_lo
    out["t_10_90_s"] = t_rise
    # 10–90% of a linear ramp is 0.8 T_full; j = a_ss / T_full.
    t_full = t_rise / 0.8
    out["j_max_mps3"] = round(a_ss / max(t_full, 1e-3), 3)
    # PT1 10–90% is ln(9) τ ≈ 2.2 τ.
    out["tau_drv_s"] = round(t_rise / 2.2, 3)
    out["note"] = (
        "j_max is a rate cap (not PT1). If the bag is algebraic, keep both at 0. "
        "Paste under ros__parameters after 25.09."
    )
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Identify j_max / tau from first accel")
    ap.add_argument("csv", type=Path)
    args = ap.parse_args(argv)
    if not args.csv.is_file():
        sys.stderr.write(f"missing {args.csv}\n")
        return 1
    out = identify(_rows(args.csv))
    for k, v in out.items():
        print(f"{k}: {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Identify Davis A_d and F_max from a coast / accel segment.

Calibrate on coast, not during slip. Does not import UKF.

Usage:
  python tools/eval/identify_coast.py synth/runs/coast_no_wire/run.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, davis_resistance_n  # noqa: E402

R0 = 0.35
DT = 0.02


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _v_wheel(row: dict, r0: float) -> float:
    ws = []
    for k in ("w0", "w1", "w2", "w3"):
        if k in row and row[k] not in (None, ""):
            ws.append(float(row[k]))
    if not ws:
        return 0.0
    return r0 * sum(ws) / len(ws)


def _mean_omega(row: dict) -> float:
    ws = []
    for k in ("w0", "w1", "w2", "w3"):
        if k in row and row[k] not in (None, ""):
            w = float(row[k])
            if math.isfinite(w):
                ws.append(w)
    if not ws:
        return 0.0
    return sum(ws) / len(ws)


def _r0_from_gt(rows: list[dict], dt_s: float) -> float | None:
    """r0 = ∫ v_gt dt / ∫ ω̄ dt on coast. Needs a gt_v column."""
    if not rows or "gt_v" not in rows[0]:
        return None
    num = 0.0
    den = 0.0
    for row in rows:
        try:
            v = float(row["gt_v"])
        except (KeyError, TypeError, ValueError):
            continue
        notch = float(row.get("notch") or 0.0)
        brake = float(row.get("brake") or 0.0)
        wbar = _mean_omega(row)
        if abs(notch) < 0.05 and brake < 0.1 and v > 1.0 and wbar > 0.1:
            num += v * dt_s
            den += wbar * dt_s
    if den < 1e-3:
        return None
    return num / den


def identify(rows: list[dict], *, dt_s: float = DT, p: PlantParams | None = None) -> dict:
    p = p or PlantParams()
    r0 = p.r0_m
    a_vals: list[float] = []
    fmax_vals: list[float] = []
    v_prev = None
    for row in rows:
        notch = float(row.get("notch") or 0.0)
        brake = float(row.get("brake") or 0.0)
        v = _v_wheel(row, r0)
        if v_prev is None:
            v_prev = v
            continue
        a = (v - v_prev) / dt_s
        v_prev = v
        if abs(notch) < 0.05 and brake < 0.1 and v > 1.0:
            # a ≈ −(A + B v + C v²) / m  →  A = −m a − B v − C v²
            a_hat = -p.m0_kg * a - p.B_d * abs(v) - p.C_d * v * v
            if 0.0 < a_hat < 5000.0:
                a_vals.append(a_hat)
        if notch >= 0.75 and brake < 0.05 and v > 0.5:
            f = p.m0_kg * a + davis_resistance_n(v, p)
            if f > 0.0:
                fmax_vals.append(f)
    out = {
        "n_coast": len(a_vals),
        "n_accel": len(fmax_vals),
        "A_d": p.A_d,
        "F_max_n": p.m0_kg * p.a_trac_max,
        "a_trac_max": p.a_trac_max,
    }
    if a_vals:
        out["A_d"] = float(statistics.median(a_vals))
    if fmax_vals:
        out["F_max_n"] = float(statistics.median(fmax_vals))
        out["a_trac_max"] = out["F_max_n"] / p.m0_kg
    r0_hat = _r0_from_gt(rows, dt_s)
    if r0_hat is not None:
        out["wheel_radius_m"] = r0_hat
    out["note"] = (
        "coast/accel median; Combino defaults if n_coast=0. "
        "Keys A_d / a_trac_max are ROS params on the node."
    )
    return out


def format_yaml(est: dict) -> str:
    lines = [
        f"# identify_coast.py {est['note']}",
        f"# n_coast={est['n_coast']} n_accel={est['n_accel']}",
        "# Paste under ros__parameters in estimator.yaml. Not 71-931M.",
        f"    A_d: {est['A_d']:.1f}",
        f"    a_trac_max: {est['a_trac_max']:.3f}",
    ]
    if "wheel_radius_m" in est:
        lines.append(f"    wheel_radius_m: {est['wheel_radius_m']:.4f}")
        lines.append("    r0_uncalibrated: false")
    lines.append("    # tau_drv_s: 0.0")
    lines.append("    # gamma_rot: 0.0")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path, help="run.csv or filter.csv")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if not args.csv.is_file():
        sys.stderr.write(f"missing {args.csv}\n")
        return 1
    rows = _rows(args.csv)
    if not rows:
        sys.stderr.write("empty csv\n")
        return 1
    est = identify(rows)
    sys.stdout.write(format_yaml(est))
    sys.stdout.write(f"# A_d={est['A_d']:.1f} N  F_max={est['F_max_n']:.0f} N  "
                     f"a_trac={est['a_trac_max']:.3f} m/s^2\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

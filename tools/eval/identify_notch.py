"""Guess whether notch is a force command or a closed-loop accel command.

Does not import UKF. Calibrate on a dry accel at low speed, not during slip.

Usage:
  python tools/eval/identify_notch.py synth/runs/jagged_notch/run.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, davis_resistance_n  # noqa: E402

DT = 0.02


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _v(row: dict, r0: float) -> float:
    if row.get("gt_v") not in (None, ""):
        try:
            return float(row["gt_v"])
        except (TypeError, ValueError):
            pass
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
    """OLS a ≈ c * n + b at |v|<v_base. Closed-loop accel keeps c ≈ a_trac independent of m.

    Also estimates the power-hyperbola knee v_b from samples above the default base.
    """
    p = p or PlantParams()
    xs: list[float] = []
    ys: list[float] = []
    high: list[tuple[float, float, float]] = []  # v, a, n
    v_prev = None
    for row in rows:
        notch = float(row.get("notch") or 0.0)
        brake = float(row.get("brake") or 0.0)
        v = _v(row, p.r0_m)
        if v_prev is None:
            v_prev = v
            continue
        a = (v - v_prev) / dt_s
        v_prev = v
        if abs(notch) < 0.3 or brake > 0.05:
            continue
        if 0.8 <= v <= p.v_base_mps:
            xs.append(notch)
            ys.append(a)
        if v > p.v_base_mps + 0.5 and notch > 0.4:
            high.append((v, a, notch))
    out = {
        "n_samples": len(xs),
        "n_high_v": len(high),
        "notch_as_accel": False,
        "c_mps2": None,
        "b_mps2": None,
        "a_trac_max": p.a_trac_max,
        "v_base_mps": p.v_base_mps,
        "note": "low-v OLS; paste notch_as_accel / v_base_mps under ros__parameters. Not a filter update.",
    }

    def _fit_knee() -> None:
        if len(high) < 12:
            return
        knees: list[float] = []
        for v, a, n in high:
            a_net = a + p.A_d / p.m0_kg + (p.B_d * v + p.C_d * v * v) / p.m0_kg
            denom = n * max(p.a_trac_max, 1e-6)
            if a_net <= 0.05 or denom < 1e-6:
                continue
            knees.append(v * a_net / denom)
        if len(knees) >= 8:
            knees.sort()
            out["v_base_mps"] = knees[len(knees) // 2]
            out["note"] = out["note"] + f" knee from {len(knees)} high-v samples."

    if len(xs) < 12:
        out["note"] = "too few low-v accel samples; keep notch_as_accel: false"
        _fit_knee()
        return out
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    den = sum((x - mx) ** 2 for x in xs)
    if den < 1e-6:
        out["note"] = "notch did not vary; keep notch_as_accel: false"
        _fit_knee()
        return out
    c = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
    b = my - c * mx
    out["c_mps2"] = c
    out["b_mps2"] = b
    davis_b = -p.A_d / p.m0_kg
    slope_ok = abs(c - p.a_trac_max) < 0.25 * p.a_trac_max
    intercept_ok = abs(b - davis_b) < 0.15
    out["notch_as_accel"] = bool(slope_ok and intercept_ok)
    _fit_knee()
    return out


def format_yaml(est: dict) -> str:
    flag = "true" if est.get("notch_as_accel") else "false"
    c = est.get("c_mps2")
    b = est.get("b_mps2")
    lines = [
        f"# identify_notch.py {est['note']}",
        f"# n_samples={est['n_samples']}",
    ]
    if c is not None:
        lines.append(f"# a ≈ {c:.3f} * n + {b:.3f}  (a_trac_max={est['a_trac_max']})")
    vb = est.get("v_base_mps")
    if vb is not None:
        lines.append(f"    v_base_mps: {float(vb):.2f}")
    lines.append(f"    notch_as_accel: {flag}")
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

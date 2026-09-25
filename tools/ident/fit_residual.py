"""Offline fit of the linear-in-parameters residual force (M4).

Writes residual_model.yaml that C++ ResidualModel can load. Default-off.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, traction_star_n, davis_resistance_n  # noqa: E402

KNOTS = (0.0, 0.0, 0.0, 0.0, 2.0, 5.0, 10.0, 15.0, 20.0, 20.0, 20.0, 20.0)
N_SPLINE = 8
N_COEFF = 34


def bsplines(v: float) -> list[float]:
    x = min(20.0, max(0.0, v))
    n0 = [1.0 if KNOTS[i] <= x < KNOTS[i + 1] else 0.0 for i in range(11)]
    if x >= 20.0:
        n0 = [0.0] * 11
        n0[7] = 1.0
    cur = n0 + [0.0]
    for p in range(1, 4):
        nxt = [0.0] * 12
        nbasis = 12 - p - 1
        for i in range(nbasis):
            d1 = KNOTS[i + p] - KNOTS[i]
            d2 = KNOTS[i + p + 1] - KNOTS[i + 1]
            a = ((x - KNOTS[i]) / d1 * cur[i]) if d1 > 0 else 0.0
            c = ((KNOTS[i + p + 1] - x) / d2 * cur[i + 1]) if d2 > 0 else 0.0
            nxt[i] = a + c
        cur = nxt
    b = cur[:N_SPLINE]
    if x >= 20.0:
        b = [0.0] * N_SPLINE
        b[-1] = 1.0
    return b


def basis(v: float, n: float, b: float, n_lag: float, b_lag: float) -> list[float]:
    spl = bsplines(v)
    n_pos = max(n, 0.0)
    n_neg = max(-n, 0.0)
    br = min(max(b, 0.0), 1.0)
    phi = []
    for g in (1.0, n_pos, n_neg, br):
        phi.extend(s * g for s in spl)
    phi.append(n_lag)
    phi.append(b_lag)
    return phi


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(row: dict, key: str) -> float:
    raw = row.get(key)
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return float("nan")
    return v


def savgol_a(v: list[float], dt: float, win: int = 51) -> list[float]:
    half = win // 2
    out = [0.0] * len(v)
    for i in range(len(v)):
        a = max(0, i - half)
        b = min(len(v), i + half + 1)
        if b - a < 5:
            continue
        # local linear slope
        ts = [(j - i) * dt for j in range(a, b)]
        vs = v[a:b]
        n = len(ts)
        mt = sum(ts) / n
        mv = sum(vs) / n
        num = sum((t - mt) * (y - mv) for t, y in zip(ts, vs))
        den = sum((t - mt) ** 2 for t in ts) or 1.0
        out[i] = num / den
    return out


def fit(path: Path, mass: float = 28000.0, lam: float = 10.0,
        r0_m: float = 0.35) -> dict:
    rows = _rows(path)
    v = [_f(r, "gt_v") for r in rows]
    a = savgol_a([0.0 if math.isnan(x) else x for x in v], 0.02)
    p = PlantParams()
    p.r0_m = r0_m
    p.m0_kg = mass
    phi_rows = []
    y = []
    n_lag = 0.0
    b_lag = 0.0
    for i, r in enumerate(rows):
        n = _f(r, "notch")
        b = _f(r, "brake")
        if math.isnan(v[i]) or v[i] < 0.2 or v[i] > 20.0:
            n_lag, b_lag = max(n, 0.0) if math.isfinite(n) else 0.0, max(b, 0.0) if math.isfinite(b) else 0.0
            continue
        wmean = []
        for k in ("w0", "w1", "w2", "w3"):
            ww = _f(r, k)
            if math.isfinite(ww):
                wmean.append(ww)
        if wmean:
            v_wh = r0_m * sum(wmean) / len(wmean)
            if abs(v_wh - v[i]) > 0.4:
                continue
        f_star = traction_star_n(n if math.isfinite(n) else 0.0, v[i], p)
        br = b if math.isfinite(b) else 0.0
        f_brake = 0.0 if abs(v[i]) < p.v_eps else max(0.0, min(1.0, br)) * mass * p.a_svc * (1.0 if v[i] >= 0 else -1.0)
        f_run = davis_resistance_n(v[i], p)
        f_phys = f_star - f_brake - f_run
        r_k = mass * a[i] - f_phys
        phi = basis(v[i], n if math.isfinite(n) else 0.0, b if math.isfinite(b) else 0.0, n_lag, b_lag)
        phi_rows.append(phi)
        y.append(r_k)
        n_lag = max(n, 0.0) if math.isfinite(n) else 0.0
        b_lag = max(b, 0.0) if math.isfinite(b) else 0.0
    m = N_COEFF
    xtx = [[0.0] * m for _ in range(m)]
    xty = [0.0] * m
    for phi, yy in zip(phi_rows, y):
        for i in range(m):
            xty[i] += phi[i] * yy
            for j in range(m):
                xtx[i][j] += phi[i] * phi[j]
    for i in range(m):
        xtx[i][i] += lam
    # First-difference penalty on each of the four spline gates (P-spline).
    lam_d = 1.0
    for g in range(4):
        for i in range(N_SPLINE - 1):
            a = g * N_SPLINE + i
            b = a + 1
            xtx[a][a] += lam_d
            xtx[b][b] += lam_d
            xtx[a][b] -= lam_d
            xtx[b][a] -= lam_d
    # Gauss-Jordan
    aug = [xtx[i][:] + [xty[i]] for i in range(m)]
    for i in range(m):
        piv = max(range(i, m), key=lambda r: abs(aug[r][i]))
        aug[i], aug[piv] = aug[piv], aug[i]
        den = aug[i][i] or 1e-12
        for j in range(i, m + 1):
            aug[i][j] /= den
        for r in range(m):
            if r == i:
                continue
            fac = aug[r][i]
            for j in range(i, m + 1):
                aug[r][j] -= fac * aug[i][j]
    theta = [aug[i][m] for i in range(m)]
    rss = 0.0
    for phi, yy in zip(phi_rows, y):
        pred = sum(p * t for p, t in zip(phi, theta))
        rss += (yy - pred) ** 2
    sigma2 = rss / max(len(y) - 1, 1)
    return {"theta": theta, "n": len(y), "sigma2": sigma2, "lambda": lam, "enabled": True}


def to_yaml(fit_d: dict) -> str:
    lines = ["residual:", "  enabled: true", f"  n: {N_COEFF}", f"  sigma2: {fit_d['sigma2']:.6g}",
             "  theta: [" + ", ".join(f"{t:.8g}" for t in fit_d["theta"]) + "]"]
    return "\n".join(lines) + "\n"


def eval_force(theta: list[float], v: float, n: float, b: float) -> float:
    phi = basis(v, n, b, 0.0, 0.0)
    return sum(p * t for p, t in zip(phi, theta))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--mass", type=float, default=28000.0)
    args = ap.parse_args(argv)
    d = fit(args.csv, args.mass)
    args.out.write_text(to_yaml(d), encoding="utf-8")
    args.out.with_suffix(".json").write_text(json.dumps(d, indent=2), encoding="utf-8")
    print(f"residual n={d['n']} sigma2={d['sigma2']:.4g} -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

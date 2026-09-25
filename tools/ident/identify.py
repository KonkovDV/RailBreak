"""Identification v2: radii, Davis coast, traction map, lag, a_svc, jerk.

Fits on generator / GT columns. Does not import the UKF. Output is a YAML
snippet plus bootstrap intervals on the recovered scalars.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, davis_resistance_n  # noqa: E402

DT = 0.02
G = 9.81


def _rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(row: dict, key: str) -> float | None:
    raw = row.get(key)
    if raw in (None, ""):
        return None
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _v_gt(row: dict) -> float | None:
    return _f(row, "gt_v")


def _a_series(rows: list[dict], dt: float) -> list[float]:
    planted = [_f(r, "a_mps2") for r in rows]
    if sum(1 for a in planted if a is not None) >= max(10, len(rows) // 2):
        return [0.0 if a is None else a for a in planted]
    v = [_v_gt(r) for r in rows]
    half = 5
    a = [0.0] * len(v)
    for i in range(len(v)):
        ts: list[float] = []
        vs: list[float] = []
        lo = max(0, i - half)
        hi = min(len(v), i + half + 1)
        for j in range(lo, hi):
            if v[j] is None or v[i] is None:
                continue
            if abs(v[j] - v[i]) > 0.5:
                continue
            ts.append((j - i) * dt)
            vs.append(v[j])
        if len(vs) < 5:
            continue
        mt = sum(ts) / len(ts)
        mv = sum(vs) / len(vs)
        num = sum((t - mt) * (y - mv) for t, y in zip(ts, vs))
        den = sum((t - mt) ** 2 for t in ts) or 1.0
        a[i] = num / den
    return a


def _solve3(xtx: list[list[float]], xty: list[float]) -> tuple[float, float, float]:
    m = [row[:] + [bb] for row, bb in zip(xtx, xty)]
    for i in range(3):
        piv = max(range(i, 3), key=lambda r: abs(m[r][i]))
        m[i], m[piv] = m[piv], m[i]
        den = m[i][i] or 1e-12
        for j in range(i, 4):
            m[i][j] /= den
        for r in range(3):
            if r == i:
                continue
            fac = m[r][i]
            for j in range(i, 4):
                m[r][j] -= fac * m[i][j]
    return m[0][3], m[1][3], m[2][3]


def identify_davis(rows: list[dict], a: list[float], mass: float, gamma: float,
                   seed: int = 42) -> dict:
    m_dyn = mass * (1.0 + gamma)
    xs, ys = [], []
    last_cmd = -1e9
    for i, r in enumerate(rows):
        n = _f(r, "notch") or 0.0
        b = _f(r, "brake") or 0.0
        v = _v_gt(r)
        t = i * DT
        if abs(n) > 0.05 or b > 0.05:
            last_cmd = t
            continue
        if t - last_cmd < 1.0:
            continue
        if v is None or abs(v) < 1.0:
            continue
        if i > 0:
            v_prev = _v_gt(rows[i - 1])
            if v_prev is not None and abs(v - v_prev) > 0.4:
                last_cmd = t
                continue
        xs.append((1.0, abs(v), v * v))
        ys.append(-m_dyn * a[i] * (1.0 if v >= 0 else -1.0))
    if len(xs) < 10:
        nan = float("nan")
        return {"A_d": nan, "B_d": nan, "C_d": nan,
                "A_d_p05": nan, "A_d_p95": nan,
                "B_d_p05": nan, "B_d_p95": nan,
                "C_d_p05": nan, "C_d_p95": nan}
    xtx = [[0.0] * 3 for _ in range(3)]
    xty = [0.0] * 3
    for x, y in zip(xs, ys):
        for i in range(3):
            xty[i] += x[i] * y
            for j in range(3):
                xtx[i][j] += x[i] * x[j]
    A, B, C = _solve3(xtx, xty)
    rng = random.Random(seed)
    As: list[float] = []
    Bs: list[float] = []
    Cs: list[float] = []
    n = len(xs)
    for _ in range(200):
        xtx_b = [[0.0] * 3 for _ in range(3)]
        xty_b = [0.0] * 3
        for _k in range(n):
            idx = rng.randrange(n)
            x, y = xs[idx], ys[idx]
            for i in range(3):
                xty_b[i] += x[i] * y
                for j in range(3):
                    xtx_b[i][j] += x[i] * x[j]
        a_b, b_b, c_b = _solve3(xtx_b, xty_b)
        As.append(max(a_b, 0.0))
        Bs.append(max(b_b, 0.0))
        Cs.append(max(c_b, 0.0))
    As.sort()
    Bs.sort()
    Cs.sort()
    k05 = int(0.05 * (len(As) - 1))
    k95 = int(0.95 * (len(As) - 1))
    # Pairs bootstrap on a noiseless twin is overconfident: samples are
    # strongly dependent and the OLS residual is discretisation, not noise.
    # Overbound the percentile interval by 2 % relative (identification
    # analogue of DeCleene κ_ob). Point estimates stay the OLS values.
    rel = 0.02

    def _widen(hat: float, lo: float, hi: float) -> tuple[float, float]:
        floor = rel * max(abs(hat), 1.0)
        return min(lo, hat - floor), max(hi, hat + floor)

    a_lo, a_hi = _widen(max(A, 0.0), As[k05], As[k95])
    b_lo, b_hi = _widen(max(B, 0.0), Bs[k05], Bs[k95])
    c_lo, c_hi = _widen(max(C, 0.0), Cs[k05], Cs[k95])
    return {
        "A_d": max(A, 0.0),
        "B_d": max(B, 0.0),
        "C_d": max(C, 0.0),
        "A_d_p05": a_lo,
        "A_d_p95": a_hi,
        "B_d_p05": b_lo,
        "B_d_p95": b_hi,
        "C_d_p05": c_lo,
        "C_d_p95": c_hi,
        "n_coast": n,
        "ident_overbound_rel": rel,
    }


def identify_radii(rows: list[dict]) -> dict[str, float]:
    out = {}
    for i in range(6):
        ratios = []
        for r in rows:
            v = _v_gt(r)
            w = _f(r, f"w{i}")
            if v is None or w is None or abs(w) < 0.2 or abs(v) < 0.5:
                continue
            ratios.append(v / w)
        if ratios:
            out[f"r{i}"] = statistics.median(ratios)
    if out:
        out["r0_mean"] = statistics.mean(out.values())
    return out


def identify_atrac(rows: list[dict], a: list[float], mass: float, p: PlantParams) -> list[float]:
    acc = []
    for i, r in enumerate(rows):
        n = _f(r, "notch") or 0.0
        v = _v_gt(r)
        if n < 0.4 or v is None or v > 5.0:
            continue
        R = davis_resistance_n(v, p)
        acc.append((mass * a[i] + R) / max(n, 1e-3) / mass)
    return acc


def identify_asvc(rows: list[dict], a: list[float], mass: float, p: PlantParams) -> list[float]:
    acc = []
    for i, r in enumerate(rows):
        b = _f(r, "brake") or 0.0
        n = _f(r, "notch") or 0.0
        v = _v_gt(r)
        if b < 0.4 or abs(n) > 0.05 or v is None or abs(v) < 1.0:
            continue
        R = abs(davis_resistance_n(v, p))
        acc.append((mass * (-a[i]) - R) / (b * mass))
    return acc


def identify_lag(rows: list[dict], a: list[float], dt: float) -> dict:
    n = [_f(r, "notch") or 0.0 for r in rows]
    best_d, best = 0, -1.0
    for d in range(0, 25):
        num = den_a = den_n = 0.0
        for i in range(d, len(a)):
            num += a[i] * n[i - d]
            den_a += a[i] * a[i]
            den_n += n[i - d] * n[i - d]
        if den_a > 0 and den_n > 0:
            c = num / math.sqrt(den_a * den_n)
            if c > best:
                best, best_d = c, d
    return {"T_d_s": best_d * dt, "corr": best}


def identify_jmax(rows: list[dict], a: list[float], dt: float) -> float:
    da = [(a[i] - a[i - 1]) / dt for i in range(1, len(a))]
    pos = [abs(x) for x in da if math.isfinite(x)]
    if len(pos) < 10:
        return float("nan")
    pos.sort()
    return pos[int(0.95 * (len(pos) - 1))]


def bootstrap_scalar(values: list[float], seed: int = 42, n: int = 200) -> dict:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return {"median": float("nan"), "p05": float("nan"), "p95": float("nan")}
    rng = random.Random(seed)
    dist = []
    for _ in range(n):
        sample = [rng.choice(finite) for _ in finite]
        dist.append(statistics.median(sample))
    dist.sort()
    return {
        "median": statistics.median(finite),
        "p05": dist[int(0.05 * (len(dist) - 1))],
        "p95": dist[int(0.95 * (len(dist) - 1))],
    }


def identify(path: Path, mass: float = 28000.0, gamma: float = 0.0) -> dict:
    rows = _rows(path)
    dt = DT
    a = _a_series(rows, dt)
    p = PlantParams()
    radii = identify_radii(rows)
    davis = identify_davis(rows, a, mass, gamma)
    p.A_d = davis.get("A_d") or p.A_d
    p.B_d = davis.get("B_d") or p.B_d
    p.C_d = davis.get("C_d") or p.C_d
    atrac_s = identify_atrac(rows, a, mass, p)
    asvc_s = identify_asvc(rows, a, mass, p)
    atrac = bootstrap_scalar(atrac_s)
    asvc = bootstrap_scalar(asvc_s)
    lag = identify_lag(rows, a, dt)
    jmax = identify_jmax(rows, a, dt)
    return {
        "radii": radii,
        "davis": davis,
        "a_trac_max": atrac["median"],
        "a_trac_max_p05": atrac["p05"],
        "a_trac_max_p95": atrac["p95"],
        "a_svc": asvc["median"],
        "a_svc_p05": asvc["p05"],
        "a_svc_p95": asvc["p95"],
        "lag": lag,
        "j_max_mps3": jmax,
        "mass_kg": mass,
        "gamma_rot": gamma,
    }


def to_yaml(est: dict) -> str:
    lines = ["/**:", "  ros__parameters:"]
    r0 = est.get("radii", {}).get("r0_mean")
    if r0 and math.isfinite(r0):
        lines.append(f"    wheel_radius_m: {r0:.5f}")
        lines.append("    r0_uncalibrated: false")
    d = est.get("davis", {})
    for k in ("A_d", "B_d", "C_d"):
        if math.isfinite(d.get(k, float("nan"))):
            lines.append(f"    {k}: {d[k]:.3f}")
            lo, hi = d.get(f"{k}_p05"), d.get(f"{k}_p95")
            if lo is not None and hi is not None and math.isfinite(lo) and math.isfinite(hi):
                lines.append(f"    # {k} bootstrap 5–95: [{lo:.3f}, {hi:.3f}]")
    if math.isfinite(est.get("a_trac_max", float("nan"))):
        lines.append(f"    a_trac_max: {est['a_trac_max']:.4f}")
    if math.isfinite(est.get("a_svc", float("nan"))):
        lines.append(f"    a_svc: {est['a_svc']:.4f}")
    if math.isfinite(est.get("j_max_mps3", float("nan"))):
        lines.append(f"    j_max_mps3: {est['j_max_mps3']:.4f}")
    td = est.get("lag", {}).get("T_d_s")
    if td is not None and math.isfinite(td):
        lines.append(f"    # T_d_s: {td:.3f} (command delay, not a plant field)")
    return "\n".join(lines) + "\n"


# Gates for overlay into replay_ukf. A 30 s traction clip can report
# j_max ~ 0.05 m/s³; loading that PT1/jerk limit starves force and DEGRADEs.
REPLAY_GATES = {
    "r0": (0.25, 0.45),
    "a_trac_max": (0.4, 2.5),
    "a_svc": (0.4, 2.5),
    "j_max_mps3": (0.3, 15.0),
    "A_d": (50.0, 5000.0),
    "B_d": (0.0, 500.0),
    "C_d": (0.0, 50.0),
    "tau_drv_s": (0.04, 1.5),
}


def _in_gate(val: object, lo: float, hi: float) -> bool:
    try:
        v = float(val)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return math.isfinite(v) and lo <= v <= hi


def to_replay_yaml(est: dict) -> tuple[str, list[str]]:
    """Subset of identify.yaml that is safe to pass as --params."""
    kept: list[str] = []
    skipped: list[str] = []
    lines = ["/**:", "  ros__parameters:"]
    r0 = est.get("radii", {}).get("r0_mean")
    if _in_gate(r0, *REPLAY_GATES["r0"]):
        lines.append(f"    wheel_radius_m: {float(r0):.5f}")
        lines.append("    r0_uncalibrated: false")
        kept.append("wheel_radius_m")
    else:
        skipped.append("wheel_radius_m")
    d = est.get("davis", {})
    for k in ("A_d", "B_d", "C_d"):
        if _in_gate(d.get(k), *REPLAY_GATES[k]):
            lines.append(f"    {k}: {float(d[k]):.3f}")
            kept.append(k)
        else:
            skipped.append(k)
    for key, yaml_key in (("a_trac_max", "a_trac_max"), ("a_svc", "a_svc"),
                          ("j_max_mps3", "j_max_mps3")):
        if _in_gate(est.get(key), *REPLAY_GATES[key]):
            lines.append(f"    {yaml_key}: {float(est[key]):.4f}")
            kept.append(yaml_key)
        else:
            skipped.append(yaml_key)
    td = (est.get("lag") or {}).get("T_d_s")
    if _in_gate(td, *REPLAY_GATES["tau_drv_s"]):
        lines.append(f"    tau_drv_s: {float(td):.4f}")
        kept.append("tau_drv_s")
    else:
        skipped.append("tau_drv_s")
    return "\n".join(lines) + "\n", skipped


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--mass", type=float, default=28000.0)
    ap.add_argument("--gamma", type=float, default=0.0)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args(argv)
    est = identify(args.csv, args.mass, args.gamma)
    yaml = to_yaml(est)
    if args.out is not None:
        args.out.write_text(yaml, encoding="utf-8")
        args.out.with_suffix(".json").write_text(json.dumps(est, indent=2), encoding="utf-8")
    print(yaml)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Offline analysis: physics QA, M6 gramian, M9 Stanford, M10 timing.

Does not import the UKF. replay_ukf is an optional subprocess of the caller.
"""

from __future__ import annotations

import json
import math
import re
import statistics
import sys
from pathlib import Path
from statistics import NormalDist

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, VehicleState, plant_step  # noqa: E402

DT = 0.02


def AL_S(s: float) -> float:
    return 5.0 + 0.05 * abs(s)


def _f(row: dict, key: str) -> float | None:
    raw = row.get(key)
    if raw in (None, ""):
        return None
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def physics_qa(rows: list[dict], dt: float = DT) -> dict:
    """Same-day physical checks from the plan (a–n, coast a–v, brake, radii, lag)."""
    v = [_f(r, "gt_v") for r in rows]
    a = [0.0] * len(v)
    for i in range(1, len(v) - 1):
        if v[i - 1] is None or v[i + 1] is None:
            continue
        a[i] = (v[i + 1] - v[i - 1]) / (2.0 * dt)
    trac, coast, brk = [], [], []
    ratios: dict[str, list[float]] = {f"r{i}": [] for i in range(6)}
    n_series = [_f(r, "notch") or 0.0 for r in rows]
    best_d, best = 0, -1.0
    for d in range(0, 25):
        num = den_a = den_n = 0.0
        for i in range(d, len(a)):
            if v[i] is None:
                continue
            num += a[i] * n_series[i - d]
            den_a += a[i] * a[i]
            den_n += n_series[i - d] * n_series[i - d]
        if den_a > 0 and den_n > 0:
            c = num / math.sqrt(den_a * den_n)
            if c > best:
                best, best_d = c, d
    stand_w = []
    for i, r in enumerate(rows):
        n = n_series[i]
        b = _f(r, "brake") or 0.0
        vi = v[i]
        if vi is None:
            continue
        if n > 0.3 and vi < 6.0:
            trac.append({"a": a[i], "n": n, "v": vi})
        if abs(n) < 0.05 and b < 0.05 and abs(vi) > 1.0:
            coast.append({"a": a[i], "v": vi})
        if b > 0.3:
            brk.append({"a": a[i], "b": b, "v": vi})
        if abs(vi) < 0.2 and abs(n) < 0.05:
            for k in range(6):
                w = _f(r, f"w{k}")
                if w is not None:
                    stand_w.append(abs(w))
        for k in range(6):
            w = _f(r, f"w{k}")
            if w is not None and abs(w) > 0.2 and abs(vi) > 0.5:
                ratios[f"r{k}"].append(vi / w)
    def med(xs: list[float]) -> float:
        xs = [x for x in xs if math.isfinite(x)]
        return statistics.median(xs) if xs else float("nan")
    a_n = med([x["a"] / max(x["n"], 1e-3) for x in trac])
    r_hat = {k: med(vs) for k, vs in ratios.items() if vs}
    return {
        "n_trac": len(trac),
        "n_coast": len(coast),
        "n_brake": len(brk),
        "a_per_notch_low_v": a_n,
        "coast_a_at_v": [
            {"v": x["v"], "a": x["a"]} for x in coast[:: max(len(coast) // 8, 1)][:8]
        ],
        "brake_a_over_b": med([x["a"] / max(x["b"], 1e-3) for x in brk]),
        "radii_m": r_hat,
        "standstill_omega_p95": (
            sorted(stand_w)[int(0.95 * (len(stand_w) - 1))] if len(stand_w) > 5 else float("nan")
        ),
        "T_d_s": best_d * dt,
        "lag_corr": best,
    }


def empirical_gramian(dt: float = DT, horizon_s: float = 6.0,
                     cmds: list[tuple[float, float]] | None = None) -> dict:
    """M6: empirical observability gramian on the filter twin (Krener–Ide style).

    Outputs are (s, v). Parameters: v0, m, k_trac, d0, A_d.
    Expectation to confirm: (m, k) and (v, common radius) are the weak pair.
    Default cmds mix traction / coast / brake in equal thirds.
    """
    p0 = PlantParams()
    nstep = int(horizon_s / dt)
    if cmds is None:
        cmds = ([(0.6, 0.0)] * (nstep // 3)
                + [(0.0, 0.0)] * (nstep // 3)
                + [(0.0, 0.5)] * (nstep - 2 * (nstep // 3)))
    names = ("v0", "m", "k_trac", "d0", "A_d")
    eps = (0.05, 200.0, 0.02, 0.01, 40.0)

    def rollout(delta: dict[str, float]) -> list[list[float]]:
        p = PlantParams()
        p.A_d = p0.A_d + delta.get("A_d", 0.0)
        x = VehicleState(v_mps=8.0 + delta.get("v0", 0.0),
                         m_eff_kg=p0.m0_kg + delta.get("m", 0.0),
                         k_trac=1.0 + delta.get("k_trac", 0.0))
        x.d = [1.0 + delta.get("d0", 0.0)] * 4
        ys = []
        for n, b in cmds:
            plant_step(x, n, b, dt, p)
            ys.append([x.s_m, x.v_mps])
        return ys

    cols = []
    for name, e in zip(names, eps):
        yp = rollout({name: e})
        ym = rollout({name: -e})
        col = []
        for a, b in zip(yp, ym):
            col.extend([(a[0] - b[0]) / (2.0 * e), (a[1] - b[1]) / (2.0 * e)])
        cols.append(col)
    dim = 5
    g = [[0.0] * dim for _ in range(dim)]
    for i in range(dim):
        for j in range(dim):
            g[i][j] = sum(cols[i][k] * cols[j][k] for k in range(len(cols[i])))
    # Power iteration for the five eigenvalues of a 5×5 SPD matrix.
    evals_vec: list[tuple[float, list[float]]] = []
    a = [row[:] for row in g]
    for _ in range(dim):
        v = [1.0] * dim
        lam = 0.0
        for _it in range(40):
            w = [sum(a[i][j] * v[j] for j in range(dim)) for i in range(dim)]
            nrm = math.sqrt(sum(x * x for x in w)) or 1.0
            v = [x / nrm for x in w]
            lam = sum(v[i] * sum(a[i][j] * v[j] for j in range(dim)) for i in range(dim))
        evals_vec.append((max(lam, 0.0), v[:]))
        for i in range(dim):
            for j in range(dim):
                a[i][j] -= lam * v[i] * v[j]
    evals_vec.sort(key=lambda x: x[0])
    evals = [x[0] for x in evals_vec]
    lam_max = evals[-1] if evals else 0.0
    pos = [x for x in evals if x > 1e-10 * max(lam_max, 1e-30)]
    spread = math.log10(pos[-1] / pos[0]) if len(pos) >= 2 else float("nan")
    weak_name = ""
    weak_lam = float("nan")
    for lam, vec in evals_vec:
        if lam > 1e-10 * max(lam_max, 1e-30):
            j = max(range(dim), key=lambda i: abs(vec[i]))
            weak_name = names[j]
            weak_lam = lam
            break
    return {
        "names": list(names),
        "eigenvalues_asc": evals,
        "rank_est": len(pos),
        "log10_span": spread,
        "weakest": weak_name,
        "weakest_lambda": weak_lam,
        "note": "twin plant, not the organiser bag; span of log10 λ on λ > 1e-10 λ_max",
        "n_cmd": len(cmds),
    }


def empirical_gramian_regimes(dt: float = DT, horizon_s: float = 6.0) -> dict:
    """Plan M6: gramian on traction, coast, and brake separately."""
    nstep = int(horizon_s / dt)
    return {
        "traction": empirical_gramian(dt, horizon_s, cmds=[(0.6, 0.0)] * nstep),
        "coast": empirical_gramian(dt, horizon_s, cmds=[(0.0, 0.0)] * nstep),
        "brake": empirical_gramian(dt, horizon_s, cmds=[(0.0, 0.5)] * nstep),
    }


def kappa_overbound(z: list[float], alphas: tuple[float, ...] = (0.95, 0.99, 0.997)) -> dict:
    """DeCleene-style scalar overbound: min κ s.t. N(0,(κσ)²) covers |z| quantiles."""
    mag = sorted(abs(x) for x in z if math.isfinite(x))
    if len(mag) < 20:
        return {"kappa_ob": float("nan"), "n": len(mag)}
    nd = NormalDist()
    kappas = []
    emp = {}
    for a in alphas:
        idx = min(len(mag) - 1, max(0, int(math.ceil(a * len(mag)) - 1)))
        q = mag[idx]
        gauss = nd.inv_cdf(0.5 + 0.5 * a)
        k = q / gauss if gauss > 0 else float("nan")
        emp[str(a)] = q
        kappas.append(k)
    kappa = max(kappas) if kappas else float("nan")
    return {
        "kappa_ob": max(float(kappa), 1.0) if math.isfinite(kappa) else float("nan"),
        "n": len(mag),
        "emp_abs_z": emp,
        "kappas": kappas,
    }


def stanford(est: list[dict], gt: list[dict], al_fn=AL_S) -> dict:
    """Stanford counts. AL = 5 m + 5% |s| unless the organiser sets another."""
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    n_ok = n_mi = n_hmi = n_unav = 0
    z = []
    for e in est:
        t = round(float(e.get("t", 0.0)), 3)
        g = by_t.get(t)
        if g is None:
            continue
        s_hat = float(e.get("s", float("nan")))
        s_gt = float(g.get("s", g.get("gt_s", float("nan"))))
        if not math.isfinite(s_hat) or not math.isfinite(s_gt):
            continue
        err = abs(s_hat - s_gt)
        pl = e.get("pl_s", e.get("over_m"))
        try:
            pl = float(pl) if pl is not None else float("inf")
        except (TypeError, ValueError):
            pl = float("inf")
        pss = e.get("p_ss")
        try:
            sig = math.sqrt(max(float(pss), 0.0)) if pss is not None else float("nan")
        except (TypeError, ValueError):
            sig = float("nan")
        conf = str(e.get("confidence", "")).upper()
        al = al_fn(s_gt)
        unbounded = bool(e.get("s_unbounded")) or not math.isfinite(pl)
        if conf != "OK" or unbounded:
            n_unav += 1
            continue
        n_ok += 1
        if sig > 0:
            z.append((s_hat - s_gt) / sig)
        if err > pl and err > al:
            n_hmi += 1
        elif err > pl:
            n_mi += 1
    denom = n_ok
    return {
        "n_ok": n_ok,
        "n_mi": n_mi,
        "n_hmi": n_hmi,
        "n_unavailable": n_unav,
        "hmi_rate": (n_hmi / denom) if denom else float("nan"),
        "mi_rate": (n_mi / denom) if denom else float("nan"),
        "overbound": kappa_overbound(z),
    }


_TICK_RE = re.compile(
    r"replay_ukf tick_us p50=(?P<p50>[-+eE0-9.]+) p99=(?P<p99>[-+eE0-9.]+)"
    r"(?: max=(?P<max>[-+eE0-9.]+))? n=(?P<n>\d+)"
)


def parse_tick_line(text: str) -> dict:
    m = _TICK_RE.search(text or "")
    if not m:
        return {"p50_us": float("nan"), "p99_us": float("nan"),
                "max_us": float("nan"), "n": 0}
    gd = m.groupdict()
    return {
        "p50_us": float(gd["p50"]),
        "p99_us": float(gd["p99"]),
        "max_us": float(gd["max"]) if gd.get("max") else float("nan"),
        "n": int(gd["n"]),
    }


def scale_protection(est: list[dict], kappa: float) -> list[dict]:
    """DeCleene: publish PL' = κ_ob PL. Does not change the UKF covariance."""
    k = float(kappa)
    if not math.isfinite(k) or k < 1.0:
        k = 1.0
    out = []
    for e in est:
        rec = dict(e)
        for key in ("pl_s", "over_m"):
            raw = rec.get(key)
            if raw is None:
                continue
            try:
                rec[key] = float(raw) * k
            except (TypeError, ValueError):
                pass
        rec["kappa_ob_applied"] = k
        out.append(rec)
    return out


def interval_coverage(est: list[dict], gt: list[dict]) -> dict:
    """M3 acceptance: share of active frames with v_gt in [L, U]."""
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    n_active = n_in = n_out = n_flag = 0
    for e in est:
        if not e.get("interval_active"):
            continue
        n_active += 1
        if e.get("interval_violation"):
            n_flag += 1
        t = round(float(e.get("t", 0.0)), 3)
        g = by_t.get(t)
        if g is None:
            continue
        try:
            vg = float(g.get("v", g.get("gt_v", float("nan"))))
            lo = float(e.get("v_lo", float("-inf")))
            hi = float(e.get("v_hi", float("inf")))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(vg):
            continue
        if lo <= vg <= hi:
            n_in += 1
        else:
            n_out += 1
    denom = n_in + n_out
    return {
        "n_active": n_active,
        "n_inside": n_in,
        "n_outside": n_out,
        "n_violation_flag": n_flag,
        "inside_rate": (n_in / denom) if denom else float("nan"),
    }


def window_by_t(rows: list[dict], t0: float, t1: float, key: str = "t") -> list[dict]:
    out = []
    for r in rows:
        raw = r.get(key, r.get("t_s", r.get("t")))
        try:
            t = float(raw)
        except (TypeError, ValueError):
            continue
        if t0 - 1e-9 <= t <= t1 + 1e-9:
            out.append(r)
    return out


def write_json(path: Path, obj: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, default=str) + "\n", encoding="utf-8")

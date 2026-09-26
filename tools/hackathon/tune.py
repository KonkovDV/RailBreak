"""M7 budget-limited noise search. Never changes filter defaults.

This host dry-run is a 1-D (1+1)-ES on log q_v with a small budget.
"""

from __future__ import annotations

import math
import random
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "hackathon"))

from analysis import parse_tick_line  # noqa: E402
from bench import score_method  # noqa: E402

# Frozen before any organiser test look. Plan §M7.
WEIGHTS = {"eps400": 1.0, "rmse_v": 1.0, "nees": 0.5, "unavail": 2.0}
Q_V0 = 0.0025
Q_V_LO = 2.5e-4
Q_V_HI = 2.5e-2


def criterion(score: dict, n_ok: int, n_total: int) -> float:
    """J: smaller is better. Missing pieces count as 1 (neutral-bad)."""
    def unit(key: str, scale: float) -> float:
        v = score.get(key, float("nan"))
        try:
            x = float(v)
        except (TypeError, ValueError):
            return 1.0
        if not math.isfinite(x):
            return 1.0
        return abs(x) / scale

    eps = unit("eps_400", 25.0)
    rmse_v = unit("rmse_v", 0.56)
    nees = score.get("nees_s", float("nan"))
    try:
        nees_t = abs(math.log(max(float(nees), 1e-6))) if math.isfinite(float(nees)) else 1.0
    except (TypeError, ValueError):
        nees_t = 1.0
    avail = (n_ok / n_total) if n_total else 0.0
    return (
        WEIGHTS["eps400"] * eps
        + WEIGHTS["rmse_v"] * rmse_v
        + WEIGHTS["nees"] * nees_t
        + WEIGHTS["unavail"] * (1.0 - avail)
    )


def _replay(ukf: Path, csv_path: Path, jsonl: Path, extra: list[str]) -> list[dict]:
    cmd = [str(ukf), str(csv_path), str(jsonl), *extra]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"tune replay_ukf rc={proc.returncode}: {proc.stderr}")
    parse_tick_line(proc.stderr or "")
    import json
    return [json.loads(line) for line in jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]


def _window_score(est: list[dict], gt: list[dict], t0: float, t1: float) -> tuple[dict, int, int]:
    def t_of(e: dict) -> float:
        try:
            return float(e.get("t", e.get("t_s", 0.0)))
        except (TypeError, ValueError):
            return float("nan")
    est_w = [e for e in est if t0 - 1e-9 <= t_of(e) <= t1 + 1e-9]
    gt_w = [g for g in gt if t0 - 1e-9 <= t_of(g) <= t1 + 1e-9]
    n = min(len(est_w), len(gt_w))
    if n < 5:
        return {"rmse_v": float("nan"), "eps_400": float("nan"), "nees_s": float("nan")}, 0, 0
    s = [float(e.get("s", "nan")) for e in est_w[:n]]
    v = [float(e.get("v", "nan")) for e in est_w[:n]]
    sg = [float(g.get("s", g.get("gt_s", "nan"))) for g in gt_w[:n]]
    vg = [float(g.get("v", g.get("gt_v", "nan"))) for g in gt_w[:n]]
    t = [t_of(e) for e in est_w[:n]]
    pss = [float(e.get("p_ss", "nan")) for e in est_w[:n]]
    sc = score_method("tune", s, v, sg, vg, t, pss)
    n_ok = sum(1 for e in est_w[:n] if str(e.get("confidence", "")).upper() == "OK")
    return sc, n_ok, n


def tune_qv(ukf: Path, csv_path: Path, gt: list[dict], work: Path,
            val_t0: float, val_t1: float, extra: list[str] | None = None,
            budget: int = 8, seed: int = 42) -> dict:
    """(1+1)-ES on log q_v. Does not write into vehicle YAML defaults."""
    extra = extra or []
    work.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    x = math.log(Q_V0)
    sigma = 0.4
    jsonl = work / "ukf_tune.jsonl"

    def eval_x(log_q: float) -> tuple[float, float, dict]:
        q = min(max(math.exp(log_q), Q_V_LO), Q_V_HI)
        est = _replay(ukf, csv_path, jsonl, extra + ["--set", f"q_v={q:.8g}"])
        sc, n_ok, n = _window_score(est, gt, val_t0, val_t1)
        return criterion(sc, n_ok, n), q, sc

    j0, q0, sc0 = eval_x(x)
    best = {"J": j0, "q_v": q0, "score": sc0, "weights": WEIGHTS, "budget": budget}
    trace = [{"q_v": q0, "J": j0, "accepted": True}]
    for _ in range(max(budget - 1, 0)):
        cand = x + sigma * rng.gauss(0.0, 1.0)
        jc, qc, sc = eval_x(cand)
        ok = jc <= j0
        trace.append({"q_v": qc, "J": jc, "accepted": ok})
        if ok:
            x, j0, q0, sc0 = cand, jc, qc, sc
            best = {"J": j0, "q_v": q0, "score": sc0, "weights": WEIGHTS, "budget": budget}
            sigma *= 1.2
        else:
            sigma *= 0.84
        sigma = min(max(sigma, 0.05), 1.0)
    best["trace"] = trace
    best["note"] = (
        "1-D (1+1)-ES on log q_v, val window only. Not CMA-ES 200 evals. "
        "Do not load this q_v as a default; seed-42 e2e stays on 0.0025."
    )
    return best

"""M4 command-only forecast: physics vs residual on model_mismatch.

Does not import the UKF. Residual stays default-off in the filter.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))
sys.path.insert(0, str(ROOT / "tools" / "synth"))

from plant_ref import PlantParams, VehicleState, plant_step  # noqa: E402
from fit_residual import eval_force, fit, to_yaml  # noqa: E402
from bench import open_loop_error  # noqa: E402


def _f(row: dict, key: str) -> float:
    try:
        v = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return float("nan")
    return v


def cmd_forecast(rows: list[dict], theta: list[float] | None = None,
                 dt: float = 0.02, p: PlantParams | None = None) -> tuple[list[float], list[float]]:
    """Integrate the filter twin from the handle. Residual via f_bias sign."""
    p = p or PlantParams()
    x = VehicleState()
    if rows:
        v0 = _f(rows[0], "gt_v")
        if math.isfinite(v0):
            x.v_mps = max(v0, 0.0)
    ss: list[float] = []
    vs: list[float] = []
    for row in rows:
        n = _f(row, "notch")
        b = _f(row, "brake")
        n = 0.0 if not math.isfinite(n) else n
        b = 0.0 if not math.isfinite(b) else b
        x.f_bias_n = 0.0
        if theta:
            x.f_bias_n = -eval_force(theta, x.v_mps, n, b)
        plant_step(x, n, b, dt, p)
        x.f_bias_n = 0.0
        ss.append(x.s_m)
        vs.append(x.v_mps)
    return ss, vs


def _horizon(rows: list[dict], s_hat: list[float], v_hat: list[float]) -> dict:
    t = [_f(r, "t_s") for r in rows]
    s_gt = [_f(r, "gt_s") for r in rows]
    v_gt = [_f(r, "gt_v") for r in rows]
    return open_loop_error(s_hat, v_hat, s_gt, v_gt, t)


def probe_model_mismatch(out: Path, duration_s: float = 20.0) -> dict:
    """Plan M4 acceptance: 10 s command forecast on generator≠filter plant."""
    from generate import simulate  # noqa: WPS433
    out.mkdir(parents=True, exist_ok=True)
    rows = simulate("model_mismatch", duration_s=duration_s)
    keys = list(rows[0].keys())
    csv_path = out / "model_mismatch.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    n = len(rows)
    split = max(int(0.6 * n), 20)
    train_path = out / "train.csv"
    with train_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows[:split])
    fitted = fit(train_path)
    (out / "residual.yaml").write_text(to_yaml(fitted), encoding="utf-8")
    phys_s, phys_v = cmd_forecast(rows, theta=None)
    res_s, res_v = cmd_forecast(rows, theta=fitted["theta"] if fitted["n"] >= 20 else None)
    phys = _horizon(rows, phys_s, phys_v)
    resid = _horizon(rows, res_s, res_v)
    improved = (
        math.isfinite(resid.get("e_s_10s", float("nan")))
        and math.isfinite(phys.get("e_s_10s", float("nan")))
        and resid["e_s_10s"] < phys["e_s_10s"]
    )
    return {
        "scenario": "model_mismatch",
        "duration_s": duration_s,
        "residual_n": fitted["n"],
        "sigma2": fitted["sigma2"],
        "physics": phys,
        "residual": resid,
        "e_s_10s_improved": improved,
        "note": "command-only Python twin; residual flag remains off in default UKF",
    }

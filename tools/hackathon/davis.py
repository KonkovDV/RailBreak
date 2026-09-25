"""M1 Davis probe: multi-speed coast, bootstrap CI, gated replay YAML.

A single coast (one v-band) makes A, B, C collinear. This probe concatenates
three coasts so OLS (1, |v|, v^2) is well-posed. It is not the organiser bag.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))

from plant_ref import PlantParams, VehicleState, plant_step  # noqa: E402
from identify import identify, to_replay_yaml  # noqa: E402

DT = 0.02


def write_multispeed_coast(path: Path, p: PlantParams | None = None) -> PlantParams:
    """Three coasts (20, 12, 5 m/s) plus short traction and brake clips."""
    p = p or PlantParams()
    rows: list[dict] = []
    t = 0.0

    def emit(x: VehicleState, notch: float, brake: float) -> None:
        nonlocal t
        w = x.v_mps / p.r0_m
        rows.append({
            "t_s": t,
            "notch": notch,
            "brake": brake,
            "gt_v": x.v_mps,
            "gt_s": x.s_m,
            "a_mps2": x.a_mps2,
            "w0": w, "w1": w, "w2": w, "w3": w,
        })
        t += DT

    for v0, nstep in ((20.0, 4000), (12.0, 4000), (5.0, 4000)):
        x = VehicleState(v_mps=v0)
        for _ in range(nstep):
            plant_step(x, 0.0, 0.0, DT, p)
            emit(x, 0.0, 0.0)
    x = VehicleState(v_mps=0.3)
    for _ in range(180):
        plant_step(x, 0.7, 0.0, DT, p)
        emit(x, 0.7, 0.0)
    x = VehicleState(v_mps=12.0)
    for _ in range(200):
        plant_step(x, 0.0, 0.6, DT, p)
        emit(x, 0.0, 0.6)
    keys = ["t_s", "notch", "brake", "gt_v", "gt_s", "a_mps2", "w0", "w1", "w2", "w3"]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return p


def _covers(truth: float, hat: object, lo: object, hi: object) -> bool:
    def f(x: object) -> float:
        try:
            v = float(x)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return float("nan")
        return v
    t, h, a, b = truth, f(hat), f(lo), f(hi)
    if not all(math.isfinite(x) for x in (t, h, a, b)):
        return False
    return min(a, b) <= t <= max(a, b)


def probe_davis_ident(out: Path) -> dict:
    """Recover Combino Davis on a well-posed generator. Do not load as default."""
    out.mkdir(parents=True, exist_ok=True)
    p = write_multispeed_coast(out / "coast_family.csv")
    est = identify(out / "coast_family.csv", mass=p.m0_kg, gamma=p.gamma_rot)
    yaml, skipped = to_replay_yaml(est)
    (out / "identify_replay.yaml").write_text(yaml, encoding="utf-8")
    d = est.get("davis") or {}
    covers = {
        "A_d": _covers(p.A_d, d.get("A_d"), d.get("A_d_p05"), d.get("A_d_p95")),
        "B_d": _covers(p.B_d, d.get("B_d"), d.get("B_d_p05"), d.get("B_d_p95")),
        "C_d": _covers(p.C_d, d.get("C_d"), d.get("C_d_p05"), d.get("C_d_p95")),
    }
    replay_has = [k for k in ("A_d", "B_d", "C_d") if k not in skipped]
    return {
        "truth": {"A_d": p.A_d, "B_d": p.B_d, "C_d": p.C_d, "r0": p.r0_m},
        "hat": {k: d.get(k) for k in ("A_d", "B_d", "C_d", "A_d_p05", "A_d_p95",
                                      "B_d_p05", "B_d_p95", "C_d_p05", "C_d_p95")},
        "covers_truth": all(covers.values()),
        "covers": covers,
        "replay_has_davis": replay_has,
        "replay_skipped": skipped,
        "r0_mean": (est.get("radii") or {}).get("r0_mean"),
        "n_coast": d.get("n_coast"),
        "ident_overbound_rel": d.get("ident_overbound_rel"),
        "note": "multi-speed generator; not the organiser bag. YAML is not a default. "
                "Davis CI is OLS plus a 2% relative overbound; pairs bootstrap on the "
                "noiseless twin is not a coverage statement.",
    }

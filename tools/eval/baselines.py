"""Three baselines. This file must not import a UKF."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from plant_ref import PlantParams, VehicleState, plant_step, sca_analyze


@dataclass
class BaselineSample:
    s_m: float
    v_mps: float


def naive_wheel(omega: list[float], dt_s: float, s0: float = 0.0,
                r0_m: float = 0.35) -> list[BaselineSample]:
    """Integrate mean r*omega. Lies under slip."""
    s = s0
    out: list[BaselineSample] = []
    for w in omega:
        v = w * r0_m
        s += v * dt_s
        out.append(BaselineSample(s_m=s, v_mps=v))
    return out


def plant_only(notch: list[float], brake: list[float] | None = None, dt_s: float = 0.02,
               p: PlantParams | None = None, mu_hat: float | None = None) -> list[BaselineSample]:
    """Integrate Newton+Davis from notch; ignore wheels.

    Default mu_hat is dry 0.35. Passing the scenario μ is a different baseline
    (oracle weather), not the filter's information set.
    """
    x = VehicleState()
    if mu_hat is not None:
        x.mu_hat = mu_hat
    p = p or PlantParams()
    br = brake or [0.0] * len(notch)
    out: list[BaselineSample] = []
    for u, b in zip(notch, br):
        plant_step(x, u, b, dt_s, p)
        out.append(BaselineSample(s_m=x.s_m, v_mps=x.v_mps))
    return out


def complementary(notch: list[float], omega_rows: list[list[float]], dt_s: float = 0.02,
                 alpha: float = 0.7, r0_m: float = 0.35,
                 brake: list[float] | None = None) -> list[BaselineSample]:
    """Constant-α blend of wheel speed and plant."""
    x = VehicleState()
    p = PlantParams()
    br = brake or [0.0] * len(notch)
    s = 0.0
    out: list[BaselineSample] = []
    for u, row, b in zip(notch, omega_rows, br):
        plant_step(x, u, b, dt_s, p)
        if not row:
            v_w = x.v_mps
        else:
            v_w = sum(w * r0_m for w in row) / len(row)
        v = alpha * v_w + (1.0 - alpha) * x.v_mps
        s += v * dt_s
        out.append(BaselineSample(s_m=s, v_mps=v))
    return out


def sca_wheel(omega_rows: list[list[float]], dt_s: float = 0.02,
              r0_m: float = 0.35) -> list[BaselineSample]:
    """SCA consensus path. Not one of the three MVP baselines
    (`naive_wheel`, `plant_only`, `complementary`). Tests only."""
    s = 0.0
    v = 0.0
    out: list[BaselineSample] = []
    for row in omega_rows:
        sca = sca_analyze(row, r0_m=r0_m)
        v = sca["v_consensus_mps"]
        s += v * dt_s
        out.append(BaselineSample(s_m=s, v_mps=v))
    return out

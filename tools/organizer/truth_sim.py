"""Longitudinal truth generator for the backup odometer.

This is not plant.cpp and not the notch table. The traction shape is an
exponential knee, the actuator is a second-order lag, and the brake is
quadratic. Grade and mass are piecewise constant. Front and rear radii
differ. Wheel errors share one draw and add a Laplace draw each. Slip
follows traction through its own lag. An external pulse is not a notch force.

The filter is given InputSample only.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


def laplace(rng: random.Random, scale: float) -> float:
    u = rng.random() - 0.5
    mag = -scale * math.log(max(1.0 - 2.0 * abs(u), 1.0e-12))
    return math.copysign(mag, u)


def piece(s: float, steps: tuple[tuple[float, float], ...]) -> float:
    for end, value in steps:
        if s < end:
            return value
    return steps[-1][1]


def traction_accel(notch: int, v: float, a_peak: float, n_ref: float, v_knee: float) -> float:
    """Positive notch only. Not a force hyperbola."""
    if notch <= 0:
        return 0.0
    shape = 1.0 - math.exp(-notch / n_ref)
    knee = math.exp(-max(v, 0.0) / v_knee)
    return a_peak * shape * knee


def brake_accel(notch: int, brake_lin: float, brake_quad: float) -> float:
    """Negative notch only. Quadratic, so twice the notch is not twice the accel."""
    if notch >= 0:
        return 0.0
    u = min(abs(notch), 15) / 15.0
    return -(brake_lin * u + brake_quad * u * u)


@dataclass
class TruthParams:
    a_peak: float = 0.9
    n_ref: float = 6.0
    v_knee: float = 14.0
    brake_lin: float = 0.35
    brake_quad: float = 1.1
    lag_wn: float = 5.0
    lag_zeta: float = 0.8
    grade_steps: tuple[tuple[float, float], ...] = ((180.0, 0.0), (320.0, 0.018), (1.0e9, 0.0))
    mass_steps: tuple[tuple[float, float], ...] = ((250.0, 34000.0), (1.0e9, 27000.0))
    mass_ref_kg: float = 30000.0
    r_front_m: float = 0.318
    r_rear_m: float = 0.324
    r_nominal_m: float = 0.320
    sigma_shared: float = 0.02
    sigma_private: float = 0.01
    slip_tau_s: float = 0.45
    slip_gain: float = 0.015
    outlier_every_s: float = 7.0
    outlier_kmh: float = 18.0
    dropout_on_s: float = 15.0
    dropout_off_s: float = 15.3
    disturb_on_s: float = 9.0
    disturb_off_s: float = 10.5
    disturb_mps2: float = 0.12
    seed: int = 1


@dataclass
class InputSample:
    t: float
    notch: int
    front_kmh: float
    rear_kmh: float


@dataclass
class TruthSample:
    t: float
    s: float
    v: float
    a: float
    grade: float
    mass_kg: float
    slip: float
    z1: float
    z2: float
    shared_mps: float = 0.0
    internals: dict = field(default_factory=dict)


def default_notch(t: float) -> int:
    if t < 2.0:
        return 0
    if t < 12.0:
        return 8
    if t < 14.0:
        return 0
    if t < 20.0:
        return -8
    return 0


def simulate(duration_s: float, dt: float, params: TruthParams | None = None,
             notch_of=default_notch) -> tuple[list[InputSample], list[TruthSample]]:
    p = params or TruthParams()
    rng = random.Random(p.seed)
    s = 0.0
    v = 0.0
    z1 = 0.0
    z2 = 0.0
    slip = 0.0
    inputs: list[InputSample] = []
    truth: list[TruthSample] = []
    n = int(duration_s / dt)
    g = 9.80665
    for i in range(n):
        t = i * dt
        notch = int(notch_of(t))
        mass = piece(s, p.mass_steps)
        grade = piece(s, p.grade_steps)
        raw = traction_accel(notch, v, p.a_peak, p.n_ref, p.v_knee)
        raw += brake_accel(notch, p.brake_lin, p.brake_quad)
        raw *= p.mass_ref_kg / mass
        # Second-order lag. z1 is lagged accel, z2 is its rate.
        w = p.lag_wn
        z2 += dt * (w * w * (raw - z1) - 2.0 * p.lag_zeta * w * z2)
        z1 += dt * z2
        disturb = p.disturb_mps2 if p.disturb_on_s <= t < p.disturb_off_s else 0.0
        a = z1 - g * grade + disturb
        v = max(0.0, v + a * dt)
        s = s + v * dt
        star = p.slip_gain * max(z1, 0.0)
        slip += dt * (star - slip) / p.slip_tau_s
        shared = rng.gauss(0.0, p.sigma_shared)
        front = v * (p.r_nominal_m / p.r_front_m) * (1.0 + slip)
        rear = v * (p.r_nominal_m / p.r_rear_m) * (1.0 + slip)
        front += shared + laplace(rng, p.sigma_private)
        rear += shared + laplace(rng, p.sigma_private)
        front_kmh = front * 3.6
        rear_kmh = rear * 3.6
        if i > 0 and abs(t - round(t / p.outlier_every_s) * p.outlier_every_s) < 0.5 * dt:
            front_kmh += p.outlier_kmh
        if p.dropout_on_s <= t < p.dropout_off_s:
            front_kmh = math.nan
        inputs.append(InputSample(t, notch, front_kmh, rear_kmh))
        truth.append(TruthSample(t, s, v, a, grade, mass, slip, z1, z2, shared))
    return inputs, truth

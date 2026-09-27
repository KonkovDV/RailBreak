"""Deterministic fault scenarios on the independent truth stream.

The odometer receives timestamp, notch and two bogie speeds. Truth mass,
grade, lag state and the disturbance are not arguments of the filter.
interval_coverage is the fraction of this run's valid intervals that contain
truth s. It is not a claimed percentile.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

from odometer import Odometer, Params
from integrity import IntegrityMonitor, obs_from_odometer
from truth_sim import InputSample, TruthParams, default_notch, simulate

ROVER_BASELINE_M = 12.436

SCENARIOS = (
    "no_fault",
    "front_slip",
    "rear_slip",
    "both_slip",
    "front_dropout",
    "rear_dropout",
    "common_mode_slip",
    "wheel_radius_bias",
    "actuator_delay",
    "wrong_notch_map",
    "wrong_mass",
    "wrong_grade",
    "timestamp_jitter",
    "queue_overflow",
    "missing_assets",
    "negative_notch_braking",
    "no_brake_topic",
    "rover_only",
    "no_gnss_after_start",
)

FILTER_FIELDS = ("t", "notch", "front_kmh", "rear_kmh")


def filter_view(sample: InputSample) -> dict:
    return {name: getattr(sample, name) for name in FILTER_FIELDS}


def _window(name: str) -> tuple[float | None, float | None]:
    """Inclusive fault times. None means there is no fault interval."""
    if name == "no_fault":
        return None, None
    if name in ("front_slip", "rear_slip", "both_slip", "common_mode_slip"):
        return 6.0, 10.0
    if name in ("front_dropout", "rear_dropout"):
        return 6.0, 9.0
    if name == "queue_overflow":
        return 7.0, 10.0
    if name == "timestamp_jitter":
        return 0.0, 1.0e9
    if name == "no_gnss_after_start":
        return 3.0, 1.0e9
    return 0.0, 1.0e9


def _mutate(name: str, params: TruthParams) -> TruthParams:
    if name == "actuator_delay":
        return replace(params, lag_wn=1.2)
    if name == "wrong_notch_map":
        return replace(params, a_peak=1.8, n_ref=2.5, v_knee=8.0)
    if name == "wrong_mass":
        return replace(params, mass_steps=((1.0e9, 52000.0),))
    if name == "wrong_grade":
        return replace(params, grade_steps=((1.0e9, 0.035),))
    return params


def _apply(name: str, inputs: list[InputSample]) -> list[InputSample]:
    out: list[InputSample] = []
    for i, sample in enumerate(inputs):
        t = sample.t
        front = sample.front_kmh
        rear = sample.rear_kmh
        notch = sample.notch
        if name == "front_slip" and 6.0 <= t <= 10.0 and math.isfinite(front):
            front *= 1.15
        elif name == "rear_slip" and 6.0 <= t <= 10.0 and math.isfinite(rear):
            rear *= 1.15
        elif name == "both_slip" and 6.0 <= t <= 10.0:
            if math.isfinite(front):
                front *= 1.12
            if math.isfinite(rear):
                rear *= 1.12
        elif name == "common_mode_slip" and 6.0 <= t <= 10.0:
            if math.isfinite(front):
                front *= 1.20
            if math.isfinite(rear):
                rear *= 1.20
        elif name == "front_dropout" and 6.0 <= t <= 9.0:
            front = math.nan
        elif name == "rear_dropout" and 6.0 <= t <= 9.0:
            rear = math.nan
        elif name == "wheel_radius_bias" and math.isfinite(rear):
            rear *= 1.05
        elif name == "timestamp_jitter":
            t = sample.t + (0.15 if i % 2 == 0 else -0.15)
        elif name == "queue_overflow" and 7.0 <= sample.t <= 8.5:
            continue
        out.append(InputSample(t, notch, front, rear))
    return out


def _flat_odometer(missing_assets: bool) -> Odometer:
    notches = np.arange(-15, 16)
    v_edges = np.array([0.0, 30.0])
    table = np.zeros((31, 2))
    for i, notch in enumerate(notches):
        table[i, :] = 0.07 * float(notch)
    if missing_assets:
        branch = np.array([0.0, 1.0])
        grade = np.array([0.0, 0.0])
        stops: list[dict] = []
    else:
        branch = np.array([0.0, 5000.0])
        grade = np.array([0.0, 0.0])
        stops = []
    return Odometer(branch, grade, table, notches, v_edges, stops, Params(), ring_len=0.0)


def _envelope(s: float, v: float, sigma_s: float, sigma_v: float, sigma_k: float,
              along_bound, bound_valid: bool, distance: float, unverified: float, lost: bool):
    if lost or not all(math.isfinite(x) for x in (s, v, sigma_s, sigma_v)):
        return math.nan, math.nan, math.nan, False
    cov = max(along_bound, 0.0) if bound_valid and along_bound is not None and math.isfinite(along_bound) else 0.0
    e_s = max(cov + abs(distance) * max(sigma_k, 0.0) + max(unverified, 0.0) * max(sigma_v, 0.0), max(sigma_s, 0.0))
    return s - e_s, s + e_s, e_s, True


def detection_metrics(rows: list[dict], t_on: float | None, t_off: float | None) -> dict:
    """Rates on one labelled run. Empty sets stay None rather than a filled number."""
    def active(row: dict) -> bool:
        if t_on is None or t_off is None:
            return False
        return t_on <= row["t"] <= t_off

    healthy = [r for r in rows if not active(r)]
    faulty = [r for r in rows if active(r)]
    false_alarm = None if not healthy else sum(1 for r in healthy if r["alarm"]) / len(healthy)
    missed = None if not faulty else sum(1 for r in faulty if not r["alarm"]) / len(faulty)
    detect = None
    if faulty and t_on is not None:
        prev_alarm = False
        for row in rows:
            if row["t"] < t_on:
                prev_alarm = row["alarm"]
                continue
            if row["t"] > t_off:
                break
            if row["alarm"] and not prev_alarm:
                detect = row["t"] - faulty[0]["t"]
                break
            prev_alarm = row["alarm"]
    recover = None
    alarmed = any(row["alarm"] for row in faulty)
    if alarmed and t_off is not None and any(row["t"] > t_off for row in rows):
        clear = next((row for row in rows if row["t"] > t_off and not row["alarm"]), None)
        if clear is not None:
            recover = clear["t"] - t_off
    lost = None
    if faulty:
        hit = next((r for r in rows if r["t"] >= faulty[0]["t"] and r["lost"]), None)
        if hit is not None:
            lost = hit["t"] - faulty[0]["t"]
    valid = [r for r in rows if r["interval_valid"]]
    coverage = None
    if valid:
        inside = sum(1 for r in valid if r["s_min"] <= r["s_true"] <= r["s_max"])
        coverage = inside / len(valid)
    rank = {"NONE": 0, "LOW": 1, "HIGH": 2}
    unsafe = [
        r for r in rows
        if (not r["interval_valid"]) or not (r["s_min"] <= r["s_true"] <= r["s_max"])
    ]
    if not unsafe:
        maximum = "NONE"
    else:
        maximum = max(unsafe, key=lambda r: rank.get(r["position_confidence"], 0))["position_confidence"]
    err = [(r["s_hat"] - r["s_true"]) ** 2 for r in rows if math.isfinite(r["s_hat"])]
    rmse = math.sqrt(sum(err) / len(err)) if err else None
    return {
        "rmse_m": rmse,
        "false_alarm_rate": false_alarm,
        "missed_detection_rate": missed,
        "time_to_detection": detect,
        "time_to_recovery": recover,
        "time_to_LOST": lost,
        "interval_coverage": coverage,
        "interval_n": len(valid),
        "claimed_percentile": False,
        "maximum_unsafe_confidence": maximum,
    }


def run_scenario(name: str, duration_s: float = 24.0, dt: float = 0.1) -> dict:
    if name not in SCENARIOS:
        raise KeyError(name)
    params = _mutate(name, TruthParams())
    inputs, truth = simulate(duration_s, dt, params, default_notch)
    inputs = _apply(name, inputs)
    missing = name == "missing_assets"
    od = _flat_odometer(missing)
    s0 = truth[0].s
    if name == "rover_only":
        od.init(s0 - ROVER_BASELINE_M, 1.0)
    elif missing:
        od.init(0.0, 5.0)
    else:
        od.init(s0, 1.0)
    monitor = IntegrityMonitor()
    unverified_since = None
    rows = []
    absolute = not missing
    for sample in inputs:
        view = filter_view(sample)
        t_before = od.t
        od.on_cmd(view["t"], int(view["notch"]))
        od.on_bogie(view["t"], "front", view["front_kmh"])
        od.on_bogie(view["t"], "rear", view["rear_kmh"])
        if od.t is None or (t_before is not None and od.t == t_before and view["t"] < t_before):
            continue
        if name == "no_gnss_after_start":
            absolute = view["t"] < 3.0
        obs = obs_from_odometer(
            od, view["t"], absolute_start=absolute, map_in_domain=not missing,
        )
        obs.distance_since_anchor = abs(float(od.x[0]) - float(od.s_anchor_ref))
        report = monitor.update(obs)
        lost = report.integrity_mode == "LOST"
        nominal = report.integrity_mode == "NOMINAL"
        if nominal or lost:
            unverified_since = None
        elif unverified_since is None:
            unverified_since = view["t"]
        unverified = 0.0 if unverified_since is None else max(0.0, view["t"] - unverified_since)
        sigma_s = math.sqrt(max(float(od.P[0, 0]), 0.0))
        sigma_v = math.sqrt(max(float(od.P[1, 1]), 0.0))
        sigma_k = math.sqrt(max(float(od.P[2, 2]), 0.0))
        s_min, s_max, _e, valid = _envelope(
            float(od.x[0]), float(od.x[1]), sigma_s, sigma_v, sigma_k,
            report.along_bound_m, report.bound_valid,
            obs.distance_since_anchor, unverified, lost,
        )
        truth_row = min(truth, key=lambda item: abs(item.t - sample.t))
        if abs(truth_row.t - sample.t) > 0.2:
            continue
        alarm = report.integrity_mode != "NOMINAL"
        rows.append({
            "t": view["t"],
            "alarm": alarm,
            "lost": lost,
            "s_true": truth_row.s,
            "s_hat": float(od.x[0]),
            "s_min": s_min,
            "s_max": s_max,
            "interval_valid": valid,
            "position_confidence": report.position_confidence,
        })
    t_on, t_off = _window(name)
    metrics = detection_metrics(rows, t_on, t_off)
    metrics["scenario"] = name
    metrics["n"] = len(rows)
    metrics["brake_field_sent"] = False
    return metrics

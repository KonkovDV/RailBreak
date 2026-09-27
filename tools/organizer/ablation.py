"""Baseline and ablation on one independent ride.

The default filter equations are unchanged. Anchors, grade and adaptation are
removed by the inputs or by a subclass that only this matrix uses. The model
bank is the five-mode consensus and does not replace the filter arc. The
integrity monitor does not write the filter state.

interval_coverage is the fraction of this ride's published intervals that
contain truth s. It is not a claimed percentile. Latency and memory are this
Python process, not the ROS node. This ride is not the published val table.
"""

from __future__ import annotations

import math
import time
import tracemalloc
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from integrity import IntegrityMonitor, obs_from_odometer
from odometer import IBA, IK, Odometer, Params
from truth_sim import InputSample, TruthParams, TruthSample, piece, simulate

VARIANTS = (
    "mean_odometry",
    "front_only",
    "rear_only",
    "model_only",
    "current_filter",
    "no_anchors",
    "no_grade",
    "no_adaptation",
    "model_bank",
    "integrity_monitor",
)

TITLES = {
    "mean_odometry": "Только средняя одометрия",
    "front_only": "Только front",
    "rear_only": "Только rear",
    "model_only": "Только модель",
    "current_filter": "Текущий фильтр",
    "no_anchors": "Фильтр без якорей",
    "no_grade": "Фильтр без уклона",
    "no_adaptation": "Фильтр без адаптации",
    "model_bank": "Банк моделей",
    "integrity_monitor": "Робастный monitor",
}

SLIP_ON_S = 4.0
SLIP_OFF_S = 8.0
WHEEL_SCALE = 1.02


def load_notch(path: Path | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    src = path or Path(__file__).resolve().parents[2] / "railbreak_backup_odometry" / "assets" / "notch.csv"
    lines = src.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    edges = np.array([float(name[1:]) - 0.5 for name in header[1:]])
    notches = []
    rows = []
    for line in lines[1:]:
        parts = line.split(",")
        notches.append(int(float(parts[0])))
        rows.append([float(x) for x in parts[1:]])
    return np.asarray(notches, dtype=float), edges, np.asarray(rows, dtype=float)


def ride_notch(t: float) -> int:
    if t < 2.0:
        return 0
    if t < 9.0:
        return 8
    if t < 11.0:
        return 0
    if t < 18.0:
        return -8
    return 0


def quiet_world() -> TruthParams:
    return TruthParams(
        outlier_every_s=1.0e9,
        dropout_on_s=1.0e9,
        dropout_off_s=1.0e9,
        disturb_mps2=0.0,
        grade_steps=((25.0, 0.0), (1.0e9, 0.02)),
        seed=3,
    )


def build_ride(duration_s: float = 26.0, dt: float = 0.1):
    """One ride. Front slip and a common wheel scale are applied to the inputs."""
    params = quiet_world()
    inputs, truth = simulate(duration_s, dt, params, ride_notch)
    slipped = []
    for sample in inputs:
        front = sample.front_kmh * WHEEL_SCALE
        rear = sample.rear_kmh * WHEEL_SCALE
        if SLIP_ON_S <= sample.t <= SLIP_OFF_S and math.isfinite(front):
            front *= 1.15
        slipped.append(InputSample(sample.t, sample.notch, front, rear))
    dwell = [row.s for row in truth if row.t >= 19.0 and row.v < 0.15]
    stop_s = float(sum(dwell) / len(dwell)) if len(dwell) >= 10 else None
    return slipped, truth, params, stop_s


class NoAdapt(Odometer):
    """Wheel updates do not move b_a, bogie scale or R. Anchors still run."""

    def _learn_pair(self, uf: float, ur: float) -> None:
        return

    def _update_scalar(self, h, innov, r, consider_k=False, gate_bias=False) -> None:
        ba = float(self.x[IBA])
        row = self.P[IBA, :].copy()
        col = self.P[:, IBA].copy()
        super()._update_scalar(h, innov, r, consider_k=consider_k, gate_bias=gate_bias)
        self.x[IBA] = ba
        self.P[IBA, :] = row
        self.P[:, IBA] = col
        self.P[IBA, IBA] = row[IBA]


@dataclass
class Mode:
    s: float = 0.0
    v: float = 0.0
    p_ss: float = 1.0
    p_sv: float = 0.0
    p_vv: float = 0.25
    innovation: float = 0.0
    likelihood: float = 0.0
    confidence: float = 0.2


class ModelBank:
    """Shadow bank. Same gates as model_bank.hpp. Not an IMM. Not written into the filter."""

    count = 5
    adhesion_cap = 0.40
    wheel_scale = 1.05
    mismatch_q = 20.0
    spread_gate_m = 8.0

    def __init__(self) -> None:
        self.mode = [Mode() for _ in range(self.count)]
        self.ready = False
        self.s = 0.0
        self.v = 0.0
        self.p_ss = 1.0
        self.n_used = self.count

    def init(self, s: float, v: float) -> None:
        for mode in self.mode:
            mode.s = s
            mode.v = max(0.0, v)
            mode.confidence = 1.0 / self.count
        self._fuse()
        self.ready = True

    def _fuse(self) -> None:
        leader = max(range(self.count), key=lambda j: self.mode[j].confidence)
        info_s = info_v = prec_s = prec_v = 0.0
        used = 0
        for mode in self.mode:
            if abs(mode.s - self.mode[leader].s) > self.spread_gate_m:
                continue
            ws = mode.confidence / max(mode.p_ss, 1.0e-4)
            wv = mode.confidence / max(mode.p_vv, 1.0e-6)
            prec_s += ws
            prec_v += wv
            info_s += ws * mode.s
            info_v += wv * mode.v
            used += 1
        self.n_used = used
        self.s = info_s / prec_s if prec_s > 0.0 else self.mode[leader].s
        self.v = info_v / prec_v if prec_v > 0.0 else self.mode[leader].v
        self.p_ss = 1.0 / prec_s if prec_s > 0.0 else self.mode[leader].p_ss

    def step(self, dt: float, a_now: float, a_delayed: float, u_mps: float, meas_var: float) -> None:
        if not self.ready or not dt > 0.0 or not math.isfinite(u_mps) or not meas_var > 0.0:
            return
        stay = 0.90
        switch = (1.0 - stay) / (self.count - 1)
        mixed = []
        for j in range(self.count):
            mixed.append(sum(self.mode[i].confidence * (stay if i == j else switch) for i in range(self.count)))
        like = []
        q_s = 1.0e-4
        q_v = 0.05
        for j, mode in enumerate(self.mode):
            a = a_delayed if j == 1 else a_now
            if j == 2:
                a = min(self.adhesion_cap, max(-self.adhesion_cap, a_now))
            qv = q_v * self.mismatch_q if j == 4 else q_v
            v_new = min(30.0, max(0.0, mode.v + a * dt))
            mode.s += 0.5 * (mode.v + v_new) * dt
            mode.v = v_new
            p_sv = mode.p_sv
            p_vv = mode.p_vv
            mode.p_ss += 2.0 * dt * p_sv + dt * dt * p_vv + q_s * dt
            mode.p_sv = p_sv + dt * p_vv
            mode.p_vv = p_vv + qv * dt
            z = u_mps / self.wheel_scale if j == 3 else u_mps
            nu = z - mode.v
            innovation_var = max(mode.p_vv + meas_var, 1.0e-9)
            ks = mode.p_sv / innovation_var
            kv = mode.p_vv / innovation_var
            p_sv = mode.p_sv
            p_ss = mode.p_ss
            mode.s += ks * nu
            mode.v = max(0.0, mode.v + kv * nu)
            mode.p_vv = max((1.0 - kv) * mode.p_vv, 1.0e-9)
            mode.p_sv = p_sv * meas_var / innovation_var
            mode.p_ss = max(p_ss - ks * p_sv, 1.0e-6)
            mode.innovation = nu
            quad = min(nu * nu / innovation_var, 40.0)
            likelihood = math.exp(-0.5 * quad) / math.sqrt(2.0 * math.pi * innovation_var)
            mode.likelihood = max(likelihood, 1.0e-12)
            like.append(mode.likelihood)
        mix = sum(mixed[j] * like[j] for j in range(self.count))
        if not mix > 0.0:
            mix = 1.0
        for j, mode in enumerate(self.mode):
            mode.confidence = mixed[j] * like[j] / mix
        self._fuse()


def _grade_axis(params: TruthParams, use_grade: bool) -> tuple[np.ndarray, np.ndarray]:
    branch = np.linspace(0.0, 2000.0, 401)
    if not use_grade:
        return branch, np.zeros_like(branch)
    return branch, np.array([piece(float(s), params.grade_steps) for s in branch])


def _make_filter(kind: str, notches, edges, table, params: TruthParams, stop_s: float | None) -> Odometer:
    use_grade = kind != "no_grade"
    branch, grade = _grade_axis(params, use_grade)
    stops = []
    if kind != "no_anchors" and stop_s is not None:
        stops = [{"s": stop_s, "sd": 0.5}]
    p = Params()
    if kind == "no_adaptation":
        p = replace(p, q_ba=0.0, rho_step=0.0, bias_alpha=0.0, r_alpha=0.0, r_alpha_grow=0.0)
        od = NoAdapt(branch, grade, table, notches, edges, stops, p, ring_len=0.0)
    else:
        od = Odometer(branch, grade, table, notches, edges, stops, p, ring_len=0.0)
    od.init(0.0, 1.0)
    return od


def _mps(kmh: float) -> float | None:
    if not math.isfinite(kmh):
        return None
    return kmh / 3.6


def _open_speed(kind: str, sample: InputSample) -> float | None:
    front = _mps(sample.front_kmh)
    rear = _mps(sample.rear_kmh)
    if kind == "front_only":
        return front
    if kind == "rear_only":
        return rear
    if front is None:
        return rear
    if rear is None:
        return front
    return 0.5 * (front + rear)


def _quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    return float(np.quantile(np.asarray(values, dtype=float), q))


def _finish(rows: list[dict], *, coverage, false_degraded, missed_slip, latency_s: float,
            memory_bytes: int, n_anchor: int) -> dict:
    def col(name: str) -> list[float]:
        return [row[name] for row in rows if row[name] is not None and math.isfinite(row[name])]

    sv = col("ev")
    ss = col("es")
    accel = [row["ev"] for row in rows if row["notch"] > 0 and row["ev"] is not None]
    brake = [row["ev"] for row in rows if row["notch"] < 0 and row["ev"] is not None]
    return {
        "rmse_v": math.sqrt(sum(x * x for x in sv) / len(sv)) if sv else None,
        "mae_v": sum(abs(x) for x in sv) / len(sv) if sv else None,
        "bias_accel": sum(accel) / len(accel) if accel else None,
        "bias_brake": sum(brake) / len(brake) if brake else None,
        "rmse_s": math.sqrt(sum(x * x for x in ss) / len(ss)) if ss else None,
        "final_drift": rows[-1]["es"] if rows else None,
        "p95_s": _quantile([abs(x) for x in ss], 0.95),
        "max_s": max((abs(x) for x in ss), default=None),
        "interval_coverage": coverage,
        "false_degraded": false_degraded,
        "missed_slip": missed_slip,
        "latency_s": latency_s,
        "memory_bytes": memory_bytes,
        "n": len(rows),
        "n_anchor": n_anchor,
        "claimed_percentile": False,
    }


def _score_open(kind: str, inputs, truth) -> dict:
    tracemalloc.start()
    t0 = time.perf_counter()
    s = 0.0
    prev_t = inputs[0].t
    rows = []
    for sample, true in zip(inputs, truth):
        dt = sample.t - prev_t
        prev_t = sample.t
        v = _open_speed(kind, sample)
        if v is not None and dt > 0.0:
            s += v * dt
        rows.append({
            "ev": None if v is None else v - true.v,
            "es": s - true.s,
            "notch": sample.notch,
        })
    latency = time.perf_counter() - t0
    _current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return _finish(rows, coverage=None, false_degraded=None, missed_slip=None,
                   latency_s=latency, memory_bytes=peak, n_anchor=0)


def _envelope(s, v, sigma_s, sigma_v, sigma_k, along_bound, bound_valid, distance, unverified, lost):
    if lost or not all(math.isfinite(x) for x in (s, v, sigma_s, sigma_v)):
        return math.nan, math.nan, False
    cov = max(along_bound, 0.0) if bound_valid and along_bound is not None and math.isfinite(along_bound) else 0.0
    e_s = max(cov + abs(distance) * max(sigma_k, 0.0) + max(unverified, 0.0) * max(sigma_v, 0.0), max(sigma_s, 0.0))
    return s - e_s, s + e_s, True


def _score_filter(kind: str, inputs, truth, notches, edges, table, world: TruthParams, stop_s) -> dict:
    tracemalloc.start()
    t0 = time.perf_counter()
    od = _make_filter(kind, notches, edges, table, world, stop_s)
    use_monitor = kind == "integrity_monitor"
    monitor = IntegrityMonitor() if use_monitor else None
    unverified_since = None
    rows = []
    inside = 0
    valid_n = 0
    false_n = 0
    healthy_n = 0
    missed_n = 0
    slip_n = 0
    for sample, true in zip(inputs, truth):
        od.on_cmd(sample.t, sample.notch)
        od.on_bogie(sample.t, "front", sample.front_kmh)
        od.on_bogie(sample.t, "rear", sample.rear_kmh)
        s_hat, v_hat, _k, sigma_s = od.state()
        sigma_v = math.sqrt(max(float(od.P[1, 1]), 0.0))
        sigma_k = math.sqrt(max(float(od.P[IK, IK]), 0.0))
        degraded = False
        slip_hit = False
        if use_monitor:
            obs = obs_from_odometer(od, sample.t, absolute_start=True, map_in_domain=True)
            report = monitor.update(obs)
            lost = report.integrity_mode == "LOST"
            nominal = report.integrity_mode == "NOMINAL"
            if nominal or lost:
                unverified_since = None
            elif unverified_since is None:
                unverified_since = sample.t
            unverified = 0.0 if unverified_since is None else max(0.0, sample.t - unverified_since)
            s_min, s_max, ok = _envelope(
                s_hat, v_hat, sigma_s, sigma_v, sigma_k, report.along_bound_m, report.bound_valid,
                obs.distance_since_anchor, unverified, lost,
            )
            degraded = report.status.startswith("DEGRADED")
            slip_hit = bool(od.slip_side["front"]) or report.front_wheel_confidence != "HIGH"
        else:
            ok = math.isfinite(sigma_s)
            s_min, s_max = s_hat - max(sigma_s, 0.0), s_hat + max(sigma_s, 0.0)
        if ok:
            valid_n += 1
            if s_min <= true.s <= s_max:
                inside += 1
        in_slip = SLIP_ON_S <= sample.t <= SLIP_OFF_S
        if use_monitor and not in_slip:
            healthy_n += 1
            false_n += int(degraded)
        if use_monitor and in_slip:
            slip_n += 1
            missed_n += int(not slip_hit)
        rows.append({"ev": v_hat - true.v, "es": s_hat - true.s, "notch": sample.notch})
    latency = time.perf_counter() - t0
    _current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    coverage = None if valid_n == 0 else inside / valid_n
    false_rate = None if not use_monitor or healthy_n == 0 else false_n / healthy_n
    missed = None if not use_monitor or slip_n == 0 else missed_n / slip_n
    out = _finish(rows, coverage=coverage, false_degraded=false_rate, missed_slip=missed,
                  latency_s=latency, memory_bytes=peak, n_anchor=int(od.n_anchor))
    return out


def _accel(od: Odometer, notch: int, s: float, v: float) -> float:
    saved = od.notch
    od.notch = notch
    a = od.a_model(v, s)
    od.notch = saved
    return a


def _score_model(inputs, truth, notches, edges, table, world, stop_s) -> dict:
    tracemalloc.start()
    t0 = time.perf_counter()
    od = _make_filter("model_only", notches, edges, table, world, stop_s)
    s = 0.0
    v = 0.0
    prev_t = inputs[0].t
    rows = []
    for sample, true in zip(inputs, truth):
        dt = sample.t - prev_t
        prev_t = sample.t
        if dt > 0.0:
            a = _accel(od, sample.notch, s, v)
            v = min(30.0, max(0.0, v + a * dt))
            s += v * dt
        rows.append({"ev": v - true.v, "es": s - true.s, "notch": sample.notch})
    latency = time.perf_counter() - t0
    _current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return _finish(rows, coverage=None, false_degraded=None, missed_slip=None,
                   latency_s=latency, memory_bytes=peak, n_anchor=0)


def _score_bank(inputs, truth, notches, edges, table, world, stop_s) -> dict:
    tracemalloc.start()
    t0 = time.perf_counter()
    od = _make_filter("model_bank", notches, edges, table, world, stop_s)
    bank = ModelBank()
    bank.init(0.0, 0.0)
    prev_t = inputs[0].t
    history = [(inputs[0].t, inputs[0].notch)]
    rows = []
    inside = 0
    valid_n = 0
    for sample, true in zip(inputs, truth):
        dt = sample.t - prev_t
        prev_t = sample.t
        history.append((sample.t, sample.notch))
        delayed = sample.notch
        for stamp, notch in history:
            if sample.t - stamp >= 0.30:
                delayed = notch
        front = _mps(sample.front_kmh)
        rear = _mps(sample.rear_kmh)
        if front is None:
            u = rear
        elif rear is None:
            u = front
        else:
            u = 0.5 * (front + rear)
        a_now = _accel(od, sample.notch, bank.s, bank.v)
        a_delayed = _accel(od, delayed, bank.s, bank.v)
        if u is not None:
            bank.step(dt, a_now, a_delayed, u, 0.05 ** 2)
        if bank.p_ss > 0.0 and math.isfinite(bank.s):
            half = math.sqrt(bank.p_ss)
            valid_n += 1
            if bank.s - half <= true.s <= bank.s + half:
                inside += 1
        rows.append({"ev": bank.v - true.v, "es": bank.s - true.s, "notch": sample.notch})
    latency = time.perf_counter() - t0
    _current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    coverage = None if valid_n == 0 else inside / valid_n
    return _finish(rows, coverage=coverage, false_degraded=None, missed_slip=None,
                   latency_s=latency, memory_bytes=peak, n_anchor=0)


def ablation_matrix() -> list[dict]:
    inputs, truth, world, stop_s = build_ride()
    notches, edges, table = load_notch()
    rows = []
    for kind in VARIANTS:
        if kind in ("mean_odometry", "front_only", "rear_only"):
            metrics = _score_open(kind, inputs, truth)
        elif kind == "model_only":
            metrics = _score_model(inputs, truth, notches, edges, table, world, stop_s)
        elif kind == "model_bank":
            metrics = _score_bank(inputs, truth, notches, edges, table, world, stop_s)
        else:
            metrics = _score_filter(kind, inputs, truth, notches, edges, table, world, stop_s)
        metrics["variant"] = kind
        metrics["title"] = TITLES[kind]
        metrics["stop_s"] = stop_s
        metrics["reference"] = "truth_sim"
        metrics["replaces_published_val"] = False
        rows.append(metrics)
    return rows

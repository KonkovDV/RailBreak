"""Shadow integrity monitor for the backup odometer.

The filter is not modified. Status rules match
railbreak_backup_odometry/include/railbreak_backup_odometry/integrity_monitor.hpp.

The along-track number is an empirical bound:

    q_0.99 * sigma_s + B_mode + B_time + B_map

It is not a certified protection level.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

# The first seven are bound classes. LOST refuses the coordinate after the
# blind budget. It is not a B_mode class and stays out of the coefficient fit.
STATUSES = (
    "NOMINAL",
    "DEGRADED_SINGLE_BOGIE",
    "DEGRADED_MODEL_CARRY",
    "DEGRADED_COMMON_MODE_UNOBSERVABLE",
    "DEGRADED_NO_MAP",
    "DEGRADED_RELATIVE_ONLY",
    "POSITION_UNTRUSTED",
    "LOST",
)

REASONS = (
    "FRONT_NIS_HIGH",
    "REAR_NIS_HIGH",
    "BOGIES_DISAGREE",
    "BOGIES_AGREE_MODEL_DISAGREES",
    "POSITION_OPEN",
    "STALE_FRONT",
    "STALE_REAR",
    "STAMP_REGRESSION",
    "NO_ABSOLUTE_START",
    "AMBIGUOUS_STATION_ANCHOR",
    "MAP_OUT_OF_DOMAIN",
    "BLIND_BUDGET",
    "DEEP_FAULT",
)

BOUND_NAME = "empirical along-track integrity bound"
BOUND_NAME_RU = "экспериментальная граница ошибки"
BOUND_STATEMENT = "empirical bound, not certified protection level"


def coeff_path() -> Path:
    return Path(__file__).resolve().parents[2] / "railbreak_backup_odometry" / "assets" / "integrity_bound.json"


@dataclass
class BoundCoeff:
    calibrated: bool = False
    q99: float = 0.0
    b_mode: dict = field(default_factory=dict)
    b_time_m: float = 0.0
    b_map_m: float = 0.0
    coverage: str = "0.99"
    split: str = "train"
    val_fraction_inside: float | None = None
    notes: dict = field(default_factory=dict)

    def mode_margin(self, status: str) -> float:
        return float(self.b_mode.get(status, 0.0))


def load_coeff(path: Path | None = None) -> BoundCoeff:
    src = path or coeff_path()
    if not src.is_file():
        return BoundCoeff()
    raw = json.loads(src.read_text(encoding="utf-8"))
    return BoundCoeff(
        calibrated=bool(raw.get("calibrated", False)),
        q99=float(raw.get("q_0_99", 0.0)),
        b_mode={k: float(v) for k, v in raw.get("b_mode_m", {}).items()},
        b_time_m=float(raw.get("b_time_m", 0.0)),
        b_map_m=float(raw.get("b_map_m", 0.0)),
        coverage=str(raw.get("calibrated_coverage", "0.99")),
        split=str(raw.get("calibration_split", "train")),
        val_fraction_inside=raw.get("val_fraction_inside"),
        notes=dict(raw.get("notes", {})),
    )


@dataclass
class Obs:
    t: float = 0.0
    sigma_s: float = 0.0
    slip_front: bool = False
    slip_rear: bool = False
    nis_front: float = 0.0
    nis_rear: float = 0.0
    front_age_s: float = 1.0e9
    rear_age_s: float = 1.0e9
    pair_fresh: bool = False
    bogies_agree: bool = False
    common_unobservable: bool = False
    stamp_regressed: bool = False
    absolute_start: bool = False
    map_in_domain: bool = True
    dwell: bool = False
    station_candidates: int = 0
    n_anchor: int = 0
    nis_gate: float = 16.0
    wheel_stale_s: float = 0.35
    max_gap_s: float = 30.0
    recover_s: float = 3.0
    distance_since_anchor: float = 0.0
    max_blind_time_s: float = 5.0
    max_blind_distance_m: float = 100.0
    degrade_recover_s: float = 0.0
    model_residual_mps: float = 0.0
    front_rear_residual_mps: float = 0.0
    fr_floor: float = 0.3


@dataclass
class Report:
    status: str
    reasons: list[str]
    along_bound_m: float | None
    bound_valid: bool
    use_position: bool
    certification_claim: bool
    calibrated_coverage: str
    calibration_split: str
    bound_name: str = BOUND_NAME
    bound_statement: str = BOUND_STATEMENT
    confidence_velocity: str = "ok"
    confidence_position: str = "ok"
    velocity_confidence: str = "HIGH"
    position_confidence: str = "HIGH"
    front_wheel_confidence: str = "HIGH"
    rear_wheel_confidence: str = "HIGH"
    model_confidence: str = "HIGH"
    integrity_mode: str = "NOMINAL"
    blind_warning: str = ""
    blind_time_s: float = 0.0
    fault_score: float = 0.0
    fault_duration_s: float = 0.0
    recovery_score: float = 0.0
    recovery_duration_s: float = 0.0
    common_score: float = 0.0
    fault_level: str = "nominal"
    time_to_lost: float = float("nan")
    distance_since_last_trusted_anchor: float = 0.0


class FaultScore:
    """F = exp(-dt/tau)*F + (1-exp(-dt/tau))*phi. One sample does not latch."""

    TAU_S = 0.25
    ENTER = 0.8
    ENTER_S = 0.5
    DEEP = 0.95
    DEEP_S = 2.0
    CLEAR = 0.2
    CLEAR_S = 3.0
    DT_CAP_S = 1.0

    def __init__(self) -> None:
        self.front = 0.0
        self.rear = 0.0
        self.t = None
        self.front_latched = False
        self.rear_latched = False
        self.front_above_since = None
        self.rear_above_since = None
        self.front_deep_since = None
        self.rear_deep_since = None
        self.front_low_since = None
        self.rear_low_since = None
        self.low_since = None

    def update(self, o: Obs):
        dt = 0.0
        if self.t is None:
            self.t = o.t
        elif o.t > self.t:
            dt = min(o.t - self.t, self.DT_CAP_S)
            self.t = o.t
        lam = math.exp(-dt / self.TAU_S) if dt > 0.0 else 1.0
        phi_f = self._phi(o.nis_front, o, True)
        phi_r = self._phi(o.nis_rear, o, False)
        self.front = lam * self.front + (1.0 - lam) * phi_f
        self.rear = lam * self.rear + (1.0 - lam) * phi_r
        score = max(self.front, self.rear)
        agree = o.pair_fresh and o.bogies_agree
        common = min(self.front, self.rear) if agree else 0.0
        self.front_above_since = self._arm(self.front_above_since, self.front > self.ENTER, o.t)
        self.rear_above_since = self._arm(self.rear_above_since, self.rear > self.ENTER, o.t)
        self.front_deep_since = self._arm(self.front_deep_since, self.front > self.DEEP, o.t)
        self.rear_deep_since = self._arm(self.rear_deep_since, self.rear > self.DEEP, o.t)
        self.front_low_since = self._arm(self.front_low_since, self.front < self.CLEAR, o.t)
        self.rear_low_since = self._arm(self.rear_low_since, self.rear < self.CLEAR, o.t)
        self.low_since = self._arm(self.low_since, score < self.CLEAR, o.t)
        front_above = self._elapsed(self.front_above_since, o.t)
        rear_above = self._elapsed(self.rear_above_since, o.t)
        front_deep = self._elapsed(self.front_deep_since, o.t)
        rear_deep = self._elapsed(self.rear_deep_since, o.t)
        front_low = self._elapsed(self.front_low_since, o.t)
        rear_low = self._elapsed(self.rear_low_since, o.t)
        both_above = self.front > self.ENTER and self.rear > self.ENTER
        common_above = min(front_above, rear_above) if both_above else 0.0
        if not self.front_latched and self.front > self.ENTER and front_above >= self.ENTER_S:
            self.front_latched = True
        if not self.rear_latched and self.rear > self.ENTER and rear_above >= self.ENTER_S:
            self.rear_latched = True
        if self.front_latched and self.front < self.CLEAR and front_low >= self.CLEAR_S:
            self.front_latched = False
        if self.rear_latched and self.rear < self.CLEAR and rear_low >= self.CLEAR_S:
            self.rear_latched = False
        if (self.front_latched and front_deep >= self.DEEP_S) or (
            self.rear_latched and rear_deep >= self.DEEP_S
        ):
            level = "deep"
        elif (self.front_latched or self.rear_latched) and score < self.CLEAR:
            level = "recovering"
        elif self.front_latched or self.rear_latched:
            level = "degraded"
        else:
            level = "nominal"
        return {
            "fault_score": score,
            "front_score": self.front,
            "rear_score": self.rear,
            "common_score": common,
            "recovery_score": 1.0 - score,
            "fault_duration_s": max(front_above, rear_above),
            "deep_duration_s": max(front_deep, rear_deep),
            "recovery_duration_s": self._elapsed(self.low_since, o.t),
            "common_above_s": common_above,
            "front_latched": self.front_latched,
            "rear_latched": self.rear_latched,
            "common_latched": agree and both_above and self.front_latched and self.rear_latched
            and common > self.ENTER and common_above >= self.ENTER_S,
            "level": level,
        }

    @staticmethod
    def _phi(nis: float, o: Obs, is_front: bool) -> float:
        p = nis / o.nis_gate if o.nis_gate > 0.0 and nis > 0.0 else 0.0
        if o.pair_fresh and not o.bogies_agree:
            worse = o.nis_front >= o.nis_rear if is_front else o.nis_rear > o.nis_front
            if worse:
                p = max(p, 1.0)
        if o.fr_floor > 0.0 and o.front_rear_residual_mps > o.fr_floor:
            worse = o.nis_front >= o.nis_rear if is_front else o.nis_rear > o.nis_front
            if worse:
                p = max(p, min(1.0, o.front_rear_residual_mps / o.fr_floor))
        if o.bogies_agree and o.fr_floor > 0.0 and abs(o.model_residual_mps) > 0.0:
            p = max(p, min(1.0, abs(o.model_residual_mps) / o.fr_floor))
        return min(1.0, max(0.0, p))

    @staticmethod
    def _arm(since, on: bool, t: float):
        if not on:
            return None
        if since is None or t < since:
            return t
        return since

    @staticmethod
    def _elapsed(since, t: float) -> float:
        return 0.0 if since is None else max(0.0, t - since)


class IntegrityMonitor:
    """Latch state belongs to the monitor. The filter object is never written."""

    def __init__(self, coeff: BoundCoeff | None = None) -> None:
        self.coeff = coeff if coeff is not None else load_coeff()
        self.common_since: float | None = None
        self.blind_since: float | None = None
        self.latched = False
        self.anchors_at_latch = 0
        self.last_anchor = None
        self.latched_status = "NOMINAL"
        self.pending_status = "NOMINAL"
        self.pending_since = None
        self.fault = FaultScore()

    def update(self, o: Obs) -> Report:
        front_nis = o.nis_front > o.nis_gate
        rear_nis = o.nis_rear > o.nis_gate
        front_stale = o.front_age_s > o.wheel_stale_s
        rear_stale = o.rear_age_s > o.wheel_stale_s
        fs = self.fault.update(o)
        front_bad = fs["front_latched"] or front_stale
        rear_bad = fs["rear_latched"] or rear_stale
        active = fs["common_latched"]
        common = active or o.common_unobservable
        if o.n_anchor != self.last_anchor:
            self.common_since = None
            self.blind_since = None
            self.last_anchor = o.n_anchor
        if active:
            if self.common_since is None:
                self.common_since = o.t
            if o.t - self.common_since >= o.recover_s:
                self._latch_on(o.n_anchor)
        else:
            if self.common_since is not None and o.t - self.common_since >= o.recover_s:
                self._latch_on(o.n_anchor)
            self.common_since = None
        blind_on = o.common_unobservable or fs["common_latched"]
        if blind_on:
            if self.blind_since is None:
                self.blind_since = o.t
        else:
            self.blind_since = None
        if self.latched and o.n_anchor > self.anchors_at_latch:
            self._latch_off()
        if self.latched and o.pair_fresh and not o.bogies_agree and not o.common_unobservable:
            self._latch_off()
        # `active` is the live slip. The latch is only the open position error.
        position_held = self.latched
        live_time = max(0.0, o.t - self.common_since) if active and self.common_since is not None else 0.0
        blind_time = max(0.0, o.t - self.blind_since) if blind_on and self.blind_since is not None else live_time
        over_budget = common and (
            (math.isfinite(o.max_blind_time_s) and blind_time > o.max_blind_time_s)
            or (math.isfinite(o.max_blind_distance_m) and o.distance_since_anchor > o.max_blind_distance_m)
        )

        if o.stamp_regressed or (
            front_stale and rear_stale and o.front_age_s > o.max_gap_s and o.rear_age_s > o.max_gap_s
        ) or (not o.absolute_start and not o.map_in_domain):
            status = "POSITION_UNTRUSTED"
        elif not o.map_in_domain:
            status = "DEGRADED_NO_MAP"
        elif not o.absolute_start:
            status = "DEGRADED_RELATIVE_ONLY"
        elif common and over_budget:
            status = "LOST"
        elif common:
            status = "DEGRADED_COMMON_MODE_UNOBSERVABLE"
        elif front_bad and rear_bad:
            status = "DEGRADED_MODEL_CARRY"
        elif front_bad or rear_bad:
            status = "DEGRADED_SINGLE_BOGIE"
        else:
            status = "NOMINAL"
        status = self._latch_status(status, o)

        reasons: list[str] = []
        if front_nis:
            reasons.append("FRONT_NIS_HIGH")
        if rear_nis:
            reasons.append("REAR_NIS_HIGH")
        if o.pair_fresh and not o.bogies_agree:
            reasons.append("BOGIES_DISAGREE")
        if common:
            reasons.append("BOGIES_AGREE_MODEL_DISAGREES")
        if over_budget:
            reasons.append("BLIND_BUDGET")
        if position_held and not common:
            reasons.append("POSITION_OPEN")
        if front_stale:
            reasons.append("STALE_FRONT")
        if rear_stale:
            reasons.append("STALE_REAR")
        if o.stamp_regressed:
            reasons.append("STAMP_REGRESSION")
        if not o.absolute_start:
            reasons.append("NO_ABSOLUTE_START")
        if o.dwell and o.station_candidates > 1:
            reasons.append("AMBIGUOUS_STATION_ANCHOR")
        if not o.map_in_domain:
            reasons.append("MAP_OUT_OF_DOMAIN")
        if fs["level"] == "deep":
            reasons.append("DEEP_FAULT")

        gap = min(o.front_age_s, o.rear_age_s)
        b_mode = self.coeff.mode_margin(status)
        if position_held:
            b_mode = max(b_mode, self.coeff.mode_margin("DEGRADED_COMMON_MODE_UNOBSERVABLE"))
        b_time = self.coeff.b_time_m if gap > o.wheel_stale_s else 0.0
        b_map = 0.0 if o.map_in_domain else self.coeff.b_map_m
        bound = None
        valid = False
        lost = status == "LOST"
        if not lost and self.coeff.calibrated and math.isfinite(o.sigma_s) and o.sigma_s >= 0.0:
            bound = self.coeff.q99 * o.sigma_s + b_mode + b_time + b_map
            valid = math.isfinite(bound)
        refused = lost or status == "POSITION_UNTRUSTED"
        if common and over_budget:
            time_left = 0.0
        elif common and math.isfinite(o.max_blind_time_s):
            time_left = max(0.0, o.max_blind_time_s - blind_time)
        else:
            time_left = float("nan")
        front_none = o.front_age_s > o.max_gap_s
        rear_none = o.rear_age_s > o.max_gap_s
        front_wheel = "NONE" if front_none else "LOW" if front_bad else "HIGH"
        rear_wheel = "NONE" if rear_none else "LOW" if rear_bad else "HIGH"
        front_high = front_wheel == "HIGH"
        rear_high = rear_wheel == "HIGH"
        if front_none and rear_none:
            velocity = "NONE"
        elif common or (not front_high and not rear_high):
            velocity = "LOW"
        else:
            velocity = "HIGH"
        if refused:
            position = "NONE"
        elif position_held or common or not o.absolute_start or not o.map_in_domain:
            position = "LOW"
        else:
            position = "HIGH"
        if refused or (front_none and rear_none):
            model = "NONE"
        elif common or (not front_high and not rear_high):
            model = "LOW"
        else:
            model = "HIGH"
        if velocity == "NONE" or position == "NONE":
            mode = "LOST"
        elif active and over_budget and not o.common_unobservable:
            mode = "SAFE_EXTRAPOLATION"
        elif common:
            mode = "VELOCITY_DEGRADED"
        elif not front_high and not rear_high:
            mode = "MODEL_ASSISTED"
        elif position == "LOW":
            mode = "POSITION_DEGRADED"
        elif not front_high or not rear_high:
            mode = "WHEEL_DEGRADED"
        else:
            mode = "NOMINAL"
        return Report(
            status=status,
            reasons=reasons,
            along_bound_m=bound,
            bound_valid=valid,
            use_position=not refused,
            certification_claim=False,
            calibrated_coverage=self.coeff.coverage if self.coeff.calibrated else "null",
            calibration_split=self.coeff.split,
            confidence_velocity="degraded" if (front_bad or rear_bad or common) else "ok",
            confidence_position=(
                "untrusted" if refused
                else "degraded" if (position_held or common or not o.absolute_start or not o.map_in_domain)
                else "ok"
            ),
            velocity_confidence=velocity,
            position_confidence=position,
            front_wheel_confidence=front_wheel,
            rear_wheel_confidence=rear_wheel,
            model_confidence=model,
            integrity_mode=mode,
            blind_warning=(
                "position extrapolated past the last anchor" if mode == "SAFE_EXTRAPOLATION" else ""
            ),
            blind_time_s=blind_time,
            fault_score=fs["fault_score"],
            fault_duration_s=fs["fault_duration_s"],
            recovery_score=fs["recovery_score"],
            recovery_duration_s=fs["recovery_duration_s"],
            common_score=fs["common_score"],
            fault_level="lost" if common and over_budget else fs["level"],
            time_to_lost=time_left,
            distance_since_last_trusted_anchor=o.distance_since_anchor,
        )

    def _latch_status(self, raw: str, o: Obs) -> str:
        safety = raw in ("POSITION_UNTRUSTED", "LOST", "DEGRADED_NO_MAP", "DEGRADED_RELATIVE_ONLY") or (
            self.latched_status in ("POSITION_UNTRUSTED", "LOST")
        )
        if safety or raw == self.latched_status:
            self.latched_status = raw
            self.pending_status = raw
            self.pending_since = o.t
            return raw
        if raw != self.pending_status:
            self.pending_status = raw
            self.pending_since = o.t
        slip_names = ("DEGRADED_MODEL_CARRY", "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        slip_class = (
            raw.startswith("DEGRADED_SINGLE")
            or raw in slip_names
            or self.latched_status.startswith("DEGRADED_SINGLE")
            or self.latched_status in slip_names
        )
        # Slip classes follow FaultScore: 0.5 s to enter, 3 s to clear.
        # degrade_recover_s is not that timer. It holds only a non-slip change.
        need = 0.0 if slip_class else o.degrade_recover_s
        elapsed = 0.0 if self.pending_since is None else o.t - self.pending_since
        if not need > 0.0 or elapsed + 1e-12 >= need:
            self.latched_status = raw
        return self.latched_status

    def _latch_on(self, n_anchor: int) -> None:
        if not self.latched:
            self.anchors_at_latch = n_anchor
        self.latched = True

    def _latch_off(self) -> None:
        self.latched = False


def station_candidates(od) -> int:
    """Same gate as Odometer._anchor, without applying it."""
    s = float(od.x[0])
    sig = math.sqrt(max(float(od.P[0, 0]), 0.0))
    n = 0
    for st in od.stops:
        if st["sd"] > od.p.stop_sd_max:
            continue
        r_sd = max(st["sd"], od.p.stop_sigma_floor)
        gate = od.p.stop_gate * math.sqrt(sig * sig + r_sd * r_sd)
        if abs(od.ring_delta(st["s"], s)) <= gate:
            n += 1
    return n


def obs_from_odometer(od, t: float, *, stamp_regressed: bool = False,
                      absolute_start: bool = True, map_in_domain: bool = True) -> Obs:
    """Read a twin snapshot. Does not write the twin."""
    ft, fu = od.last["front"]
    rt, ru = od.last["rear"]
    have_f = ft is not None and math.isfinite(fu)
    have_r = rt is not None and math.isfinite(ru)
    front_age = max(0.0, t - ft) if have_f else 1.0e9
    rear_age = max(0.0, t - rt) if have_r else 1.0e9
    pair = (
        have_f and have_r and abs(ft - rt) < od.p.wheel_stale_s
    )
    agree = False
    if pair:
        gate = max(od.p.fr_floor, od.p.fr_sigma_gate * math.sqrt(2.0 * od.r))
        agree = abs(od.corrected("front", fu) - od.corrected("rear", ru)) <= gate
    return Obs(
        t=t,
        sigma_s=math.sqrt(max(float(od.P[0, 0]), 0.0)),
        slip_front=bool(od.slip_side["front"]),
        slip_rear=bool(od.slip_side["rear"]),
        nis_front=float(od.slip_nis["front"]),
        nis_rear=float(od.slip_nis["rear"]),
        front_age_s=front_age,
        rear_age_s=rear_age,
        pair_fresh=pair,
        bogies_agree=agree,
        stamp_regressed=stamp_regressed,
        absolute_start=absolute_start,
        map_in_domain=map_in_domain,
        dwell=od.mode == "ZUPT",
        station_candidates=station_candidates(od),
        n_anchor=int(od.n_anchor),
        distance_since_anchor=float(od.distance_since_anchor()),
        nis_gate=od.p.nis_gate,
        wheel_stale_s=od.p.wheel_stale_s,
        max_gap_s=od.p.max_gap_s,
        recover_s=od.p.recover_s,
    )

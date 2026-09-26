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

STATUSES = (
    "NOMINAL",
    "DEGRADED_SINGLE_BOGIE",
    "DEGRADED_MODEL_CARRY",
    "DEGRADED_COMMON_MODE_UNOBSERVABLE",
    "DEGRADED_NO_MAP",
    "DEGRADED_RELATIVE_ONLY",
    "POSITION_UNTRUSTED",
)

REASONS = (
    "FRONT_NIS_HIGH",
    "REAR_NIS_HIGH",
    "BOGIES_DISAGREE",
    "BOGIES_AGREE_MODEL_DISAGREES",
    "STALE_FRONT",
    "STALE_REAR",
    "STAMP_REGRESSION",
    "NO_ABSOLUTE_START",
    "AMBIGUOUS_STATION_ANCHOR",
    "MAP_OUT_OF_DOMAIN",
)

BOUND_NAME = "empirical along-track integrity bound"
BOUND_NAME_RU = "экспериментальная граница ошибки"


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


class IntegrityMonitor:
    """Latch state belongs to the monitor. The filter object is never written."""

    def __init__(self, coeff: BoundCoeff | None = None) -> None:
        self.coeff = coeff if coeff is not None else load_coeff()
        self.common_since: float | None = None
        self.latched = False
        self.anchors_at_latch = 0

    def update(self, o: Obs) -> Report:
        front_nis = o.nis_front > o.nis_gate
        rear_nis = o.nis_rear > o.nis_gate
        front_stale = o.front_age_s > o.wheel_stale_s
        rear_stale = o.rear_age_s > o.wheel_stale_s
        front_bad = o.slip_front or front_nis or front_stale
        rear_bad = o.slip_rear or rear_nis or rear_stale
        active = o.pair_fresh and o.bogies_agree and o.slip_front and o.slip_rear
        if active:
            if self.common_since is None:
                self.common_since = o.t
            if o.t - self.common_since >= o.recover_s:
                self._latch_on(o.n_anchor)
        else:
            if self.common_since is not None and o.t - self.common_since >= o.recover_s:
                self._latch_on(o.n_anchor)
            self.common_since = None
        if self.latched and o.n_anchor > self.anchors_at_latch:
            self._latch_off()
        if self.latched and o.pair_fresh and not o.bogies_agree:
            self._latch_off()
        common = active or self.latched

        if o.stamp_regressed or (
            front_stale and rear_stale and o.front_age_s > o.max_gap_s and o.rear_age_s > o.max_gap_s
        ) or (not o.absolute_start and not o.map_in_domain):
            status = "POSITION_UNTRUSTED"
        elif not o.map_in_domain:
            status = "DEGRADED_NO_MAP"
        elif not o.absolute_start:
            status = "DEGRADED_RELATIVE_ONLY"
        elif common:
            status = "DEGRADED_COMMON_MODE_UNOBSERVABLE"
        elif front_bad and rear_bad:
            status = "DEGRADED_MODEL_CARRY"
        elif front_bad or rear_bad:
            status = "DEGRADED_SINGLE_BOGIE"
        else:
            status = "NOMINAL"

        reasons: list[str] = []
        if front_nis:
            reasons.append("FRONT_NIS_HIGH")
        if rear_nis:
            reasons.append("REAR_NIS_HIGH")
        if o.pair_fresh and not o.bogies_agree:
            reasons.append("BOGIES_DISAGREE")
        if common:
            reasons.append("BOGIES_AGREE_MODEL_DISAGREES")
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

        gap = min(o.front_age_s, o.rear_age_s)
        b_mode = self.coeff.mode_margin(status)
        b_time = self.coeff.b_time_m if gap > o.wheel_stale_s else 0.0
        b_map = 0.0 if o.map_in_domain else self.coeff.b_map_m
        bound = None
        valid = False
        if self.coeff.calibrated and math.isfinite(o.sigma_s) and o.sigma_s >= 0.0:
            bound = self.coeff.q99 * o.sigma_s + b_mode + b_time + b_map
            valid = math.isfinite(bound)
        return Report(
            status=status,
            reasons=reasons,
            along_bound_m=bound,
            bound_valid=valid,
            use_position=status != "POSITION_UNTRUSTED",
            certification_claim=False,
            calibrated_coverage=self.coeff.coverage if self.coeff.calibrated else "null",
            calibration_split=self.coeff.split,
        )

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
        nis_gate=od.p.nis_gate,
        wheel_stale_s=od.p.wheel_stale_s,
        max_gap_s=od.p.max_gap_s,
        recover_s=od.p.recover_s,
    )

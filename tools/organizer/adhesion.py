"""Adhesion anomaly proxy.

The filter is not modified. mu is not estimated: motor torque, motor current,
an IMU, an independent body speed and the contact forces are not inputs.
Coefficient of adhesion is not observable from two wheel speeds and driver
command alone.

A common-mode label means both bogies agree with each other and both disagree
with the notch model. Low adhesion can look like that. A wrong table, grade or
bias looks the same. The proxy does not separate them.
"""

from __future__ import annotations

import json
from dataclasses import dataclass


CLASSIFICATIONS = (
    "NO_FRESH_PAIR",
    "BOGIES_DISAGREE",
    "SINGLE_BOGIE_ANOMALY",
    "COMMON_MODE_SUSPECTED",
    "CONSISTENT",
)


@dataclass
class AdhesionObs:
    t: float = 0.0
    pair_fresh: bool = False
    bogies_agree: bool = False
    slip_front: bool = False
    slip_rear: bool = False
    have_consensus: bool = False
    wheel_consensus_residual: float | None = None
    have_model: bool = False
    model_consistency_residual: float | None = None
    front_nis: float = 0.0
    rear_nis: float = 0.0
    notch: int = 0
    speed: float = 0.0


@dataclass
class AdhesionReport:
    classification: str
    mu_estimate: None
    reason: str
    duration_s: float
    common_mode_duration_s: float
    wheel_consensus_residual: float | None
    model_consistency_residual: float | None
    front_nis: float
    rear_nis: float
    notch: int
    speed: float
    mu_observable: bool = False

    def as_dict(self) -> dict:
        return {
            "classification": self.classification,
            "mu_estimate": None,
            "mu_observable": False,
            "reason": self.reason,
            "duration_s": self.duration_s,
            "wheel_consensus_residual": self.wheel_consensus_residual,
            "model_consistency_residual": self.model_consistency_residual,
            "front_nis": self.front_nis,
            "rear_nis": self.rear_nis,
            "common_mode_duration_s": self.common_mode_duration_s,
            "notch": self.notch,
            "speed": self.speed,
        }

    def json(self) -> str:
        return json.dumps(self.as_dict(), ensure_ascii=False)


class AdhesionProxy:
    """Episode clock only. The filter object is never written."""

    def __init__(self) -> None:
        self.since: float | None = None

    def update(self, o: AdhesionObs) -> AdhesionReport:
        common = o.pair_fresh and o.bogies_agree and o.slip_front and o.slip_rear
        if common:
            if self.since is None:
                self.since = o.t
            duration = max(0.0, o.t - self.since)
        else:
            self.since = None
            duration = 0.0
        if not o.pair_fresh:
            classification, reason = "NO_FRESH_PAIR", "no fresh bogie pair"
        elif not o.bogies_agree:
            classification, reason = "BOGIES_DISAGREE", "bogies differ from each other"
        elif common:
            classification, reason = "COMMON_MODE_SUSPECTED", "both bogies agree but differ from model"
        elif o.slip_front or o.slip_rear:
            classification, reason = "SINGLE_BOGIE_ANOMALY", "one bogie differs from the model"
        else:
            classification, reason = "CONSISTENT", "bogies agree with each other and with the model"
        consensus = o.wheel_consensus_residual if o.have_consensus else None
        model = o.model_consistency_residual if o.have_model else None
        return AdhesionReport(
            classification=classification,
            mu_estimate=None,
            reason=reason,
            duration_s=duration,
            common_mode_duration_s=duration,
            wheel_consensus_residual=consensus,
            model_consistency_residual=model,
            front_nis=o.front_nis,
            rear_nis=o.rear_nis,
            notch=o.notch,
            speed=o.speed,
        )

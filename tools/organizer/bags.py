"""Decode the organiser contract. No filter, no GNSS in the estimator.

Topics (README of the drop):
  /vehicle/front_bogie_velocity   tram_vehicle_msgs/VelocitySensor
  /vehicle/rear_bogie_velocity    tram_vehicle_msgs/VelocitySensor
  /vehicle/driver_position_cmd    tram_vehicle_msgs/DriverControllerCommand
  /sensing/gnss/master/fix        sensor_msgs/NavSatFix          (reference / init only)
  /sensing/gnss/rover/fix         sensor_msgs/NavSatFix
  /sensing/gnss/*/vel             geometry_msgs/TwistStamped
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Iterator

_EVAL = Path(__file__).resolve().parents[1] / "eval"
if str(_EVAL) not in sys.path:
    sys.path.insert(0, str(_EVAL))

from rosbag2_io import decode_message, iter_messages  # noqa: E402

FRONT = "/vehicle/front_bogie_velocity"
REAR = "/vehicle/rear_bogie_velocity"
CMD = "/vehicle/driver_position_cmd"
MASTER_FIX = "/sensing/gnss/master/fix"
ROVER_FIX = "/sensing/gnss/rover/fix"
MASTER_VEL = "/sensing/gnss/master/vel"
ROVER_VEL = "/sensing/gnss/rover/vel"

CONTRACT = {
    FRONT: "tram_vehicle_msgs/msg/VelocitySensor",
    REAR: "tram_vehicle_msgs/msg/VelocitySensor",
    CMD: "tram_vehicle_msgs/msg/DriverControllerCommand",
    MASTER_FIX: "sensor_msgs/msg/NavSatFix",
    ROVER_FIX: "sensor_msgs/msg/NavSatFix",
    MASTER_VEL: "geometry_msgs/msg/TwistStamped",
    ROVER_VEL: "geometry_msgs/msg/TwistStamped",
}


def iter_decoded(bagdir: Path) -> Iterator[tuple[int, str, dict[str, Any]]]:
    """Yield (bag_time_ns, topic, fields). Undecodable payloads are skipped."""
    for ts, topic, typ, blob in iter_messages(bagdir):
        rec = decode_message(typ, blob)
        if rec is None:
            continue
        yield ts, topic, rec

"""Split the rate-1 latency bag by input topic.

Same definition as latency_probe: recorder time of /result/position minus
recorder time of the input with that header. This does not replace the
53/154/305 ms row by itself.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools" / "organizer"))

from latency_probe import match_receives, percentiles  # noqa: E402
from tools.eval.rosbag2_io import decode_message, iter_messages  # noqa: E402

TOPICS = (
    "/vehicle/front_bogie_velocity",
    "/vehicle/rear_bogie_velocity",
    "/vehicle/driver_position_cmd",
)


def main() -> None:
    bag = Path(sys.argv[1])
    inputs = {topic: [] for topic in TOPICS}
    outputs = []
    for ts, topic, typ, blob in iter_messages(bag):
        if topic not in inputs and topic != "/result/position":
            continue
        rec = decode_message(typ, blob)
        if not rec or "stamp_s" not in rec:
            continue
        row = (float(rec["stamp_s"]), ts * 1e-9)
        if topic == "/result/position":
            outputs.append(row)
        else:
            inputs[topic].append(row)
    for topic in TOPICS:
        lat = match_receives(inputs[topic], outputs)
        steady = percentiles(lat[10:])
        over100 = float(np.mean(np.asarray(lat) > 0.1)) if lat else float("nan")
        over250 = float(np.mean(np.asarray(lat) > 0.25)) if lat else float("nan")
        print(
            topic,
            "n",
            len(lat),
            "p50",
            round(steady["p50_s"] * 1e3, 2),
            "p95",
            round(steady["p95_s"] * 1e3, 2),
            "p99",
            round(steady["p99_s"] * 1e3, 2),
            "max",
            round(steady["max_s"] * 1e3, 2),
            "share>100ms",
            round(over100, 4),
            "share>250ms",
            round(over250, 4),
        )
    # Velocity headers are the input stamp plus 0.105 s, so the same-stamp
    # probe does not apply. Match each output to the bogie at header - 0.105.
    bogie = {}
    velocity = []
    for ts, topic, typ, blob in iter_messages(bag):
        if topic not in {
            "/vehicle/front_bogie_velocity",
            "/vehicle/rear_bogie_velocity",
            "/result/velocity",
        }:
            continue
        rec = decode_message(typ, blob)
        if not rec or "stamp_s" not in rec:
            continue
        if topic == "/result/velocity":
            velocity.append((float(rec["stamp_s"]), ts * 1e-9))
        else:
            key = round(float(rec["stamp_s"]), 6)
            bogie[key] = max(bogie.get(key, 0.0), ts * 1e-9)
    causal = []
    for stamp, received in velocity:
        trigger = bogie.get(round(stamp - 0.105, 6))
        if trigger is None or trigger <= 0.0:
            continue
        causal.append(received - trigger)
    steady = percentiles(causal[10:])
    print(
        "velocity causal",
        "n",
        len(causal),
        "p50",
        round(steady["p50_s"] * 1e3, 2),
        "p95",
        round(steady["p95_s"] * 1e3, 2),
        "p99",
        round(steady["p99_s"] * 1e3, 2),
        "max",
        round(steady["max_s"] * 1e3, 2),
    )


if __name__ == "__main__":
    main()

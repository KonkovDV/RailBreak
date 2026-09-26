"""External latency of the backup odometer.

L = t_output_receive_steady - t_input_receive_steady.

Both times are time.monotonic() in this process, when this node receives the
input topic and when it receives /result/velocity. The odometry node keeps
the input header stamp on the output. This probe reads that stamp only to
match the two receives. It does not publish /result/* and does not write a
stamp.

callback_max_us is the longest single callback inside the odometry node. It
is not L.

The first warmup_n matched pairs, in stamp order, are warm-up. The steady
percentiles are the pairs after those.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from judge import pair_one_to_one  # noqa: E402

STAMP_TOL_S = 1e-9
WARMUP_N = 10


def percentiles(samples) -> dict:
    a = np.asarray(list(samples), float)
    a = a[np.isfinite(a)]
    nan = float("nan")
    if a.size == 0:
        return {"n": 0, "p50_s": nan, "p95_s": nan, "p99_s": nan, "max_s": nan}
    return {
        "n": int(a.size),
        "p50_s": float(np.percentile(a, 50)),
        "p95_s": float(np.percentile(a, 95)),
        "p99_s": float(np.percentile(a, 99)),
        "max_s": float(np.max(a)),
    }


def split_warmup(latencies_s, warmup_n: int = WARMUP_N) -> dict:
    lat = [float(x) for x in latencies_s if np.isfinite(x)]
    n_warm = max(0, int(warmup_n))
    return {
        "warmup_n": n_warm,
        "warmup": percentiles(lat[:n_warm]),
        "steady": percentiles(lat[n_warm:]),
    }


def match_receives(inputs, outputs, tol_s: float = STAMP_TOL_S) -> list[float]:
    """Pair (stamp, t_receive) sequences. Return L in stamp order.

    Each input stamp and each output stamp is used at most once. The match
    does not cross in stamp time.
    """
    if not inputs or not outputs:
        return []
    in_stamp = np.array([p[0] for p in inputs], float)
    out_stamp = np.array([p[0] for p in outputs], float)
    idx = pair_one_to_one(out_stamp, in_stamp, tol_s)
    order = np.argsort(in_stamp, kind="mergesort")
    lat = []
    for i in order:
        j = int(idx[i])
        if j < 0:
            continue
        lat.append(float(outputs[j][1]) - float(inputs[i][1]))
    return lat


def report(inputs, outputs, warmup_n: int = WARMUP_N) -> dict:
    lat = match_receives(inputs, outputs)
    out = split_warmup(lat, warmup_n)
    out["definition"] = "t_output_receive_steady - t_input_receive_steady"
    out["stamp_modified"] = False
    out["callback_max_us_is_this"] = False
    out["n_input"] = len(inputs)
    out["n_output"] = len(outputs)
    out["n_matched"] = len(lat)
    return out


def _stamp_s(msg) -> float:
    return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)


def run(duration_s: float, warmup_n: int, input_topic: str, output_topic: str) -> dict:
    import time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    inputs: list[tuple[float, float]] = []
    outputs: list[tuple[float, float]] = []

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("backup_odometry_latency_probe")
            out_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(self._sensor_type(), input_topic, self._on_input, qos_profile_sensor_data)
            self.create_subscription(self._sensor_type(), output_topic, self._on_output, out_qos)

        @staticmethod
        def _sensor_type():
            from tram_vehicle_msgs.msg import VelocitySensor

            return VelocitySensor

        def _on_input(self, msg) -> None:
            inputs.append((_stamp_s(msg), time.monotonic()))

        def _on_output(self, msg) -> None:
            outputs.append((_stamp_s(msg), time.monotonic()))

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return report(inputs, outputs, warmup_n)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="External steady-clock latency of /result/velocity")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--warmup-n", type=int, default=WARMUP_N)
    ap.add_argument("--input-topic", default="/vehicle/front_bogie_velocity")
    ap.add_argument("--output-topic", default="/result/velocity")
    args = ap.parse_args(argv)
    try:
        out = run(args.duration, args.warmup_n, args.input_topic, args.output_topic)
    except ModuleNotFoundError as exc:
        print(f"ROS imports are not available ({exc}). The percentile function is in this file.", file=sys.stderr)
        return 2
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

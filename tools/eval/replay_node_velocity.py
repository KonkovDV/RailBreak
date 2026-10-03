#!/usr/bin/env python3
"""Dump the host replay of /result/velocity and score it like the offline checker.

This is not a ROS run and not the official checker. GNSS snaps are absent.
The event order is the bag order.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.eval.offline_checker import load_reference, load_solution_csv, score_pairs  # noqa: E402
from tools.eval.rosbag2_io import decode_message, iter_messages  # noqa: E402

TOPICS = {
    "/vehicle/front_bogie_velocity": 0,
    "/vehicle/rear_bogie_velocity": 1,
    "/vehicle/driver_position_cmd": 2,
}


def write_events(bag: Path, dest: Path) -> tuple[int, int, int]:
    counts = [0, 0, 0]
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("t,stream,value\n")
        for _ts, topic, msg_type, blob in iter_messages(bag):
            stream = TOPICS.get(topic)
            if stream is None:
                continue
            decoded = decode_message(msg_type, blob)
            if not decoded or "stamp_s" not in decoded:
                continue
            if stream == 2:
                value = float(decoded["position"])
            else:
                value = float(decoded["velocity"])
            handle.write(f"{decoded['stamp_s']:.9f},{stream},{value:.9f}\n")
            counts[stream] += 1
    return counts[0], counts[1], counts[2]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument(
        "--assets",
        type=Path,
        default=ROOT / "railbreak_backup_odometry" / "assets",
    )
    parser.add_argument(
        "--workdir",
        type=Path,
        default=ROOT / "local" / "postmortem",
    )
    args = parser.parse_args(argv)
    events = args.workdir / "velocity_events.csv"
    series = args.workdir / "node_velocity.csv"
    front, rear, cmd = write_events(args.bag, events)
    print(f"events front={front} rear={rear} cmd={cmd}")
    subprocess.run(
        [str(args.exe), str(args.assets), str(events), str(series)],
        check=True,
    )
    ref_v, _ = load_reference(args.bag)
    sol_v, _ = load_solution_csv(series)
    rmse, maximum, count, bias = score_pairs(ref_v, sol_v)
    print(f"host replay velocity: RMSE={rmse:.6f} m/s, max={maximum:.6f}, n={count}")
    print(f"host replay bias={bias:+.6f} m/s messages={len(sol_v)}")
    print("not a ROS checker run; GNSS snaps were not applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Dump the host replay of /result/velocity and score it like the offline checker.

This is not a ROS run and not the official checker. GNSS snaps are absent.
The event order is the bag order.

Two scores:
  stamp order — both streams sorted by header, as score_pairs does
  arrival order — kinematic_state at its bag time, and each published
  velocity row at the bag time of the input that emitted it. One bogie
  publishes the whole 0.04 s grid at once.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.eval.offline_checker import ApproximateTime, load_solution_csv, score_pairs  # noqa: E402
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
        handle.write("t,stream,value,bag_ns\n")
        for bag_ns, topic, msg_type, blob in iter_messages(bag):
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
            handle.write(f"{decoded['stamp_s']:.9f},{stream},{value:.9f},{int(bag_ns)}\n")
            counts[stream] += 1
    return counts[0], counts[1], counts[2]


def load_reference_arrival(bag: Path) -> list[tuple[int, float, float]]:
    rows: list[tuple[int, float, float]] = []
    for bag_ns, topic, msg_type, blob in iter_messages(bag):
        if topic != "/localization/kinematic_state":
            continue
        decoded = decode_message(msg_type, blob)
        if not decoded or "stamp_s" not in decoded:
            continue
        rows.append((int(bag_ns), float(decoded["stamp_s"]), float(decoded["v"])))
    return rows


def load_solution_arrival(path: Path) -> list[tuple[int, float, float, float]]:
    rows: list[tuple[int, float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((int(row["bag_ns"]), float(row["t"]), float(row["v"]), float(row["vf"])))
    return rows


def score_arrival(
    reference: list[tuple[int, float, float]],
    solution: list[tuple[int, float, float, float]],
    *,
    extra_ns: int = 0,
) -> tuple[float, float, int, float]:
    """Feed ApproximateTime in bag-time order. Headers stay the message stamps."""
    events: list[tuple[int, int, int, float, float]] = [
        (bag_ns, 0, i, stamp, value) for i, (bag_ns, stamp, value) in enumerate(reference)
    ]
    events.extend(
        (bag_ns + extra_ns, 1, i, stamp, value)
        for i, (bag_ns, stamp, value, _vf) in enumerate(solution)
    )
    events.sort()
    sync = ApproximateTime(100, 0.05)
    for _bag, index, _seq, stamp, value in events:
        sync.add(index, stamp, value)
    errors = [sol[1] - ref[1] for ref, sol in sync.pairs]
    if not errors:
        return math.nan, math.nan, 0, math.nan
    squared = sum(error * error for error in errors)
    return (
        math.sqrt(squared / len(errors)),
        max(abs(error) for error in errors),
        len(errors),
        sum(errors) / len(errors),
    )


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
    ref_arr = load_reference_arrival(args.bag)
    sol_arr = load_solution_arrival(series)
    ref_v = [(stamp, value) for _bag, stamp, value in ref_arr]
    sol_v, _ = load_solution_csv(series)
    rmse, maximum, count, bias = score_pairs(ref_v, sol_v)
    print(f"stamp order: RMSE={rmse:.6f} m/s, max={maximum:.6f}, n={count}, bias={bias:+.6f}")
    bursts: dict[int, int] = {}
    for bag_ns, _stamp, _value, _vf in sol_arr:
        bursts[bag_ns] = bursts.get(bag_ns, 0) + 1
    filt = [(stamp, vf) for _bag, stamp, _value, vf in sol_arr]
    frmse, fmax, fcount, fbias = score_pairs(ref_v, filt)
    print(
        f"filter speed on the same stamps: RMSE={frmse:.6f} m/s, max={fmax:.6f}, "
        f"n={fcount}, bias={fbias:+.6f}"
    )
    sizes = sorted(bursts.values())
    print(
        f"publish bursts={len(sizes)} max={sizes[-1] if sizes else 0} "
        f"median={sizes[len(sizes) // 2] if sizes else 0}"
    )
    for extra_ms in (0, 1, 10, 20, 50):
        armse, amax, acount, abias = score_arrival(ref_arr, sol_arr, extra_ns=extra_ms * 1_000_000)
        print(
            f"arrival +{extra_ms} ms: RMSE={armse:.6f} m/s, max={amax:.6f}, "
            f"n={acount}, bias={abias:+.6f}"
        )
    print("not a ROS checker run; GNSS snaps were not applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

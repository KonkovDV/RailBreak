#!/usr/bin/env python3
"""Offline copy of the official checker metric.

The live checker pairs /localization/kinematic_state with /result/velocity and
/result/position through message_filters.ApproximateTimeSynchronizer
(queue 100, slop 0.05 s, age penalty 0.1). Velocity uses the VelocitySensor
field ``velocity``. Position uses the pose. The two pairs are independent.

This module does not start ROS. It reads a rosbag2 sqlite directory with
tools.eval.rosbag2_io and either a solution CSV or the raw bogie mean.

CSV columns, header required:
  t,v,x,y,z
    one stamp for both streams
  t_vel,v,t_pos,x,y,z
    the node publishes those stamps separately
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.eval.rosbag2_io import decode_message, iter_messages  # noqa: E402


class ApproximateTime:
    """Two-topic port of humble message_filters ApproximateTime.

    Arrival order is the order of add(). The policy compares header stamps.
    """

    def __init__(self, queue_size: int = 100, slop: float = 0.05, age_penalty: float = 0.1) -> None:
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        if slop < 0.0:
            raise ValueError("slop must be non-negative")
        self.queue_size = queue_size
        self.slop = slop
        self.age_penalty = age_penalty
        self.deques: list[deque[tuple[float, float]]] = [deque(), deque()]
        self.past: list[list[tuple[float, float]]] = [[], []]
        self.num_non_empty = 0
        self.pivot: int | None = None
        self.pivot_time = 0.0
        self.candidate: tuple[tuple[float, float], tuple[float, float]] | None = None
        self.candidate_start = 0.0
        self.candidate_end = 0.0
        self.has_dropped = [False, False]
        self.pairs: list[tuple[tuple[float, float], tuple[float, float]]] = []

    def add(self, index: int, stamp: float, value: float) -> None:
        queue = self.deques[index]
        was_empty = not queue
        queue.append((stamp, value))
        if was_empty:
            self.num_non_empty += 1
            if self.num_non_empty == 2:
                self._process()
        if len(queue) + len(self.past[index]) > self.queue_size:
            self.num_non_empty = 0
            self._recover_all(0)
            self._recover_all(1)
            queue.popleft()
            self.has_dropped[index] = True
            if self.pivot is not None:
                self.candidate = None
                self.pivot = None
                self._process()

    def _recover_all(self, index: int) -> None:
        queue = self.deques[index]
        past = self.past[index]
        while past:
            queue.appendleft(past.pop())
        if queue:
            self.num_non_empty += 1

    def _recover_n(self, index: int, count: int) -> None:
        queue = self.deques[index]
        past = self.past[index]
        for _ in range(count):
            queue.appendleft(past.pop())
        if queue:
            self.num_non_empty += 1

    def _delete_front(self, index: int) -> None:
        queue = self.deques[index]
        queue.popleft()
        if not queue:
            self.num_non_empty -= 1

    def _move_front_to_past(self, index: int) -> None:
        queue = self.deques[index]
        self.past[index].append(queue.popleft())
        if not queue:
            self.num_non_empty -= 1

    def _boundary(self, end: bool) -> tuple[int, float]:
        time = self.deques[0][0][0]
        index = 0
        other = self.deques[1][0][0]
        if (other < time) ^ end:
            time = other
            index = 1
        return index, time

    def _make_candidate(self) -> None:
        self.candidate = (self.deques[0][0], self.deques[1][0])
        self.past[0].clear()
        self.past[1].clear()

    def _publish(self) -> None:
        if self.candidate is None:
            raise RuntimeError("publish without a candidate")
        self.pairs.append(self.candidate)
        self.candidate = None
        self.pivot = None
        self.num_non_empty = 0
        for index in (0, 1):
            queue = self.deques[index]
            past = self.past[index]
            while past:
                queue.appendleft(past.pop())
            queue.popleft()
            if queue:
                self.num_non_empty += 1

    def _virtual_time(self, index: int) -> float:
        """Humble getVirtualTime. The lower bound on the gap is zero here."""
        queue = self.deques[index]
        if not queue:
            if not self.past[index]:
                return self.pivot_time
            last = self.past[index][-1][0]
            if last > self.pivot_time:
                return last
            return self.pivot_time
        return queue[0][0]

    def _process(self) -> None:
        while self.num_non_empty == 2:
            end_index, end_time = self._boundary(True)
            start_index, start_time = self._boundary(False)
            for index in (0, 1):
                if index != end_index:
                    self.has_dropped[index] = False
            if self.pivot is None:
                if end_time - start_time > self.slop:
                    self._delete_front(start_index)
                    continue
                if self.has_dropped[end_index]:
                    self._delete_front(start_index)
                    continue
                self._make_candidate()
                self.candidate_start = start_time
                self.candidate_end = end_time
                self.pivot = end_index
                self.pivot_time = end_time
                self._move_front_to_past(start_index)
            else:
                span = (end_time - self.candidate_end) * (1.0 + self.age_penalty)
                if span >= (start_time - self.candidate_start):
                    self._move_front_to_past(start_index)
                else:
                    self._make_candidate()
                    self.candidate_start = start_time
                    self.candidate_end = end_time
                    self._move_front_to_past(start_index)
            if self.pivot is None:
                continue
            too_wide = (end_time - self.candidate_end) * (1.0 + self.age_penalty) >= (
                self.pivot_time - self.candidate_start
            )
            if start_index == self.pivot or too_wide:
                self._publish()
            elif self.num_non_empty < 2:
                virtual_moves = [0, 0]
                while True:
                    times = [self._virtual_time(0), self._virtual_time(1)]
                    vend_index, vend_time = (0, times[0]) if times[0] >= times[1] else (1, times[1])
                    vstart_index, vstart_time = (0, times[0]) if times[0] <= times[1] else (1, times[1])
                    if (vend_time - self.candidate_end) * (1.0 + self.age_penalty) >= (
                        self.pivot_time - self.candidate_start
                    ):
                        self._publish()
                        break
                    if (vend_time - self.candidate_end) * (1.0 + self.age_penalty) < (
                        vstart_time - self.candidate_start
                    ):
                        self.num_non_empty = 0
                        self._recover_n(0, virtual_moves[0])
                        self._recover_n(1, virtual_moves[1])
                        break
                    self._move_front_to_past(vstart_index)
                    virtual_moves[vstart_index] += 1


def _rmse(errors: list[float]) -> tuple[float, float, int]:
    if not errors:
        return math.nan, math.nan, 0
    squared = 0.0
    maximum = 0.0
    for error in errors:
        absolute = abs(error)
        squared += absolute * absolute
        if absolute > maximum:
            maximum = absolute
    return math.sqrt(squared / len(errors)), maximum, len(errors)


def score_pairs(
    reference: list[tuple[float, float]],
    solution: list[tuple[float, float]],
    *,
    slop: float = 0.05,
    queue_size: int = 100,
) -> tuple[float, float, int]:
    """RMSE and max |error| of solution minus reference, official sync."""
    events: list[tuple[float, int, float]] = [(stamp, 0, value) for stamp, value in reference]
    events.extend((stamp, 1, value) for stamp, value in solution)
    events.sort(key=lambda item: (item[0], item[1]))
    sync = ApproximateTime(queue_size, slop)
    for stamp, index, value in events:
        sync.add(index, stamp, value)
    errors = [sol[1] - ref[1] for ref, sol in sync.pairs]
    rmse, maximum, count = _rmse(errors)
    bias = sum(errors) / count if count else math.nan
    return rmse, maximum, count, bias


def score_position(
    reference: list[tuple[float, tuple[float, float, float]]],
    solution: list[tuple[float, tuple[float, float, float]]],
    *,
    slop: float = 0.05,
    queue_size: int = 100,
) -> dict[str, tuple[float, float, int]]:
    """Independent x, y, z and 3D RMSE. Values are packed into one float stream per axis."""
    names = ("x", "y", "z")
    out: dict[str, tuple[float, float, int]] = {}
    distance: list[float] = []
    paired: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
    # Pair on stamps only, then read the vectors. One sync, payload is the index.
    ref_s = [(stamp, float(i)) for i, (stamp, _) in enumerate(reference)]
    sol_s = [(stamp, float(i)) for i, (stamp, _) in enumerate(solution)]
    events: list[tuple[float, int, float]] = [(stamp, 0, value) for stamp, value in ref_s]
    events.extend((stamp, 1, value) for stamp, value in sol_s)
    events.sort(key=lambda item: (item[0], item[1]))
    sync = ApproximateTime(queue_size, slop)
    for stamp, index, value in events:
        sync.add(index, stamp, value)
    for ref_msg, sol_msg in sync.pairs:
        ref_xyz = reference[int(ref_msg[1])][1]
        sol_xyz = solution[int(sol_msg[1])][1]
        paired.append((ref_xyz, sol_xyz))
    axes = [[], [], []]
    for ref_xyz, sol_xyz in paired:
        square = 0.0
        for axis in range(3):
            error = sol_xyz[axis] - ref_xyz[axis]
            axes[axis].append(error)
            square += error * error
        distance.append(math.sqrt(square))
    for name, errors in zip(names, axes):
        out[name] = _rmse(errors)
    out["distance"] = _rmse(distance)
    return out


def load_reference(bagdir: Path) -> tuple[list[tuple[float, float]], list[tuple[float, tuple[float, float, float]]]]:
    velocity: list[tuple[float, float]] = []
    position: list[tuple[float, tuple[float, float, float]]] = []
    for _ts, topic, msg_type, blob in iter_messages(bagdir):
        if topic != "/localization/kinematic_state":
            continue
        decoded = decode_message(msg_type, blob)
        if not decoded or "stamp_s" not in decoded:
            continue
        stamp = float(decoded["stamp_s"])
        velocity.append((stamp, float(decoded["v"])))
        position.append((stamp, (float(decoded["s"]), float(decoded["y"]), float(decoded["z"]))))
    return velocity, position


def load_bogies(bagdir: Path) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    front: list[tuple[float, float]] = []
    rear: list[tuple[float, float]] = []
    for _ts, topic, msg_type, blob in iter_messages(bagdir):
        if topic not in {"/vehicle/front_bogie_velocity", "/vehicle/rear_bogie_velocity"}:
            continue
        decoded = decode_message(msg_type, blob)
        if not decoded or "stamp_s" not in decoded:
            continue
        sample = (float(decoded["stamp_s"]), float(decoded["velocity"]) / 3.6)
        if topic.endswith("front_bogie_velocity"):
            front.append(sample)
        else:
            rear.append(sample)
    return front, rear


def wheel_mean_solution(
    front: list[tuple[float, float]],
    rear: list[tuple[float, float]],
    offset_s: float,
) -> list[tuple[float, float]]:
    """Mean of bogies that share a header stamp, published at stamp + offset."""
    pairs = matched_bogie_means(front, rear)
    return [(stamp + offset_s, value) for stamp, value in pairs]


def matched_bogie_means(
    front: list[tuple[float, float]],
    rear: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Mean at the shared header stamp. Stamps are rounded to 1 µs, as the node mates them."""
    rear_at = {round(stamp, 6): value for stamp, value in rear}
    out: list[tuple[float, float]] = []
    for stamp, value in front:
        other = rear_at.get(round(stamp, 6))
        if other is None:
            continue
        out.append((stamp, 0.5 * (value + other)))
    out.sort(key=lambda item: item[0])
    return out


def velocity_resample(
    t0: float,
    v0: float,
    t1: float,
    v1: float,
    step: float,
    cap: int = 32,
) -> list[tuple[float, float]]:
    """Python twin of railbreak::velocity_resample. Samples on (t0, t1]."""
    if cap <= 0 or not math.isfinite(t1) or not math.isfinite(v1):
        return []
    grid = (
        math.isfinite(t0)
        and math.isfinite(v0)
        and math.isfinite(step)
        and step > 0.0
        and t1 > t0
    )
    if not grid:
        return [(t1, v1)]
    out: list[tuple[float, float]] = []
    span = t1 - t0
    t = t0 + step
    while len(out) < cap and t <= t1 + 1e-9:
        alpha = min(1.0, max(0.0, (t - t0) / span))
        ts = t1 if t > t1 else t
        vs = v0 + alpha * (v1 - v0)
        if out and not (ts > out[-1][0]):
            break
        out.append((ts, vs))
        if out[-1][0] >= t1 - 1e-12:
            break
        t += step
    if (not out or out[-1][0] < t1 - 1e-6) and len(out) < cap:
        out.append((t1, v1))
    return out


def resampled_wheel_mean(
    front: list[tuple[float, float]],
    rear: list[tuple[float, float]],
    offset_s: float,
    step_s: float = 0.04,
    gap_max_s: float = 0.35,
) -> list[tuple[float, float]]:
    """Node publish rule while both bogies share a stamp: linear grid, then stamp offset.

    This is not the filter. A pair the node would replace with filter speed stays the raw mean.
    publish_one_velocity drops a stamp that is not strictly newer than the previous header.
    """
    pairs = matched_bogie_means(front, rear)
    published: list[tuple[float, float]] = []
    prev_t = math.nan
    prev_v = math.nan
    last_header = math.nan
    for stamp, value in pairs:
        gap = stamp - prev_t if math.isfinite(prev_t) else math.nan
        gap_ok = math.isfinite(gap) and 0.0 < gap <= gap_max_s
        samples = velocity_resample(
            prev_t if gap_ok else math.nan,
            prev_v if gap_ok else math.nan,
            stamp,
            value,
            step_s,
        )
        for sample_t, sample_v in samples:
            header = sample_t + offset_s
            if math.isfinite(last_header) and not (header > last_header):
                continue
            last_header = header
            published.append((header, sample_v))
        prev_t = stamp
        prev_v = value
    return published


def raw_disagreements(
    front: list[tuple[float, float]],
    rear: list[tuple[float, float]],
    floor_m_s: float = 0.3,
) -> tuple[int, int]:
    """Pairs whose raw |front − rear| exceeds the floor. The node gate can be wider."""
    rear_at = {round(stamp, 6): value for stamp, value in rear}
    matched = 0
    wide = 0
    for stamp, value in front:
        other = rear_at.get(round(stamp, 6))
        if other is None:
            continue
        matched += 1
        if abs(value - other) > floor_m_s:
            wide += 1
    return matched, wide


def load_solution_csv(path: Path) -> tuple[list[tuple[float, float]], list[tuple[float, tuple[float, float, float]]]]:
    velocity: list[tuple[float, float]] = []
    position: list[tuple[float, tuple[float, float, float]]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if "t" in fields and "v" in fields:
            for row in reader:
                stamp = float(row["t"])
                velocity.append((stamp, float(row["v"])))
                if "x" in fields:
                    position.append((stamp, (float(row["x"]), float(row["y"]), float(row["z"]))))
        elif "t_vel" in fields and "t_pos" in fields:
            for row in reader:
                velocity.append((float(row["t_vel"]), float(row["v"])))
                position.append(
                    (float(row["t_pos"]), (float(row["x"]), float(row["y"]), float(row["z"])))
                )
        else:
            raise ValueError("CSV needs t,v or t_vel,v,t_pos")
    return velocity, position


def _format(name: str, metric: tuple[float, float, int], unit: str) -> str:
    rmse, maximum, count = metric
    return f"{name}: RMSE={rmse:.6f} {unit}, max={maximum:.6f}, n={count}"


def self_test() -> None:
    stamps = [i * 0.1 for i in range(20)]
    reference = [(stamp, 1.0) for stamp in stamps]
    same = score_pairs(reference, list(reference))
    if same[2] != 20 or same[0] != 0.0:
        raise AssertionError(f"identical streams: {same}")
    shifted = [(stamp + 0.04, 1.25) for stamp in stamps]
    # The last candidate stays unpublished until a later message on the pivot
    # topic proves it. That is the humble synchronizer, not a dropped pair.
    near = score_pairs(reference, shifted)
    if near[2] != 19 or abs(near[0] - 0.25) > 1e-9:
        raise AssertionError(f"0.04 s shift: {near}")
    # process() runs when the empty topic receives a message. A later reference
    # flushes the held pair; another solution on the already-occupied topic does not.
    flushed = score_pairs(reference + [(stamps[-1] + 1.0, 1.0)], shifted)
    if flushed[2] != 20 or abs(flushed[0] - 0.25) > 1e-9:
        raise AssertionError(f"flushed 0.04 s shift: {flushed}")
    # 0.1 s samples alias a 0.2 s shift onto a later sample. A 1 s grid does not.
    coarse = [(i * 1.0, 1.0) for i in range(10)]
    far = [(i * 1.0 + 0.2, 1.0) for i in range(10)]
    missed = score_pairs(coarse, far)
    if missed[2] != 0:
        raise AssertionError(f"0.2 s shift paired {missed[2]}")
    dense = [(i * 0.02, 0.0) for i in range(100)]
    sparse = [(i * 0.1, 0.5) for i in range(20)]
    mixed = score_pairs(dense, sparse)
    if mixed[2] < 18 or abs(mixed[0] - 0.5) > 1e-9:
        raise AssertionError(f"50 Hz vs 10 Hz: {mixed}")
    grid = velocity_resample(0.0, 0.0, 0.1, 1.0, 0.04)
    if [round(v, 6) for _, v in grid] != [0.4, 0.8, 1.0]:
        raise AssertionError(f"resample grid: {grid}")
    published = resampled_wheel_mean([(0.0, 0.0), (0.1, 2.0)], [(0.0, 0.0), (0.1, 2.0)], 0.105, 0.04)
    headers = [round(stamp, 6) for stamp, _ in published]
    if headers != [0.105, 0.145, 0.185, 0.205]:
        raise AssertionError(f"resampled headers: {headers}")
    print("offline_checker: sync self-test passed")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--solution", type=Path, help="CSV of the node output")
    parser.add_argument("--wheels", action="store_true", help="score the raw bogie mean")
    parser.add_argument(
        "--resample",
        type=float,
        default=0.0,
        help="also score the node grid of the raw mean; 0.04 matches output_velocity_resample_s",
    )
    parser.add_argument("--offset", type=float, default=0.105)
    parser.add_argument("--slop", type=float, default=0.05)
    args = parser.parse_args(argv)
    if args.self_test:
        self_test()
        return 0
    if args.bag is None:
        parser.error("--bag is required unless --self-test")
    ref_v, ref_p = load_reference(args.bag)
    if args.wheels:
        front, rear = load_bogies(args.bag)
        sol_v = wheel_mean_solution(front, rear, args.offset)
        sol_p: list[tuple[float, tuple[float, float, float]]] = []
        if args.resample > 0.0:
            matched, wide = raw_disagreements(front, rear)
            resampled = resampled_wheel_mean(front, rear, args.offset, args.resample)
            print(f"matched bogie stamps={matched}, raw |front-rear|>0.3 m/s: {wide}")
            print(f"resampled messages={len(resampled)}")
    elif args.solution is not None:
        sol_v, sol_p = load_solution_csv(args.solution)
    else:
        parser.error("pass --wheels or --solution")
        return 2
    rmse, maximum, count, bias = score_pairs(ref_v, sol_v, slop=args.slop)
    print(_format("velocity", (rmse, maximum, count), "m/s"))
    print(f"velocity bias={bias:+.6f} m/s")
    if args.wheels and args.resample > 0.0:
        r_rmse, r_max, r_count, r_bias = score_pairs(ref_v, resampled, slop=args.slop)
        print(_format("velocity resampled raw mean", (r_rmse, r_max, r_count), "m/s"))
        print(f"velocity resampled bias={r_bias:+.6f} m/s")
    if sol_p:
        pos = score_position(ref_p, sol_p, slop=args.slop)
        for name in ("x", "y", "z", "distance"):
            print(_format(name, pos[name], "m"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

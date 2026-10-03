#!/usr/bin/env python3
"""Score a recorded /result/position against the checker reference.

Nearest-stamp pairing is the harness rule in tools/v2/emu.py. It is not the
official checker. The ApproximateTime numbers are the offline port, also not
a new official checker run.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.eval.offline_checker import score_position  # noqa: E402
from tools.eval.rosbag2_io import decode_message, iter_messages  # noqa: E402


def load_pose(bag: Path, topic: str) -> list[tuple[float, tuple[float, float, float]]]:
    rows: list[tuple[float, tuple[float, float, float]]] = []
    for _ts, name, msg_type, blob in iter_messages(bag):
        if name != topic:
            continue
        decoded = decode_message(msg_type, blob)
        if not decoded or "stamp_s" not in decoded:
            continue
        rows.append(
            (
                float(decoded["stamp_s"]),
                (float(decoded["s"]), float(decoded["y"]), float(decoded["z"])),
            )
        )
    rows.sort(key=lambda item: item[0])
    return rows


def nearest(
    reference: list[tuple[float, tuple[float, float, float]]],
    solution: list[tuple[float, tuple[float, float, float]]],
    slop: float = 0.05,
) -> dict[str, float]:
    stamps = [stamp for stamp, _ in solution]
    xyz = [point for _stamp, point in solution]
    errors = [[], [], []]
    distance: list[float] = []
    j = 0
    for stamp, ref in reference:
        while j + 1 < len(stamps) and stamps[j + 1] <= stamp:
            j += 1
        best = None
        for k in (j - 1, j, j + 1):
            if not 0 <= k < len(stamps):
                continue
            gap = abs(stamps[k] - stamp)
            if gap <= slop and (best is None or gap < best[0]):
                best = (gap, xyz[k])
        if best is None:
            continue
        square = 0.0
        for axis in range(3):
            error = best[1][axis] - ref[axis]
            errors[axis].append(error)
            square += error * error
        distance.append(math.sqrt(square))
    def rmse(values: list[float]) -> tuple[float, int]:
        if not values:
            return math.nan, 0
        return math.sqrt(sum(v * v for v in values) / len(values)), len(values)
    out = {}
    for name, values in zip(("x", "y", "z"), errors):
        value, count = rmse(values)
        out[name] = value
        out["n"] = count
    dist, count = rmse(distance)
    out["distance"] = dist
    out["n"] = count
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solution-bag", type=Path, required=True)
    parser.add_argument("--reference-bag", type=Path, required=True)
    args = parser.parse_args(argv)
    solution = load_pose(args.solution_bag, "/result/position")
    reference = load_pose(args.reference_bag, "/localization/kinematic_state")
    print(f"solution={len(solution)} reference={len(reference)}")
    synced = score_position(reference, solution)
    for name in ("x", "y", "z", "distance"):
        rmse, maximum, count = synced[name]
        print(f"port {name}: RMSE={rmse:.6f} m, max={maximum:.6f}, n={count}")
    near = nearest(reference, solution)
    print(
        "nearest x={x:.6f} y={y:.6f} z={z:.6f} distance={distance:.6f} n={n}".format(**near)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Score a recorded /result/velocity bag with the offline synchronizer.

The bag is the node's published stream. This is not a second official checker
run: the live RMSE stays the number printed by metrics.py.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.eval.offline_checker import score_pairs  # noqa: E402
from tools.eval.replay_node_velocity import score_arrival  # noqa: E402
from tools.eval.rosbag2_io import decode_message, iter_messages  # noqa: E402


def load_topic(bag: Path, topic: str, value_key: str) -> list[tuple[int, float, float]]:
    rows: list[tuple[int, float, float]] = []
    for bag_ns, name, msg_type, blob in iter_messages(bag):
        if name != topic:
            continue
        decoded = decode_message(msg_type, blob)
        if not decoded or "stamp_s" not in decoded or decoded.get(value_key) is None:
            continue
        rows.append((int(bag_ns), float(decoded["stamp_s"]), float(decoded[value_key])))
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--reference-bag", type=Path)
    args = parser.parse_args(argv)
    recorded_ref = load_topic(args.bag, "/localization/kinematic_state", "v")
    solution = load_topic(args.bag, "/result/velocity", "velocity")
    print(f"recorded reference={len(recorded_ref)} velocity={len(solution)}")
    ref_v = [(stamp, value) for _bag, stamp, value in recorded_ref]
    sol_v = [(stamp, value) for _bag, stamp, value in solution]
    rmse, maximum, count, bias = score_pairs(ref_v, sol_v)
    print(f"stamp order, recorded reference: RMSE={rmse:.6f} m/s, max={maximum:.6f}, n={count}, bias={bias:+.6f}")
    sol_arr = [(bag_ns, stamp, value, value) for bag_ns, stamp, value in solution]
    armse, amax, acount, abias = score_arrival(recorded_ref, sol_arr)
    print(
        f"arrival order, recorded reference: RMSE={armse:.6f} m/s, max={amax:.6f}, "
        f"n={acount}, bias={abias:+.6f}"
    )
    if args.reference_bag:
        original = load_topic(args.reference_bag, "/localization/kinematic_state", "v")
        original_v = [(stamp, value) for _bag, stamp, value in original]
        ormse, omax, ocount, obias = score_pairs(original_v, sol_v)
        print(
            f"stamp order, original reference: RMSE={ormse:.6f} m/s, max={omax:.6f}, "
            f"n={ocount}, bias={obias:+.6f}"
        )
        oarmse, oamax, oacount, oabias = score_arrival(original, sol_arr)
        print(
            f"arrival order, original reference: RMSE={oarmse:.6f} m/s, max={oamax:.6f}, "
            f"n={oacount}, bias={oabias:+.6f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

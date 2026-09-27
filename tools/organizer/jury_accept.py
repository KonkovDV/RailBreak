"""Fail-closed checks for scripts/jury.sh acceptance.

The interactive record scenario does not call this. A clean observation
returns no reasons. Any reason is a nonzero exit.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "eval"))
from rosbag2_io import decode_message, iter_messages  # noqa: E402
from score_ros import output_rate_report  # noqa: E402

# Published floor is 10 Hz. The gap allowance is the published 250 ms peak.
MIN_RATE_HZ = 10.0
MAX_GAP_S = 0.25


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def acceptance_failures(obs: dict) -> list[str]:
    reasons: list[str] = []
    if not obs.get("node_alive", False):
        reasons.append("node died")
    if not obs.get("have_velocity", False):
        reasons.append("no /result/velocity")
    if obs.get("require_position", True) and not obs.get("have_position", False):
        reasons.append("no /result/position")
    stamps = np.asarray(obs.get("velocity_stamps") or [], float)
    report = output_rate_report(stamps)
    if int(report["regressed_stamp_count"]) > 0:
        reasons.append(f"stamp regressed ({report['regressed_stamp_count']})")
    rate = report["rate_unique_stamp_hz"]
    if not _finite(rate) or float(rate) < MIN_RATE_HZ:
        reasons.append(f"frequency {rate} Hz is below {MIN_RATE_HZ:g} Hz")
    gap = report["max_gap"]
    if not _finite(gap) or float(gap) > MAX_GAP_S:
        reasons.append(f"max gap {gap} s is above {MAX_GAP_S:g} s")
    if obs.get("require_position", True) and obs.get("have_position", False):
        reasons.extend(_position_failures(obs))
    status = int(obs.get("scorer_status", 1))
    if status != 0:
        reasons.append(f"scorer failed ({status})")
    if obs.get("frame_id") != "map":
        reasons.append(f"frame is {obs.get('frame_id')!r}, not map")
    if obs.get("child_frame_id") != "base_link":
        reasons.append(f"child frame is {obs.get('child_frame_id')!r}, not base_link")
    twist = obs.get("twist_linear_x", None)
    if not _finite(twist):
        reasons.append("twist.linear.x is missing or NaN")
    if not obs.get("gnss_closed", False):
        reasons.append("GNSS subscriptions did not close")
    db3 = [str(p) for p in obs.get("db3_in_git") or []]
    if db3:
        reasons.append("recorded .db3 in the git tree: " + ", ".join(db3))
    return reasons


def _position_failures(obs: dict) -> list[str]:
    """Same 10 Hz / 0.25 s / no-regression contract, on /result/position."""
    stamps = np.asarray(obs.get("position_stamps") or [], float)
    report = output_rate_report(stamps)
    obs["position_rate"] = report["rate_unique_stamp_hz"]
    obs["position_gap"] = report["max_gap"]
    obs["position_regressed_stamp_count"] = int(report["regressed_stamp_count"])
    reasons: list[str] = []
    if obs["position_regressed_stamp_count"] > 0:
        reasons.append(f"position stamp regressed ({obs['position_regressed_stamp_count']})")
    rate = obs["position_rate"]
    if not _finite(rate) or float(rate) < MIN_RATE_HZ:
        reasons.append(f"position frequency {rate} Hz is below {MIN_RATE_HZ:g} Hz")
    gap = obs["position_gap"]
    if not _finite(gap) or float(gap) > MAX_GAP_S:
        reasons.append(f"position max gap {gap} s is above {MAX_GAP_S:g} s")
    if not obs.get("position_finite_xyz", False):
        reasons.append("position x/y/z is not finite")
    return reasons


def list_db3(root: Path) -> list[str]:
    root = root.resolve()
    found = []
    for path in root.rglob("*.db3"):
        if not path.is_file():
            continue
        if ".git" in path.relative_to(root).parts:
            continue
        found.append(str(path.resolve()))
    return sorted(found)


def unexpected_db3(before: list[str], after: list[str], allow_dirs: list[Path]) -> list[str]:
    """New .db3 files that are not under an allowed record directory."""
    previous = {str(Path(p).resolve()) for p in before}
    allowed = [p.resolve() for p in allow_dirs]
    extra = []
    for raw in after:
        path = Path(raw).resolve()
        if str(path) in previous:
            continue
        if any(path == allow or allow in path.parents for allow in allowed):
            continue
        extra.append(str(path))
    return sorted(extra)


def observe_result(bag: Path) -> dict:
    obs = {
        "have_velocity": False,
        "have_position": False,
        "velocity_stamps": [],
        "position_stamps": [],
        "position_finite_xyz": True,
        "frame_id": None,
        "child_frame_id": None,
        "twist_linear_x": None,
        "gnss_closed": False,
    }
    meta = bag / "metadata.yaml"
    if not meta.is_file():
        return obs
    gnss = None
    for _ts, topic, typ, blob in iter_messages(bag):
        rec = decode_message(typ, blob)
        if rec is None:
            continue
        if topic == "/result/velocity":
            obs["have_velocity"] = True
            if "stamp_s" in rec:
                obs["velocity_stamps"].append(float(rec["stamp_s"]))
        elif topic == "/result/position":
            obs["have_position"] = True
            obs["frame_id"] = rec.get("frame_id")
            obs["child_frame_id"] = rec.get("child_frame_id")
            obs["twist_linear_x"] = rec.get("v")
            if "stamp_s" in rec:
                obs["position_stamps"].append(float(rec["stamp_s"]))
            if not all(_finite(rec.get(key)) for key in ("s", "y", "z")):
                obs["position_finite_xyz"] = False
        elif topic == "/result/diagnostics":
            vals = rec.get("values") or {}
            if "gnss" in vals:
                gnss = vals["gnss"]
    obs["gnss_closed"] = gnss == "closed"
    return obs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Fail-closed jury acceptance checks")
    ap.add_argument("--result", type=Path)
    ap.add_argument("--node-alive", type=int, choices=(0, 1), default=1)
    ap.add_argument("--scorer-status", type=int, default=0)
    ap.add_argument("--require-position", type=int, choices=(0, 1), default=1)
    ap.add_argument("--db3", action="append", default=[])
    ap.add_argument("--list-db3", type=Path)
    ap.add_argument("--diff-db3", type=Path, help="file of .db3 paths from before the run")
    ap.add_argument("--git-root", type=Path)
    ap.add_argument("--allow", type=Path, action="append", default=[])
    args = ap.parse_args(argv)
    if args.list_db3:
        for path in list_db3(args.list_db3):
            print(path)
        return 0
    reasons: list[str] = []
    if args.diff_db3:
        if args.git_root is None:
            print("diff-db3 needs --git-root", file=sys.stderr)
            return 2
        before = []
        for ln in args.diff_db3.read_text(encoding="utf-8-sig").splitlines():
            ln = ln.strip().lstrip("\ufeff")
            if ln:
                before.append(ln)
        reasons.extend(
            f"recorded .db3 in the git tree: {path}"
            for path in unexpected_db3(before, list_db3(args.git_root), args.allow)
        )
    if args.result is not None:
        obs = observe_result(args.result)
        obs["node_alive"] = bool(args.node_alive)
        obs["scorer_status"] = args.scorer_status
        obs["require_position"] = bool(args.require_position)
        obs["db3_in_git"] = list(args.db3)
        reasons.extend(acceptance_failures(obs))
    if not reasons and args.result is None and args.diff_db3 is None:
        print("nothing to check", file=sys.stderr)
        return 2
    if reasons:
        print(json.dumps({"ok": False, "reasons": reasons}, ensure_ascii=False, indent=1))
        return 1
    print(json.dumps({"ok": True, "reasons": []}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

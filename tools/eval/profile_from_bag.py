"""Offline path height from a rosbag2 3D pose. Never a filter input.

If lidar / CBT localization publishes Odometry or PoseStamped, write
s, z, dh/ds for the generator and for a pitch slide «F_bias vs profile».
Does not import the UKF. DEM is not used here (DSM on the bridge is junk).

Usage:
  python tools/eval/profile_from_bag.py data/bags/run01 --out data/route_10_profile.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from rosbag2_io import decode_message, iter_messages, list_topics, read_metadata  # noqa: E402

# Same cap as inspect_bag.probe_bag. 0 in extract_profile means no cap.
MAX_MESSAGES = 250_000

SKIP_TOPICS = {"/tram/state_estimate"}

ODOMETRY_TYPES = {
    "nav_msgs/Odometry",
    "nav_msgs/msg/Odometry",
}
POSE_STAMPED_TYPES = {
    "geometry_msgs/PoseStamped",
    "geometry_msgs/msg/PoseStamped",
}


def _norm_type(typ: str) -> str:
    return typ.replace("/msg/", "/")


def _is_odometry_type(typ: str) -> bool:
    return _norm_type(typ) in {t.replace("/msg/", "/") for t in ODOMETRY_TYPES}


def _is_pose_stamped_type(typ: str) -> bool:
    return _norm_type(typ) in {t.replace("/msg/", "/") for t in POSE_STAMPED_TYPES}


def _is_height_type(typ: str) -> bool:
    return _is_odometry_type(typ) or _is_pose_stamped_type(typ)


def _prefer_key(name: str) -> tuple[int, str]:
    n = name.lower()
    if name in SKIP_TOPICS:
        return (9, name)
    if "/gt/" in n:
        return (0, name)
    if "lidar" in n or "localiz" in n or "cbt" in n:
        return (1, name)
    if "odom" in n:
        return (2, name)
    return (3, name)


def _choose_topic(
    topics: list[dict],
    by_name: dict[str, str],
    topic: str | None,
) -> str:
    if topic is not None:
        if topic not in by_name:
            raise SystemExit(f"topic not in bag: {topic}")
        typ = by_name[topic]
        if not _is_height_type(typ):
            raise SystemExit(
                f"{topic} type {typ} is not Odometry or PoseStamped; "
                "cannot build h(s) (and must not be a filter input)"
            )
        return topic

    odom = sorted(
        (t["name"] for t in topics if _is_odometry_type(t["type"]) and t["name"] not in SKIP_TOPICS),
        key=_prefer_key,
    )
    poses = sorted(
        (t["name"] for t in topics if _is_pose_stamped_type(t["type"]) and t["name"] not in SKIP_TOPICS),
        key=_prefer_key,
    )
    if "/gt/odometry" in odom:
        return "/gt/odometry"
    if odom:
        return odom[0]
    if "/gt/pose" in poses:
        return "/gt/pose"
    if poses:
        return poses[0]
    raise SystemExit(
        "no nav_msgs/Odometry or geometry_msgs/PoseStamped with z; "
        "lidar height is not in this bag"
    )


def extract_profile(
    bagdir: Path,
    *,
    topic: str | None = None,
    min_dz: float = 0.2,
    max_messages: int = MAX_MESSAGES,
) -> list[dict]:
    info = read_metadata(bagdir)
    topics = list_topics(info)
    by_name = {t["name"]: t["type"] for t in topics}
    chosen = _choose_topic(topics, by_name, topic)
    chosen_type = by_name[chosen]

    rows: list[dict] = []
    s_cum = 0.0
    prev: tuple[float, float, float] | None = None
    n_seen = 0
    n_dec = 0
    truncated = False
    for _ts, name, typ, blob in iter_messages(bagdir):
        n_seen += 1
        if max_messages > 0 and n_seen > max_messages:
            truncated = True
            break
        if name != chosen:
            continue
        rec = decode_message(typ, blob)
        if rec is None:
            continue
        n_dec += 1
        try:
            x = float(rec.get("s", 0.0))
            y = float(rec.get("y", 0.0))
            z = float(rec.get("z", 0.0))
        except (TypeError, ValueError):
            continue
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(z):
            continue
        if prev is None:
            prev = (x, y, z)
            rows.append({"s_m": 0.0, "z_m": z, "i_est": 0.0})
            continue
        ds = math.hypot(x - prev[0], y - prev[1])
        if ds < 0.05:
            continue
        s_cum += ds
        i_est = (z - prev[2]) / ds
        if not math.isfinite(i_est):
            continue
        rows.append({"s_m": round(s_cum, 3), "z_m": z, "i_est": i_est})
        prev = (x, y, z)

    if n_dec == 0:
        raise SystemExit(
            f"topic {chosen} ({chosen_type}) present but not decoded — "
            "need Odometry or PoseStamped with finite z"
        )
    zs = [r["z_m"] for r in rows]
    span = (max(zs) - min(zs)) if zs else 0.0
    if span < min_dz:
        extra = f"; scan truncated at {max_messages} messages" if truncated else ""
        raise SystemExit(
            f"{chosen}: z span {span:.3f} m < {min_dz} m — 1D odometry, not a height profile{extra}"
        )
    if truncated:
        sys.stderr.write(
            f"profile_from_bag: stopped at {max_messages} messages (partial h(s))\n"
        )
    return rows


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Offline h(s) from bag pose.z. Not a filter measurement."
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("--topic", default=None, help="Odometry or PoseStamped; default auto")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument(
        "--max-messages",
        type=int,
        default=MAX_MESSAGES,
        help="scan cap (same default as inspect_bag); 0 = no cap",
    )
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    rows = extract_profile(args.bag, topic=args.topic, max_messages=args.max_messages)
    out = args.out or (args.bag / "route_profile.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["s_m", "z_m", "i_est"])
        w.writeheader()
        w.writerows(rows)
    print(
        f"wrote {out} n={len(rows)} "
        f"z=[{rows[0]['z_m']:.2f},{rows[-1]['z_m']:.2f}] "
        "(offline only; do not load i(s) in the node)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

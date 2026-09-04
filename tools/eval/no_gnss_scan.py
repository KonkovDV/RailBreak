"""GNSS_IN_FILTER: estimator / lib must not subscribe to GNSS, IMU, lidar, cameras."""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NODE_DIR = ROOT / "tram_dr_localization" / "src" / "nodes"
LIB = ROOT / "tram_dr_localization" / "src" / "lib"

FORBIDDEN = [
    re.compile(r"create_subscription\s*<\s*sensor_msgs::msg::Imu"),
    re.compile(r"create_subscription\s*<\s*sensor_msgs::msg::NavSatFix"),
    re.compile(r"create_subscription\s*<\s*sensor_msgs::msg::PointCloud2"),
    re.compile(r"create_subscription\s*<\s*sensor_msgs::msg::Image"),
    re.compile(r"#include\s+<sensor_msgs/msg/imu"),
    re.compile(r"#include\s+<sensor_msgs/msg/point_cloud"),
]
JARGON = re.compile(r"\bfossil\b", re.I)


def scan(path: Path) -> tuple[list[str], list[str]]:
    text = path.read_text(encoding="utf-8")
    hits = []
    jargon = []
    for i, line in enumerate(text.splitlines(), start=1):
        loc = f"{path.as_posix()}:{i}: {line.strip()}"
        for pat in FORBIDDEN:
            if pat.search(line):
                hits.append(loc)
        if JARGON.search(line):
            jargon.append(loc)
    return hits, jargon


def main() -> int:
    files = [
        *[p for p in sorted(NODE_DIR.glob("*.cpp")) if p.name != "map_projector_node.cpp"],
        *sorted(LIB.glob("*.cpp")),
    ]
    hits: list[str] = []
    jargon: list[str] = []
    for f in files:
        if f.is_file():
            h, j = scan(f)
            hits.extend(h)
            jargon.extend(j)
    if hits:
        sys.stderr.write("GNSS_IN_FILTER:\n")
        for h in hits:
            sys.stderr.write(f"  {h}\n")
    if jargon:
        sys.stderr.write("DIAG_JARGON:\n")
        for h in jargon:
            sys.stderr.write(f"  {h}\n")
    if hits or jargon:
        return 2
    sys.stdout.write("GNSS_IN_FILTER empty\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

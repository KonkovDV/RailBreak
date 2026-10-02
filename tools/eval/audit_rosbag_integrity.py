"""Fail-closed integrity audit for rosbag2 SQLite metadata.

The rosbag2 metadata file is not evidence by itself: it duplicates message
counts, time bounds, and per-file records that must agree with the SQLite
payload. This tool checks those copies without importing ROS.

Exit status is non-zero for any mismatch. It intentionally does not rewrite a
bag; repair must be performed by re-recording or by an explicitly reviewed
conversion step.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

from rosbag2_io import read_metadata, sqlite_paths


@dataclass(frozen=True)
class FileFacts:
    path: str
    message_count: int
    start_ns: int | None
    end_ns: int | None

    @property
    def duration_ns(self) -> int | None:
        if self.start_ns is None or self.end_ns is None:
            return None
        return self.end_ns - self.start_ns


def _actual_file_facts(path: Path) -> FileFacts:
    con = sqlite3.connect(f"file:{path.resolve().as_posix()}?mode=ro", uri=True)
    try:
        count, start, end = con.execute(
            "SELECT count(*), min(timestamp), max(timestamp) FROM messages"
        ).fetchone()
    finally:
        con.close()
    return FileFacts(str(path), int(count), None if start is None else int(start), None if end is None else int(end))


def _actual_topic_counts(paths: list[Path]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for path in paths:
        con = sqlite3.connect(f"file:{path.resolve().as_posix()}?mode=ro", uri=True)
        try:
            rows = con.execute(
                "SELECT topics.name, count(messages.id) "
                "FROM topics LEFT JOIN messages ON messages.topic_id = topics.id "
                "GROUP BY topics.id, topics.name"
            )
            for name, count in rows:
                counts[str(name)] = counts.get(str(name), 0) + int(count)
        finally:
            con.close()
    return counts


def _expected_file_entries(info: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for entry in info.get("files") or []:
        if not isinstance(entry, dict) or not entry.get("path"):
            continue
        result[str(entry["path"])] = entry
    return result


def audit_bag(bagdir: Path) -> dict[str, Any]:
    """Return a JSON-serialisable report. `ok` is false for every mismatch."""
    info = read_metadata(bagdir)
    paths = sqlite_paths(bagdir, info)
    issues: list[str] = []
    if not paths:
        issues.append("no SQLite bag files resolved from metadata")

    facts = [_actual_file_facts(path) for path in paths]
    actual_total = sum(f.message_count for f in facts)
    actual_start = min((f.start_ns for f in facts if f.start_ns is not None), default=None)
    actual_end = max((f.end_ns for f in facts if f.end_ns is not None), default=None)

    declared_total = info.get("message_count")
    if declared_total is not None and int(declared_total) != actual_total:
        issues.append(f"message_count metadata={declared_total} sqlite={actual_total}")

    declared_start = (info.get("starting_time") or {}).get("nanoseconds_since_epoch")
    declared_duration = (info.get("duration") or {}).get("nanoseconds")
    if declared_start is not None and actual_start is not None and int(declared_start) != actual_start:
        issues.append(f"starting_time metadata={declared_start} sqlite={actual_start}")
    if declared_duration is not None and actual_start is not None and actual_end is not None:
        actual_duration = actual_end - actual_start
        if int(declared_duration) != actual_duration:
            issues.append(f"duration metadata={declared_duration} sqlite={actual_duration}")

    expected_topics = {
        str(item.get("topic_metadata", {}).get("name")): int(item.get("message_count") or 0)
        for item in info.get("topics_with_message_count") or []
        if item.get("topic_metadata", {}).get("name")
    }
    actual_topics = _actual_topic_counts(paths)
    if expected_topics != actual_topics:
        for name in sorted(set(expected_topics) | set(actual_topics)):
            if expected_topics.get(name) != actual_topics.get(name):
                issues.append(
                    f"topic {name} metadata={expected_topics.get(name)} sqlite={actual_topics.get(name)}"
                )

    expected_files = _expected_file_entries(info)
    actual_file_names = {Path(f.path).name for f in facts}
    declared_file_names = {Path(name).name for name in expected_files}
    for missing in sorted(actual_file_names - declared_file_names):
        issues.append(f"SQLite file missing from metadata files: {missing}")
    for extra in sorted(declared_file_names - actual_file_names):
        issues.append(f"metadata files entry has no SQLite payload: {extra}")

    for fact in facts:
        entry = next((v for k, v in expected_files.items() if Path(k).name == Path(fact.path).name), None)
        if not entry:
            continue
        if entry.get("message_count") is not None and int(entry["message_count"]) != fact.message_count:
            issues.append(
                f"file {Path(fact.path).name} message_count metadata={entry['message_count']} sqlite={fact.message_count}"
            )
        e_start = (entry.get("starting_time") or {}).get("nanoseconds_since_epoch")
        e_duration = (entry.get("duration") or {}).get("nanoseconds")
        if e_start is not None and fact.start_ns is not None and int(e_start) != fact.start_ns:
            issues.append(
                f"file {Path(fact.path).name} starting_time metadata={e_start} sqlite={fact.start_ns}"
            )
        if e_duration is not None and fact.duration_ns is not None and int(e_duration) != fact.duration_ns:
            issues.append(
                f"file {Path(fact.path).name} duration metadata={e_duration} sqlite={fact.duration_ns}"
            )

    return {
        "bag": str(bagdir),
        "ok": not issues,
        "issues": issues,
        "declared_message_count": declared_total,
        "actual_message_count": actual_total,
        "declared_topics": expected_topics,
        "actual_topics": actual_topics,
        "files": [{**asdict(f), "duration_ns": f.duration_ns} for f in facts],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args()
    try:
        report = audit_bag(args.bag)
    except (OSError, sqlite3.Error, ValueError, KeyError) as exc:
        print(f"rosbag integrity audit error: {exc}")
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        status = "PASS" if report["ok"] else "FAIL"
        print(f"{status}: {args.bag}")
        print(f"messages: declared={report['declared_message_count']} actual={report['actual_message_count']}")
        for issue in report["issues"]:
            print(f"- {issue}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

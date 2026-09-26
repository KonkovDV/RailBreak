"""The published-number manifest names every provenance field.

A null is an absent run record. A sha256 is checked against git, not against
a re-run. This test does not execute the odometer.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "docs" / "solution" / "provenance.json"
FIELDS = (
    "commit_sha",
    "evaluator_sha256",
    "bag_id",
    "input_sha256",
    "parameters",
    "frame",
    "gate",
    "playback_rate",
    "docker_image_digest",
    "date",
    "command",
)


def git_bytes(rev: str, path: str) -> bytes:
    return subprocess.check_output(["git", "show", f"{rev}:{path}"], cwd=ROOT)


def check_blob(spec: dict, label: str, errors: list[str]) -> None:
    rev = spec.get("commit")
    path = spec.get("path")
    digest = spec.get("sha256")
    if not rev or not path or not digest:
        errors.append(f"{label}: blob spec needs commit, path, sha256")
        return
    data = git_bytes(rev, path)
    got = hashlib.sha256(data).hexdigest()
    if got != digest:
        errors.append(f"{label}: {path}@{rev} sha256 {got} != {digest}")


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    errors: list[str] = []
    if data.get("fields") != list(FIELDS):
        errors.append("manifest field list drifted")
    if data.get("published_numbers_recomputed_after_filter_change") is not False:
        errors.append("manifest claims a rescore that was not run")
    for path, spec in data.get("tree_sha256", {}).items():
        check_blob({**spec, "path": path}, f"tree {path}", errors)
    results = data.get("results") or []
    if not results:
        errors.append("no results")
    seen = set()
    for row in results:
        rid = row.get("id")
        if rid in seen:
            errors.append(f"duplicate id {rid}")
        seen.add(rid)
        for key in FIELDS:
            if key not in row:
                errors.append(f"{rid}: missing {key}")
        if row.get("date") is not None:
            errors.append(f"{rid}: run date was not stored and must stay null")
        if row.get("input_sha256") is not None:
            errors.append(f"{rid}: bag bytes are not in git and must stay null")
        digest = row.get("docker_image_digest")
        if digest is not None and not (
            isinstance(digest, str) and len(digest) > 7 and digest.startswith("sha256:")
        ):
            errors.append(f"{rid}: docker digest must be null or sha256:...")
        ev = row.get("evaluator_sha256")
        if ev is not None:
            check_blob(ev, rid or "result", errors)
        companion = row.get("filter_sha256_before_last_change")
        if companion is not None:
            check_blob(companion, f"{rid} filter", errors)
    for error in errors:
        print(error)
    if errors:
        return 1
    print(f"provenance: {len(results)} results; hashes match git; absent run fields stay null")
    return 0


if __name__ == "__main__":
    sys.exit(main())

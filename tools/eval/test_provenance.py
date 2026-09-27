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
    if data.get("package_tree_commit") != "21ce24a76f03ade4c6307c091792fd44775421df":
        errors.append("package tree commit is not 21ce24a")
    if data.get("filter_commit") != data.get("package_tree_commit"):
        errors.append("filter commit is not the submission package tree")
    if data.get("metrics_commit") != "6af0037710baa3b67df2cc5720c6cc3272719712":
        errors.append("metrics commit is not 6af0037")
    if data.get("metrics_valid_for_current_head") is not False:
        errors.append("metrics are marked valid for the package tree")
    if data.get("current_replay_status") != "pending":
        errors.append("current replay status is not pending")
    if data.get("package_tree_commit") == data.get("metrics_commit"):
        errors.append("metrics commit was overwritten with the submission tree")
    if data.get("filter_commit_used_for_published_metrics") != data.get("metrics_commit"):
        errors.append("published metrics are not pinned to metrics_commit")
    if data.get("published_metrics_valid_for_current_head") is not False:
        errors.append("published metrics are marked valid for HEAD")
    if data.get("current_head_replay_status") != "pending":
        errors.append("HEAD replay is not pending")
    if data.get("published_numbers_recomputed_after_filter_change") is not False:
        errors.append("recheck flag claims the current filter was remeasured")
    if data.get("recheck_commit") != "6af0037710baa3b67df2cc5720c6cc3272719712":
        errors.append("recheck commit is not the tree that was run")
    ident = data.get("identity") or {}
    if ident.get("head_commit") != "efc43670068e2651cf039c3038f7ce9dd3157afc":
        errors.append("head commit is not the parent named when the identity block was written")
    if ident.get("runtime_tree_commit") != data.get("package_tree_commit"):
        errors.append("runtime tree is not the package tree")
    if ident.get("manifest_commit") != ident.get("head_commit"):
        errors.append("manifest commit is not the same parent as head")
    if ident.get("asset_pin_commit") != "2ce42d7101dced2e32d74f0e5fdc786e48d273a5":
        errors.append("asset pin is not 2ce42d7")
    if ident.get("metrics_commit") != data.get("metrics_commit"):
        errors.append("identity metrics commit drifted")
    if ident.get("evaluator_commit") is not None:
        errors.append("a single evaluator commit was invented")
    if ident.get("metrics_valid_for_current_head") is not False:
        errors.append("identity marks metrics valid for HEAD")
    if ident.get("current_replay_status") != "pending":
        errors.append("identity replay is not pending")
    if ident.get("head_commit") == "f6a21e91b084663591faef567f6d170672f5bac6":
        errors.append("f6a21e9 was written as the cloneable head")
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
        if row.get("timestamp") is not None:
            errors.append(f"{rid}: timestamp was not stored and must stay null")
        for key in ("bag_sha256", "params_sha256", "assets_sha256", "timestamp", "head_commit", "runtime_tree_commit"):
            if key not in row:
                errors.append(f"{rid}: missing {key}")
        if row.get("input_sha256") is not None or row.get("bag_sha256") is not None:
            errors.append(f"{rid}: bag bytes are not in git and must stay null")
        if row.get("params_sha256") is not None or row.get("assets_sha256") is not None:
            errors.append(f"{rid}: run asset hashes were not stored and must stay null")
        if row.get("head_commit") is not None or row.get("runtime_tree_commit") is not None:
            errors.append(f"{rid}: the run did not record head or runtime, so they stay null")
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
        measured = row.get("measured") or {}
        if rid == "ros-record-e9a34502-recheck" and measured.get("matches_published_0_77") is not False:
            errors.append("record recheck hides the 0.77 miss")
        if rid == "offline-val-recheck":
            if not (measured.get("along_rmse_median_m") < 1.467):
                errors.append("val recheck median is above 1.467")
            if measured.get("printed_p95_m") != 5.828:
                errors.append("val recheck print is not 5.828")
    for error in errors:
        print(error)
    if errors:
        return 1
    print(f"provenance: {len(results)} results; hashes match git; absent run fields stay null")
    return 0


if __name__ == "__main__":
    sys.exit(main())

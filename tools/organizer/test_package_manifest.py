"""The submission manifest names the package tree and does not hash bags."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "railbreak_backup_odometry" / "MANIFEST.json"


def git_bytes(rev: str, path: str) -> bytes:
    return subprocess.check_output(["git", "show", f"{rev}:{path}"], cwd=ROOT)


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    errors: list[str] = []
    if data.get("submission_package") != "railbreak_backup_odometry":
        errors.append("package name")
    if data.get("model_version") != "railbreak-backup-odometry-1":
        errors.append("model version")
    commit = data.get("commit")
    if not isinstance(commit, str) or len(commit) != 40:
        errors.append("commit is not a full sha")
    else:
        kind = subprocess.check_output(["git", "cat-file", "-t", commit], cwd=ROOT, text=True).strip()
        if kind != "commit":
            errors.append(f"commit object is {kind}")
        ancestor = subprocess.call(["git", "merge-base", "--is-ancestor", commit, "HEAD"], cwd=ROOT)
        if ancestor != 0:
            errors.append("commit is not an ancestor of HEAD")
    paths = data.get("asset_paths") or {}
    assets = data.get("assets") or {}
    for key, digest in assets.items():
        path = paths.get(key)
        if not path:
            errors.append(f"no path for {key}")
            continue
        got = hashlib.sha256(git_bytes(commit, path)).hexdigest()
        if got != digest:
            errors.append(f"{path}@{commit} sha256 {got} != {digest}")
    defaults = data.get("defaults") or {}
    if defaults.get("wheel_unit_scale") != 0.2777777777777778:
        errors.append("wheel_unit_scale")
    if defaults.get("sigma_k0") != 0.004:
        errors.append("sigma_k0")
    if defaults.get("output_frame") != "mgrs":
        errors.append("output_frame")
    params = (ROOT / "railbreak_backup_odometry/config/params.yaml").read_text(encoding="utf-8")
    for line in (
        "wheel_unit_scale: 0.2777777777777778",
        "sigma_k0: 0.004",
        "output_frame: mgrs",
    ):
        if line not in params:
            errors.append(f"params.yaml missing {line}")
    metrics = data.get("validated_metrics") or {}
    if metrics.get("along_rmse_median_m") != 1.467 or metrics.get("along_rmse_p95_m") != 5.828:
        errors.append("validated metrics are not the published val pair")
    prov = data.get("metric_provenance") or {}
    if prov.get("status") != "author-local-run":
        errors.append("provenance status")
    if prov.get("commands") != ["python tools/organizer/eval_odometer.py --split val"]:
        errors.append("provenance command")
    if prov.get("published_numbers_recomputed_after_filter_change") is not False:
        errors.append("manifest claims a rescore")
    if prov.get("commit_sha") is not None or prov.get("bag_hashes_included") is not False:
        errors.append("bag or metric commit was filled in")
    blob = json.dumps(data)
    if "input_sha256" in blob or "bag_sha256" in blob:
        errors.append("a bag hash field is present")
    for error in errors:
        print(error)
    if errors:
        return 1
    print("manifest: asset hashes match the named commit; bag hashes stay out")
    return 0


if __name__ == "__main__":
    sys.exit(main())

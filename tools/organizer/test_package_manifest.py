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
    if prov.get("commands") != [
        "python tools/organizer/eval_odometer.py --split val",
        "python tools/organizer/eval_odometer.py --split val --fault scale_rear_5pct",
    ]:
        errors.append("provenance command")
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
    if prov.get("metrics_commit") != data.get("metrics_commit"):
        errors.append("metric block metrics_commit drifted")
    if prov.get("filter_commit") != data.get("filter_commit"):
        errors.append("metric block filter_commit drifted")
    if prov.get("metrics_valid_for_current_head") is not False:
        errors.append("metric block marks the numbers valid for the package tree")
    if prov.get("current_replay_status") != "pending":
        errors.append("metric block replay status is not pending")
    cmake = (ROOT / "railbreak_backup_odometry" / "CMakeLists.txt").read_text(encoding="utf-8")
    if "ament_add_gtest(test_gnss_stress" not in cmake:
        errors.append("package CMake does not register test_gnss_stress")
    if "add_executable(test_core" not in cmake or "add_test(NAME test_core" not in cmake:
        errors.append("package CMake hides test_core")
    tools_cmake = (ROOT / "railbreak_backup_odometry" / "tools" / "CMakeLists.txt").read_text(encoding="utf-8")
    if "add_test(NAME test_core" not in tools_cmake:
        errors.append("host CMake does not register test_core")
    if data.get("published_metrics_valid_for_current_head") is not False:
        errors.append("published metrics are marked valid for HEAD")
    if data.get("current_head_replay_status") != "pending":
        errors.append("HEAD replay is not pending")
    if prov.get("published_numbers_recomputed_after_filter_change") is not False:
        errors.append("recheck flag claims the current filter was remeasured")
    if prov.get("filter_commit_used_for_published_metrics") != "6af0037710baa3b67df2cc5720c6cc3272719712":
        errors.append("metric block is not pinned to 6af0037")
    if prov.get("published_metrics_valid_for_current_head") is not False:
        errors.append("metric block marks the numbers valid for HEAD")
    if prov.get("current_head_replay_status") != "pending":
        errors.append("metric block replay status is not pending")
    if prov.get("commit_sha") != "6af0037710baa3b67df2cc5720c6cc3272719712":
        errors.append("metric commit is not the recheck tree")
    if prov.get("ros_record_matches_published_0_77") is not False:
        errors.append("manifest treats the rate-1 record as 0.77")
    if prov.get("bag_hashes_included") is not False:
        errors.append("bag hash was filled in")
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

"""Hash synth runs + UKF metrics into an evidence pack.

Reads synth/runs/*/ (run.csv, filter.csv, gt.jsonl, meta.json, ukf.jsonl),
scores every scenario with tools/synth/score.py logic, and writes
evidence/<pack>/{metrics.json, meta.json} where meta.json carries sha256 of
every input and a pack hash. Not a substitute for the rosbag evidence pack.

Usage: python tools/eval/pack_evidence.py [--runs DIR] [--out DIR]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "synth"))

from score import score_run  # noqa: E402

INPUT_NAMES = ("run.csv", "filter.csv", "gt.jsonl", "meta.json", "ukf.jsonl")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, default=ROOT / "synth" / "runs")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if not args.runs.is_dir():
        sys.stderr.write(f"missing {args.runs}; run generate.py + run_e2e.py first\n")
        return 1
    out = args.out or (ROOT / "evidence" / f"synth-{datetime.now(timezone.utc):%Y-%m-%d}")
    out.mkdir(parents=True, exist_ok=True)

    metrics: dict[str, dict] = {}
    files: dict[str, str] = {}
    seeds = set()
    for d in sorted(p for p in args.runs.iterdir() if p.is_dir()):
        if not (d / "run.csv").is_file():
            continue
        metrics[d.name] = score_run(d)
        for name in INPUT_NAMES:
            f = d / name
            if f.is_file():
                files[f"{d.name}/{name}"] = _sha256(f)
        meta_path = d / "meta.json"
        if meta_path.is_file():
            seeds.add(json.loads(meta_path.read_text(encoding="utf-8")).get("seed"))
    if not metrics:
        sys.stderr.write("no runs found\n")
        return 1

    pack_src = "\n".join(f"{k}:{v}" for k, v in sorted(files.items()))
    pack_hash = hashlib.sha256(pack_src.encode("utf-8")).hexdigest()

    (out / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    meta = {
        "created_utc": f"{datetime.now(timezone.utc):%Y-%m-%dT%H:%M:%SZ}",
        "kind": "synth evidence pack (not rosbag, not route 10)",
        "generator_seed": sorted(seeds),
        "estimator": "replay_ukf from working tree (libtram_dr UKF)",
        "files_sha256": files,
        "pack_sha256": pack_hash,
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(f"{out}  scenarios={len(metrics)}  pack_sha256={pack_hash[:16]}…")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Group identical wheel streams and propose a date split.

Reads local npz produced by the extractor. Writes JSON only when --out is set.
Does not read GNSS into the estimator.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

FRONT = "vehicle_front_bogie_velocity"


def wheel_hash(path: Path) -> str:
    z = np.load(path)
    arr = np.ascontiguousarray(z[FRONT][:, 1:])
    return hashlib.sha1(arr.tobytes()).hexdigest()


def start_utc(path: Path) -> dt.datetime:
    z = np.load(path)
    return dt.datetime.fromtimestamp(float(z[FRONT][0, 0]), dt.UTC)


def build(org: Path) -> dict:
    by_hash: dict[str, list[str]] = defaultdict(list)
    meta = {}
    for path in sorted(org.glob("*.npz")):
        by_hash[wheel_hash(path)].append(path.stem)
        meta[path.stem] = start_utc(path)
    groups = [sorted(v) for v in by_hash.values() if len(v) > 1]
    keep = sorted(v[0] for v in by_hash.values())
    # One representative per identical stream. Dates are calendar days of the recording.
    def day(name: str) -> str:
        return meta[name].date().isoformat()

    train, val, stress = [], [], []
    for name in keep:
        d = day(name)
        veh = name.split("_", 1)[0]
        if veh == "30618" and d in {"2026-07-27", "2026-08-10"}:
            train.append(name)
        elif d == "2026-08-26":
            val.append(name)
        else:
            stress.append(name)
    return {
        "n_files": len(meta),
        "n_unique": len(keep),
        "duplicate_groups": groups,
        "train": train,
        "val": val,
        "stress": stress,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("org", type=Path)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    doc = build(args.org)
    text = json.dumps(doc, indent=1)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(
        f"files={doc['n_files']} unique={doc['n_unique']} "
        f"dup_groups={len(doc['duplicate_groups'])} "
        f"train={len(doc['train'])} val={len(doc['val'])} stress={len(doc['stress'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Deterministic measurement-only fault injection. GT columns are untouched."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


FAULTS = {
    "slip_ramp_20": {"kind": "slip", "gain": 1.20, "all": True},
    "slip_ramp_50": {"kind": "slip", "gain": 1.50, "all": True},
    "slide_wsp": {"kind": "slide_cycle", "period_s": 0.4},
    "freeze_axle0": {"kind": "freeze", "axle": 0},
    "dropout_all": {"kind": "nan", "all": True},
    "nan_burst": {"kind": "nan_burst", "len": 25},
    "quantize": {"kind": "quantize", "step": 0.05},
    "delay_50ms": {"kind": "delay", "samples": 3},
    "delay_100ms": {"kind": "delay", "samples": 5},
    "scale_axle0": {"kind": "scale", "axle": 0, "gain": 0.97},
}


def _wheels(row: dict) -> list[str]:
    return [k for k in row if k.startswith("w") and k[1:].isdigit()]


def inject_rows(rows: list[dict], fault: str, seed: int = 42) -> list[dict]:
    spec = FAULTS[fault]
    rng = random.Random(seed)
    keys = _wheels(rows[0]) if rows else []
    out = []
    frozen = {k: None for k in keys}
    delay_buf: list[dict] = []
    for i, src in enumerate(rows):
        row = dict(src)
        t = float(row.get("t_s", row.get("t", i * 0.02)) or 0.0)
        kind = spec["kind"]
        if kind == "slip":
            for k in keys:
                v = row.get(k)
                if v not in (None, ""):
                    row[k] = float(v) * spec["gain"]
        elif kind == "slide_cycle":
            phase = (t / spec["period_s"]) % 1.0
            if phase < 0.5:
                for k in keys:
                    row[k] = 0.0
        elif kind == "freeze":
            axle = keys[spec["axle"]] if spec["axle"] < len(keys) else keys[0]
            if frozen[axle] is None:
                frozen[axle] = row.get(axle, 0.0)
            row[axle] = frozen[axle]
        elif kind == "nan":
            for k in keys:
                row[k] = float("nan")
        elif kind == "nan_burst":
            if 100 <= i < 100 + spec["len"]:
                for k in keys:
                    row[k] = float("nan")
        elif kind == "quantize":
            step = spec["step"]
            for k in keys:
                v = row.get(k)
                if v in (None, ""):
                    continue
                fv = float(v)
                if math.isfinite(fv):
                    row[k] = round(fv / step) * step
        elif kind == "delay":
            delay_buf.append(dict(src))
            if len(delay_buf) <= spec["samples"]:
                for k in keys:
                    row[k] = src.get(k)
            else:
                delayed = delay_buf[-(spec["samples"] + 1)]
                for k in keys:
                    row[k] = delayed.get(k)
        elif kind == "scale":
            axle = keys[spec["axle"]] if spec["axle"] < len(keys) else keys[0]
            v = row.get(axle)
            if v not in (None, ""):
                row[axle] = float(v) * spec["gain"]
        # Never rewrite GT.
        out.append(row)
        rng.random()  # keep the stream advancing for future stochastic faults
    return out


def inject_on_indices(rows: list[dict], indices: list[int], fault: str,
                      seed: int = 42) -> list[dict]:
    """Mutate wheels on `indices` only. Train/test and all GT columns stay put.

    The fault generator sees the selected block as a contiguous recording so
    delay/freeze state does not leak from train into val.
    """
    out = [dict(r) for r in rows]
    if not indices:
        return out
    block = [out[i] for i in indices]
    faulted = inject_rows(block, fault, seed)
    preserve = {"t_s", "t", "notch", "brake", "gt_s", "gt_v", "gt"}
    for i, fr in zip(indices, faulted):
        merged = dict(fr)
        orig = rows[i]
        for k in orig:
            if k in preserve or k.startswith("gt"):
                merged[k] = orig[k]
        out[i] = merged
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fault", required=True, choices=sorted(FAULTS))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args(argv)
    with args.csv.open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
        fieldnames = list(rows[0].keys()) if rows else []
    out_rows = inject_rows(rows, args.fault, args.seed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(out_rows)
    print(json.dumps({"fault": args.fault, "n": len(out_rows), "seed": args.seed}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

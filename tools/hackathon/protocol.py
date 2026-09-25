"""T0 protocol primitives: splits, data-contract text, CSV → rosbag2.

Does not import the UKF. Organiser bags stay under data/bags/ (gitignored).
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from rosbag2_io import (  # noqa: E402
    CANONICAL,
    encode_float32,
    encode_float64,
    encode_float64_array,
    write_bag,
)

DT = 0.02
MIN_BLOCK_S = 600.0  # plan §6: a split unit is a trip or ≥10 min


def _f(row: dict, key: str) -> float | None:
    raw = row.get(key)
    if raw in (None, ""):
        return None
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _t(row: dict, i: int) -> float:
    v = _f(row, "t_s")
    if v is None:
        v = _f(row, "t")
    return float(i * DT) if v is None else v


def split_indices(rows: list[dict], train: float = 0.6, val: float = 0.2,
                  test: float = 0.2) -> dict:
    """Time-fraction splits. Neighbouring 50 Hz frames are not i.i.d. trials."""
    n = len(rows)
    if n == 0:
        return {"train": [], "val": [], "test": [], "note": "empty"}
    t0 = _t(rows[0], 0)
    t1 = _t(rows[-1], n - 1)
    dur = max(t1 - t0, 0.0)
    note = ""
    if dur + 1e-9 < MIN_BLOCK_S:
        note = (
            f"duration {dur:.3f}s < {MIN_BLOCK_S:.0f}s plan block; "
            "fractions are still by time, not by i.i.d. frames"
        )
    parts: dict[str, list[int]] = {"train": [], "val": [], "test": []}
    for i, row in enumerate(rows):
        t = _t(row, i)
        frac = 0.0 if dur <= 0.0 else (t - t0) / dur
        if frac < train:
            parts["train"].append(i)
        elif frac < train + val:
            parts["val"].append(i)
        else:
            parts["test"].append(i)
    if not parts["train"]:
        parts["train"] = [0]
    if not parts["val"] and len(parts["train"]) > 1:
        parts["val"].append(parts["train"].pop())
    if not parts["test"] and parts["val"]:
        parts["test"].append(parts["val"].pop())
    return {
        "train": parts["train"],
        "val": parts["val"],
        "test": parts["test"],
        "t0_s": t0,
        "t1_s": t1,
        "duration_s": dur,
        "fractions": {"train": train, "val": val, "test": test},
        "note": note or "time-fraction split; test unused until freeze",
    }


def write_split_csvs(rows: list[dict], splits: dict, out: Path) -> dict[str, Path]:
    out.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0].keys()) if rows else ["t_s"]
    paths = {}
    for name in ("train", "val", "test"):
        path = out / f"{name}.csv"
        idx = splits.get(name, [])
        with path.open("w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for i in idx:
                w.writerow({k: rows[i].get(k, "") for k in keys})
        paths[name] = path
    (out / "splits.json").write_text(json.dumps(
        {k: v for k, v in splits.items() if k != "train" and k != "val" and k != "test"}
        | {"n_train": len(splits.get("train", [])),
           "n_val": len(splits.get("val", [])),
           "n_test": len(splits.get("test", []))},
        indent=2,
    ) + "\n", encoding="utf-8")
    return paths


def csv_to_bag(rows: list[dict], bagdir: Path) -> Path:
    """Canonical /tram/* Float32 + Float64MultiArray bag. No ROS runtime."""
    msgs: list[tuple[str, str, int, bytes]] = []
    for i, row in enumerate(rows):
        t_s = _t(row, i)
        t_ns = int(round(t_s * 1e9))
        n = _f(row, "notch")
        if n is not None:
            msgs.append((CANONICAL["notch"], "std_msgs/msg/Float32", t_ns, encode_float32(n)))
        b = _f(row, "brake")
        if b is not None:
            msgs.append((CANONICAL["brake"], "std_msgs/msg/Float32", t_ns, encode_float32(b)))
        wheels = []
        for k in range(6):
            w = _f(row, f"w{k}")
            if w is None:
                break
            wheels.append(w)
        if wheels:
            msgs.append((
                CANONICAL["wheels"],
                "std_msgs/msg/Float64MultiArray",
                t_ns,
                encode_float64_array(wheels),
            ))
        gv = _f(row, "gt_v")
        if gv is None:
            gv = _f(row, "v")
        if gv is not None:
            msgs.append(("/gt/v", "std_msgs/msg/Float64", t_ns, encode_float64(gv)))
        gs = _f(row, "gt_s")
        if gs is None:
            gs = _f(row, "s")
        if gs is not None:
            msgs.append(("/gt/s", "std_msgs/msg/Float64", t_ns, encode_float64(gs)))
    write_bag(bagdir, msgs)
    return bagdir


def data_contract_markdown(rows: list[dict], meta: dict) -> str:
    """Fill observed fields only. Unknown organiser facts stay TBD."""
    n = len(rows)
    t0 = _t(rows[0], 0) if rows else float("nan")
    t1 = _t(rows[-1], n - 1) if rows else float("nan")
    dt = (t1 - t0) / max(n - 1, 1) if n > 1 else float("nan")
    notches = [_f(r, "notch") for r in rows]
    brakes = [_f(r, "brake") for r in rows]
    nf = [v for v in notches if v is not None]
    bf = [v for v in brakes if v is not None]
    n_w = 0
    for r in rows[: min(n, 20)]:
        n_w = max(n_w, sum(1 for i in range(6) if _f(r, f"w{i}") is not None))
    neg_n = any(v < -0.05 for v in nf)
    has_gt = any(_f(r, "gt_v") is not None or _f(r, "gt_s") is not None for r in rows)
    lines = [
        "# Data contract",
        "",
        "Generated from observed rows. Slots that were not measured stay `TBD`.",
        "This is not the organiser specification and not a seed-42 table.",
        "",
        "| Field | Value |",
        "| --- | --- |",
        f"| source | {meta.get('source', 'TBD')} |",
        f"| n_frames | {n} |",
        f"| t0_s | {t0:.6g} |",
        f"| t1_s | {t1:.6g} |",
        f"| dt_med_s | {dt:.6g} |",
        f"| omega_unit | {meta.get('omega_unit', 'TBD')} |",
        f"| notch_encoding | {meta.get('notch_encoding', 'TBD')} |",
        f"| brake_source | {meta.get('brake_source', 'TBD')} |",
        f"| n_wheels | {meta.get('n_wheels', n_w)} |",
        f"| notch_min | {min(nf) if nf else 'TBD'} |",
        f"| notch_max | {max(nf) if nf else 'TBD'} |",
        f"| negative_notch | {'yes' if neg_n else 'no'} |",
        f"| brake_present | {'yes' if bf else 'no'} |",
        f"| gt_columns | {'yes' if has_gt else 'no'} |",
        f"| hidden_test | TBD |",
        f"| map_stops_allowed | TBD |",
        f"| vehicle | TBD |",
        "",
        "If `negative_notch` is yes and `brake_source` is none/topic-empty, "
        "set `brake_source=notch` before scoring. Do not tune on test.",
        "",
    ]
    return "\n".join(lines)

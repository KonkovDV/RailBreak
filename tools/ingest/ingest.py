"""Universal ingest: CSV / JSONL / parquet / rosbag2 → filter.csv + gt.jsonl.

Does not import the UKF. Autodetects ω units, notch scale, a structurally
absent brake, event-driven commands, and a one-channel odometer.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from rosbag2_io import decode_message, iter_messages, map_notch, pad_wheels, read_metadata  # noqa: E402

K_OMEGA_ABS_MAX = 80.0


def _finite(x: object) -> float | None:
    try:
        v = float(x)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def detect_omega_scale(samples: list[list[float]]) -> str:
    vals = [abs(w) for row in samples for w in row if math.isfinite(w)]
    if not vals:
        return "rad_s"
    peak = max(vals)
    if peak > K_OMEGA_ABS_MAX:
        return "rpm"
    return "rad_s"


def apply_omega_scale(row: list[float], unit: str) -> list[float]:
    if unit == "rpm":
        return [w * math.tau / 60.0 if math.isfinite(w) else w for w in row]
    return list(row)


def detect_notch_encoding(values: list[float]) -> tuple[str, float]:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return "normalized", 1.0
    peak = max(abs(v) for v in finite)
    if peak <= 1.0 + 1e-6:
        return "normalized", 1.0
    if peak <= 8.5:
        return "discrete", 8.0
    return "percent", 100.0


def map_notch_value(raw: float, enc: str, nmax: float) -> float:
    if not math.isfinite(raw):
        return raw
    if enc == "normalized":
        return max(-1.0, min(1.0, raw))
    return max(-1.0, min(1.0, raw / max(nmax, 1.0)))


def detect_brake_source(has_brake_column: bool, brake_values: list[float | None],
                        notch_values: list[float]) -> str:
    if not has_brake_column:
        if any(v < -0.05 for v in notch_values if math.isfinite(v)):
            return "notch"
        return "none"
    finite = [v for v in brake_values if v is not None and math.isfinite(v)]
    if not finite:
        return "none"
    return "topic"


def write_run_dir(out: Path, rows: list[dict], meta: dict) -> None:
    out.mkdir(parents=True, exist_ok=True)
    filter_path = out / "filter.csv"
    gt_path = out / "gt.jsonl"
    n_w = 0
    for row in rows:
        for i in range(6):
            v = _finite(row.get(f"w{i}"))
            if v is not None:
                n_w = max(n_w, i + 1)
    n_w = max(n_w, 1)
    keys = ["t_s", "notch", "brake"] + [f"w{i}" for i in range(n_w)]
    has_gt = any(
        row.get("gt_s") not in (None, "") or row.get("gt_v") not in (None, "")
        for row in rows
    )
    if has_gt:
        keys += ["gt_s", "gt_v"]
    with filter_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in rows:
            rec = {k: row.get(k, "") for k in keys}
            w.writerow(rec)
    with gt_path.open("w", encoding="utf-8") as f:
        for row in rows:
            gt_s = row.get("gt_s")
            gt_v = row.get("gt_v")
            if gt_s is None and gt_v is None:
                continue
            rec = {"t": row["t_s"]}
            if gt_s is not None:
                rec["s"] = gt_s
            if gt_v is not None:
                rec["v"] = gt_v
            f.write(json.dumps(rec) + "\n")
    (out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    qa = [
        f"# ingest QA",
        f"n={len(rows)}",
        f"omega_unit={meta.get('omega_unit')}",
        f"notch_encoding={meta.get('notch_encoding')}",
        f"brake_source={meta.get('brake_source')}",
        f"n_wheels={meta.get('n_wheels')}",
    ]
    (out / "qa.md").write_text("\n".join(qa) + "\n", encoding="utf-8")


def ingest_csv(path: Path) -> tuple[list[dict], dict]:
    with path.open(encoding="utf-8", newline="") as f:
        raw = list(csv.DictReader(f))
    if not raw:
        return [], {"source": str(path)}
    wheels: list[list[float]] = []
    notches: list[float | None] = []
    brakes: list[float | None] = []
    has_brake = any("brake" in r for r in raw)
    n_w = 0
    for r in raw:
        row_w = []
        for i in range(6):
            key = f"w{i}"
            if key in r and r[key] not in (None, ""):
                v = _finite(r[key])
                row_w.append(float("nan") if v is None else v)
        n_w = max(n_w, len(row_w))
        wheels.append(row_w)
        if "notch" not in r or r["notch"] in (None, ""):
            notches.append(None)
        else:
            notches.append(_finite(r.get("notch")))
        if has_brake:
            if r.get("brake") in (None, ""):
                brakes.append(None)
            else:
                brakes.append(_finite(r.get("brake")))
        else:
            brakes.append(None)
    last_n: float | None = None
    filled_notches: list[float] = []
    for nv in notches:
        if nv is None:
            filled_notches.append(last_n if last_n is not None else float("nan"))
        else:
            last_n = nv
            filled_notches.append(nv)
    last_b: float | None = None
    filled_brakes: list[float | None] = []
    for bv in brakes:
        if has_brake and bv is None:
            filled_brakes.append(last_b)
        else:
            if bv is not None:
                last_b = bv
            filled_brakes.append(bv)
    unit = detect_omega_scale(wheels)
    enc, nmax = detect_notch_encoding(filled_notches)
    src = detect_brake_source(has_brake, filled_brakes, filled_notches)
    out_rows = []
    for i, r in enumerate(raw):
        t = _finite(r.get("t_s", r.get("t", i * 0.02))) or 0.0
        w = apply_omega_scale(wheels[i] + [float("nan")] * (6 - len(wheels[i])), unit)
        rec = {
            "t_s": t,
            "notch": map_notch_value(filled_notches[i], enc, nmax),
            "brake": "" if filled_brakes[i] is None else filled_brakes[i],
            "w0": w[0], "w1": w[1], "w2": w[2], "w3": w[3], "w4": w[4], "w5": w[5],
        }
        if src == "none":
            rec["brake"] = 0.0
        gs = _finite(r.get("gt_s"))
        gv = _finite(r.get("gt_v"))
        if gs is not None:
            rec["gt_s"] = gs
        if gv is not None:
            rec["gt_v"] = gv
        out_rows.append(rec)
    meta = {
        "source": str(path),
        "omega_unit": unit,
        "notch_encoding": enc,
        "notch_max_abs": nmax,
        "brake_source": src,
        "n_wheels": max(n_w, 1),
    }
    return out_rows, meta


def ingest_jsonl(path: Path) -> tuple[list[dict], dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        rec = json.loads(line)
        rows.append({
            "t_s": rec.get("t", rec.get("t_s", 0.0)),
            "notch": rec["notch"] if "notch" in rec else "",
            "brake": rec["brake"] if "brake" in rec else "",
            **{f"w{i}": rec.get(f"w{i}", float("nan")) for i in range(6)},
            "gt_s": rec.get("gt_s", rec.get("s")),
            "gt_v": rec.get("gt_v", rec.get("v")),
        })
    tmp = Path(path.with_suffix(".tmpcsv"))
    # Reuse CSV detection on a synthesized table.
    keys = ["t_s", "notch", "brake"] + [f"w{i}" for i in range(6)] + ["gt_s", "gt_v"]
    with tmp.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})
    out, meta = ingest_csv(tmp)
    tmp.unlink(missing_ok=True)
    meta["source"] = str(path)
    return out, meta


def ingest_parquet(path: Path) -> tuple[list[dict], dict]:
    try:
        import pyarrow.parquet as pq  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyarrow is required for parquet ingest") from exc
    table = pq.read_table(path)
    cols = table.to_pydict()
    n = 0
    for v in cols.values():
        n = len(v)
        break
    keys = list(cols)
    csv_path = path.with_suffix(".ingested.csv")
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for i in range(n):
            w.writerow({k: cols[k][i] for k in keys})
    out, meta = ingest_csv(csv_path)
    csv_path.unlink(missing_ok=True)
    meta["source"] = str(path)
    return out, meta


def ingest_bag(path: Path) -> tuple[list[dict], dict]:
    md = read_metadata(path)
    topics = {t["name"]: t for t in md.get("topics", [])}
    notch_t = "/tram/controller_notch"
    brake_t = "/tram/brake_cmd"
    wheel_t = "/tram/wheel_odom"
    for name in topics:
        if "notch" in name:
            notch_t = name
        if "brake" in name:
            brake_t = name
        if "wheel" in name or name.endswith("wheel_odom"):
            wheel_t = name
    samples: dict[float, dict] = {}

    def payload(decoded: object) -> object:
        if decoded is None:
            return None
        if isinstance(decoded, dict) and "data" in decoded:
            return decoded["data"]
        return decoded

    for ts_ns, name, typ, blob in iter_messages(path):
        t = ts_ns * 1e-9
        rec = samples.setdefault(t, {"t_s": t})
        decoded = decode_message(typ, blob)
        body = payload(decoded)
        if name == notch_t and body is not None and not isinstance(body, list):
            rec["notch"] = float(body)
        elif name == brake_t and body is not None and not isinstance(body, list):
            rec["brake"] = float(body)
        elif name == wheel_t:
            wheels = body if isinstance(body, list) else ([body] if body is not None else [])
            wheels = [float(x) for x in wheels if x is not None and math.isfinite(float(x))]
            padded = pad_wheels(wheels, max(len(wheels), 1))
            for i, w in enumerate(padded):
                rec[f"w{i}"] = w
        elif name in ("/gt/s", "/gt/v") or name.startswith("/gt/"):
            if body is not None and not isinstance(body, list):
                if name.endswith("/s"):
                    rec["gt_s"] = float(body)
                elif name.endswith("/v"):
                    rec["gt_v"] = float(body)
    ordered = [samples[k] for k in sorted(samples)]
    tmp = path.parent / "_ingest_bag.csv"
    keys = ["t_s", "notch", "brake"] + [f"w{i}" for i in range(6)] + ["gt_s", "gt_v"]
    with tmp.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in ordered:
            w.writerow({k: r.get(k, "") for k in keys})
    out, meta = ingest_csv(tmp)
    tmp.unlink(missing_ok=True)
    meta["source"] = str(path)
    meta["bag_topics"] = list(topics)
    return out, meta


def ingest(path: Path) -> tuple[list[dict], dict]:
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return ingest_bag(path)
    suf = path.suffix.lower()
    if suf == ".csv":
        return ingest_csv(path)
    if suf in {".jsonl", ".ndjson"}:
        return ingest_jsonl(path)
    if suf in {".parquet", ".pq"}:
        return ingest_parquet(path)
    raise SystemExit(f"unsupported ingest input: {path}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    rows, meta = ingest(args.src)
    write_run_dir(args.out, rows, meta)
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

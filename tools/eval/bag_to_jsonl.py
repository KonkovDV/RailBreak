"""rosbag2 → JSONL (and optional filter.csv). Independent of UKF."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from rosbag2_io import (  # noqa: E402
    CANONICAL,
    DIAGNOSTICS_TOPIC,
    decode_message,
    iter_messages,
    map_notch,
    pad_wheels,
    read_metadata,
)

DEFAULT_YAML = ROOT / "tram_dr_localization" / "config" / "customer_topics.yaml"


def _parse_adapter_yaml(text: str) -> dict:
    try:
        import yaml  # type: ignore
    except ImportError:
        yaml = None  # type: ignore
    if yaml is not None:
        doc = yaml.safe_load(text) or {}
        return (doc.get("topic_adapter") or {}).get("ros__parameters") or doc
    params: dict = {}
    for line in text.splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, _, val = line.partition(":")
        key = key.strip()
        val = val.strip().strip("'\"")
        if key in {
            "in_notch_topic",
            "in_brake_topic",
            "in_wheels_topic",
            "in_notch_type",
            "in_wheels_type",
        }:
            params[key] = val
        elif key in {"enable", "in_notch_int8", "twist_is_omega"}:
            params[key] = val.lower() in {"true", "1", "yes"}
        elif key in {"notch_max_abs", "wheel_radius_m"}:
            try:
                params[key] = float(val)
            except ValueError:
                pass
        elif key in {"n_wheels"}:
            try:
                params[key] = int(val)
            except ValueError:
                pass
    return params


def _load_adapter(path: Path | None) -> tuple[dict[str, str], float, int, float, bool]:
    aliases = {
        CANONICAL["notch"]: "notch",
        CANONICAL["brake"]: "brake",
        CANONICAL["wheels"]: "wheels",
        CANONICAL["estimate"]: "est",
        DIAGNOSTICS_TOPIC: "diag",
        "/gt/s": "gt",
        "/gt/pose": "gt",
        "/gt/odometry": "gt",
    }
    notch_max_abs = 8.0
    n_wheels = 4
    wheel_radius_m = 0.35
    twist_is_omega = False
    if path is None or not path.is_file():
        return aliases, notch_max_abs, n_wheels, wheel_radius_m, twist_is_omega
    params = _parse_adapter_yaml(path.read_text(encoding="utf-8"))
    if params.get("in_notch_topic"):
        aliases[str(params["in_notch_topic"])] = "notch"
    if params.get("in_brake_topic"):
        aliases[str(params["in_brake_topic"])] = "brake"
    if params.get("in_wheels_topic"):
        aliases[str(params["in_wheels_topic"])] = "wheels"
    if params.get("notch_max_abs") is not None:
        notch_max_abs = float(params["notch_max_abs"])
    if params.get("n_wheels") is not None:
        n_wheels = max(1, min(6, int(params["n_wheels"])))
    if params.get("wheel_radius_m") is not None:
        wheel_radius_m = max(0.05, float(params["wheel_radius_m"]))
    if params.get("twist_is_omega") is not None:
        twist_is_omega = bool(params["twist_is_omega"])
    return aliases, notch_max_abs, n_wheels, wheel_radius_m, twist_is_omega


def _load_aliases(path: Path | None) -> dict[str, str]:
    aliases, _, _, _, _ = _load_adapter(path)
    return aliases


def _attach_diag_confidence(rows: list[dict], window_s: float = 0.05) -> None:
    """Copy KeyValue confidence from /tram/diagnostics onto nearby Odometry.

    Does not invent OK: no diagnostics → no confidence.
    Init P_ss ≥ 5e5 stays UNINITIALIZED even if a diag row says OK.
    """
    diags: list[tuple[float, str]] = []
    for r in rows:
        if r.get("kind") != "diag":
            continue
        conf = str(r.get("confidence") or "").upper()
        if conf in {"OK", "DEGRADED", "LOST", "UNINITIALIZED"}:
            diags.append((float(r["t"]), conf))
    if not diags:
        return
    for r in rows:
        if r.get("kind") != "est":
            continue
        if str(r.get("confidence") or "").upper() == "UNINITIALIZED":
            continue
        t = float(r["t"])
        best: str | None = None
        best_dt = window_s + 1.0
        for td, conf in diags:
            dt = abs(td - t)
            if dt <= window_s and dt < best_dt:
                best_dt = dt
                best = conf
        if best is not None:
            r["confidence"] = best
            r["confidence_src"] = "diagnostics"


def bag_to_rows(
    bagdir: Path,
    aliases: dict[str, str],
    *,
    notch_max_abs: float = 8.0,
    n_wheels: int = 4,
    wheel_radius_m: float = 0.35,
    twist_is_omega: bool = False,
) -> tuple[list[dict], list[str]]:
    read_metadata(bagdir)  # raise if not a bag
    from inspect_bag import guess_roles, probe_bag

    aliases = dict(aliases)
    guessed = guess_roles(probe_bag(bagdir)["stats"])
    role_keys = {"notch": "notch", "brake": "brake", "wheels": "wheels", "estimate": "est"}
    for role, dest in role_keys.items():
        name = guessed.get(role)
        if name:
            aliases.setdefault(name, dest)
    rows: list[dict] = []
    skipped: dict[str, int] = {}
    last: dict[str, float | list[float]] = {"notch": math.nan, "brake": math.nan}
    for ts_ns, name, typ, blob in iter_messages(bagdir):
        t = ts_ns * 1e-9
        role = aliases.get(name)
        if name.startswith("/gt/") or role == "gt":
            decoded = decode_message(typ, blob)
            rec = {"kind": "gt", "t": t, "topic": name}
            if decoded:
                rec.update(decoded)
                if "data" in decoded and "s" not in rec:
                    rec["s"] = decoded["data"]
            rows.append(rec)
            continue
        if role == "est" or name == CANONICAL["estimate"]:
            decoded = decode_message(typ, blob) or {}
            rec = {
                "kind": "est",
                "t": t,
                "topic": name,
                "s": decoded.get("s"),
                "v": decoded.get("v"),
                "p_ss": decoded.get("p_ss"),
                "p_vv": decoded.get("p_vv"),
            }
            # Odometry has no OK/DEGRADED/LOST. Do not invent OK.
            pss = decoded.get("p_ss")
            try:
                if pss is not None and float(pss) >= 5.0e5:
                    rec["confidence"] = "UNINITIALIZED"
            except (TypeError, ValueError):
                pass
            rows.append(rec)
            continue
        if role == "diag" or name == DIAGNOSTICS_TOPIC or (
            typ.replace("/msg/", "/") == "diagnostic_msgs/DiagnosticArray"
        ):
            decoded = decode_message(typ, blob) or {}
            rec = {
                "kind": "diag",
                "t": t,
                "topic": name,
            }
            if decoded.get("confidence"):
                rec["confidence"] = str(decoded["confidence"]).upper()
            rows.append(rec)
            continue
        decoded = decode_message(typ, blob)
        if decoded is None:
            skipped[typ] = skipped.get(typ, 0) + 1
            continue
        if role == "notch":
            raw = float(decoded["data"])
            if not math.isfinite(raw):
                continue
            last["notch"] = map_notch(raw, notch_max_abs)
            rows.append({"kind": "input", "t": t, "notch": last["notch"], "topic": name})
        elif role == "brake":
            raw = float(decoded["data"])
            if not math.isfinite(raw):
                continue
            last["brake"] = max(0.0, min(1.0, raw))
            rows.append({"kind": "input", "t": t, "brake": last["brake"], "topic": name})
        elif role == "wheels":
            raw = decoded.get("data") or decoded.get("velocity")
            if decoded.get("twist_vx") is not None and (
                not raw or (isinstance(raw, list) and len(raw) <= 1)
            ):
                vx = float(decoded["twist_vx"])
                w = vx if twist_is_omega else vx / max(wheel_radius_m, 1e-6)
                raw = [w]
            omega = pad_wheels(list(raw or []), n_wheels)
            rec = {
                "kind": "input",
                "t": t,
                "notch": last["notch"],
                "brake": last["brake"],
                "topic": name,
            }
            for i, w in enumerate(omega[:n_wheels]):
                rec[f"w{i}"] = w
            rows.append(rec)
    _attach_diag_confidence(rows)
    notes = [f"skipped_type {k} n={v}" for k, v in sorted(skipped.items())]
    return rows, notes


def rows_to_filter_csv(rows: list[dict], dest: Path) -> int:
    n = 0
    dest.parent.mkdir(parents=True, exist_ok=True)
    n_w = 4
    for rec in rows:
        keys = [k for k in rec if k.startswith("w") and k[1:].isdigit()]
        if keys:
            n_w = max(n_w, max(int(k[1:]) for k in keys) + 1)
    n_w = min(n_w, 6)
    wkeys = [f"w{i}" for i in range(n_w)]
    with dest.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "notch", "brake", *wkeys])
        for rec in rows:
            if rec.get("kind") != "input":
                continue
            if "w0" not in rec:
                continue
            w.writerow(
                [
                    rec["t"],
                    rec.get("notch", math.nan),
                    rec.get("brake", math.nan),
                    *[rec.get(k, math.nan) for k in wkeys],
                ]
            )
            n += 1
    return n


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--csv", type=Path, default=None, help="filter.csv for replay_ukf")
    ap.add_argument(
        "--topics-yaml",
        type=Path,
        default=DEFAULT_YAML,
    )
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    try:
        aliases, notch_max, n_w, r0, twist_omega = _load_adapter(args.topics_yaml)
        rows, notes = bag_to_rows(
            args.bag,
            aliases,
            notch_max_abs=notch_max,
            n_wheels=n_w,
            wheel_radius_m=r0,
            twist_is_omega=twist_omega,
        )
    except FileNotFoundError as e:
        sys.stderr.write(f"{e}\n")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8", newline="\n") as f:
        for rec in rows:
            f.write(json.dumps(rec) + "\n")
    print(f"jsonl={args.out} records={len(rows)}")
    for n in notes:
        print(n)
    if args.csv is not None:
        n = rows_to_filter_csv(rows, args.csv)
        print(f"filter_csv={args.csv} rows={n}")
        if n == 0:
            sys.stderr.write("no wheel rows — cannot replay UKF on this bag yet\n")
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

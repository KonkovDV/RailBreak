"""Independent envelope checker. Must not import UKF / plant identification.

Reads JSONL records or a rosbag2 directory (`--bag` / directory positional).
Exit 0 empty; 2 dirty; 1 usage.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

ALLOWED_CONF = {"OK", "DEGRADED", "LOST", "UNINITIALIZED"}


@dataclass
class CheckResult:
    hits: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    has_gt: bool = False
    n_ok: int = 0
    n_hmi: int = 0
    missed_path_m: float | None = None


def _num(row: dict, *keys: str) -> float | None:
    for k in keys:
        if k in row and row[k] is not None:
            try:
                return float(row[k])
            except (TypeError, ValueError):
                return None
    return None


def check_rows(rows: list[dict], *, require_gt: bool) -> CheckResult:
    out = CheckResult()
    gt_s: list[tuple[int, float]] = []
    est: list[dict] = []
    for i, row in enumerate(rows):
        topic = str(row.get("topic", ""))
        if topic.startswith("/gt/") or row.get("kind") == "gt":
            s = _num(row, "s", "s_m")
            if s is not None:
                gt_s.append((i, s))
            continue
        if topic in {"/tram/state_estimate", "/tram/odometry"} or row.get("kind") == "est":
            est.append(row)
            cov = _num(row, "p_ss", "pose_cov_0")
            pvv = _num(row, "p_vv", "twist_cov_0")
            if cov is None or pvv is None or not math.isfinite(cov) or not math.isfinite(pvv):
                out.hits.append(f"NO_COVARIANCE at record {i}")
            elif cov == 0.0 and pvv == 0.0:
                out.hits.append(f"NO_COVARIANCE at record {i}")
            conf = str(row.get("confidence", row.get("status", ""))).upper()
            if conf not in ALLOWED_CONF:
                out.hits.append(f"NO_CONFIDENCE at record {i}")
            if conf == "UNINITIALIZED":
                out.hits.append(f"UNINITIALIZED at record {i}")
            s_hat = _num(row, "s", "s_m")
            s_gt = gt_s[-1][1] if gt_s else None
            # Envelope vs GT whenever GT is present. --require-gt only means
            # "fail if there is no GT", not "skip the line when GT exists".
            # ENVELOPE_GT at OK is a hazardously-misleading (HMI) event.
            if gt_s and conf == "OK":
                out.n_ok += 1
                if s_hat is not None:
                    ds = abs(s_hat - s_gt)
                    # 5 m + 5% of travelled GT (calibration line, not a certificate).
                    limit = 5.0 + 0.05 * abs(s_gt)
                    if ds > limit:
                        out.n_hmi += 1
                        out.hits.append(f"ENVELOPE_GT at record {i}: |s|={ds:.2f} > {limit:.2f}")
            if (
                gt_s
                and conf == "DEGRADED"
                and out.missed_path_m is None
                and s_hat is not None
                and s_gt is not None
            ):
                out.missed_path_m = abs(s_hat - s_gt)
    out.has_gt = bool(gt_s)
    if require_gt and not gt_s:
        out.hits = ["usage: ENVELOPE_GT requested but no /gt records"]
        return out
    if not out.has_gt:
        out.notes.append("ENVELOPE_GT skipped: no GT")
    else:
        rate = (out.n_hmi / out.n_ok) if out.n_ok else 0.0
        out.notes.append(f"HMI-rate={rate:.6f} ({out.n_hmi}/{out.n_ok})")
        if out.missed_path_m is None:
            out.notes.append("missed_path_until_degraded=n/a")
        else:
            out.notes.append(f"missed_path_until_degraded={out.missed_path_m:.3f}")
    if not est:
        out.hits.append("NO_ESTIMATE")
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="tramDR envelope / HMI checker (no UKF)")
    p.add_argument("jsonl", nargs="?", help="JSONL file or rosbag2 directory")
    p.add_argument("--require-gt", action="store_true")
    args = p.parse_args(argv)
    if not args.jsonl:
        sys.stderr.write("usage: check_envelope.py <file.jsonl|bagdir> [--require-gt]\n")
        return 1
    path = Path(args.jsonl)
    rows: list[dict] = []
    if path.is_dir():
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from bag_to_jsonl import _load_adapter, bag_to_rows

        try:
            aliases, notch_max, n_w, r0, twist_omega = _load_adapter(None)
            rows, notes = bag_to_rows(
                path,
                aliases,
                notch_max_abs=notch_max,
                n_wheels=n_w,
                wheel_radius_m=r0,
                twist_is_omega=twist_omega,
            )
        except FileNotFoundError as e:
            sys.stderr.write(f"{e}\n")
            return 1
        for n in notes:
            sys.stderr.write(n + "\n")
    elif path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    else:
        sys.stderr.write(f"missing {path}\n")
        return 1
    result = check_rows(rows, require_gt=args.require_gt)
    if any(h.startswith("usage:") for h in result.hits):
        sys.stderr.write(result.hits[0] + "\n")
        return 1
    for n in result.notes:
        sys.stdout.write(n + "\n")
    if result.hits:
        sys.stderr.write("checker dirty:\n")
        for h in result.hits:
            sys.stderr.write(f"  {h}\n")
        return 2
    sys.stdout.write("checker empty\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

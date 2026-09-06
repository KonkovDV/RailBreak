"""25.09 path: inspect rosbag2 → JSONL → optional UKF replay → envelope.

Does not import UKF. Organiser bag is not in this repository.
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from bag_to_jsonl import _load_adapter, bag_to_rows, rows_to_filter_csv  # noqa: E402
from check_envelope import check_rows  # noqa: E402
from inspect_bag import main as inspect_main  # noqa: E402


def _merge(gt_rows: list[dict], est_rows: list[dict], dest: Path) -> None:
    recs = [(float(r.get("t") or 0.0), 0, r) for r in gt_rows]
    recs += [(float(r.get("t") or 0.0), 1, r) for r in est_rows]
    recs.sort(key=lambda x: (x[0], x[1]))
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open("w", encoding="utf-8", newline="\n") as f:
        for _, _, r in recs:
            f.write(json.dumps(r) + "\n")


def _lerp(tq: list[float], ts: list[float], ys: list[float]) -> list[float]:
    if not ts or not ys or len(ts) != len(ys):
        return [float("nan")] * len(tq)
    out: list[float] = []
    j = 0
    n = len(ts)
    for t in tq:
        while j + 1 < n and ts[j + 1] < t:
            j += 1
        if t <= ts[0]:
            out.append(ys[0])
        elif t >= ts[-1]:
            out.append(ys[-1])
        else:
            t0, t1 = ts[j], ts[j + 1]
            y0, y1 = ys[j], ys[j + 1]
            a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            out.append(y0 + a * (y1 - y0))
    return out


def _median_dt(times: list[float]) -> float:
    dts = [times[i] - times[i - 1] for i in range(1, len(times)) if times[i] > times[i - 1]]
    if not dts:
        return 0.02
    dts.sort()
    return dts[len(dts) // 2]


def print_baselines(filter_csv: Path, gt_rows: list[dict], est_rows: list[dict]) -> None:
    """UKF vs 3 baselines on THIS bag. Does not substitute seed-42 synth numbers."""
    sys.path.insert(0, str(ROOT / "tools" / "synth"))
    from score import _fmt, score_series  # noqa: WPS433

    with filter_csv.open(encoding="utf-8") as f:
        raw = list(csv.DictReader(f))
    if not raw:
        print("empty filter.csv — no baseline table")
        return
    times = [float(r["t_s"]) for r in raw]
    notch = [float(r["notch"]) for r in raw]
    brake = [float(r["brake"]) for r in raw]
    keys = sorted(
        (k for k in raw[0] if k.startswith("w") and k[1:].isdigit()),
        key=lambda k: int(k[1:]),
    )
    omega_rows = [[float(r[k]) for k in keys] for r in raw]
    gt_pairs = [(float(r["t"]), float(r["s"])) for r in gt_rows if r.get("s") is not None]
    if len(gt_pairs) < 2:
        print(
            "NO_GT: UKF vs 3 baseline table cannot be built on this bag. "
            "Not substituting docs/metrics.md seed-42 numbers."
        )
        return
    gt_t = [p[0] for p in gt_pairs]
    gt_s = [p[1] for p in gt_pairs]
    s_gt = _lerp(times, gt_t, gt_s)
    gt_v_pairs = [
        (float(r["t"]), float(r["v"])) for r in gt_rows if r.get("v") is not None
    ]
    if len(gt_v_pairs) >= 2:
        v_gt = _lerp(times, [p[0] for p in gt_v_pairs], [p[1] for p in gt_v_pairs])
    else:
        v_gt = [0.0] * len(s_gt)
        for i in range(1, len(s_gt)):
            dt = max(times[i] - times[i - 1], 1e-6)
            v_gt[i] = (s_gt[i] - s_gt[i - 1]) / dt
        if len(v_gt) > 1:
            v_gt[0] = v_gt[1]
    ukf_s = ukf_v = None
    est_sv = [
        (float(r.get("t") or 0.0), float(r["s"]), float(r["v"]))
        for r in est_rows
        if r.get("s") is not None and r.get("v") is not None
    ]
    if len(est_sv) >= 2:
        ukf_s = _lerp(times, [p[0] for p in est_sv], [p[1] for p in est_sv])
        ukf_v = _lerp(times, [p[0] for p in est_sv], [p[2] for p in est_sv])
    dt = _median_dt(times)
    table = score_series(notch, brake, omega_rows, s_gt, v_gt, dt, ukf_s, ukf_v)
    print(f"UKF vs 3 baselines on this bag  dt_med={dt:.4f} s  n={len(times)}")
    print("  [this recording, not seed 42]")
    for key in ("naive", "plant", "complementary", "ukf"):
        if key in table:
            print(f"  {key:14s} {_fmt(table[key])}")
    (filter_csv.parent / "baselines.json").write_text(
        json.dumps(table, indent=2), encoding="utf-8"
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--ukf", type=Path, default=None, help="replay_ukf binary")
    ap.add_argument(
        "--vehicle",
        type=Path,
        default=ROOT / "tram_dr_localization" / "config" / "vehicle_lvenok_moscow.yaml",
        help="vehicle YAML for replay_ukf; n=4 Львёнок default. Use vityaz_m for 6 axles.",
    )
    ap.add_argument("--work", type=Path, default=None)
    ap.add_argument("--require-gt", action="store_true")
    ap.add_argument(
        "--baselines",
        action="store_true",
        help="print UKF vs 3 baselines if GT is in the bag; refuse seed-42 fallback",
    )
    ap.add_argument(
        "--topics-yaml",
        type=Path,
        default=ROOT / "tram_dr_localization" / "config" / "customer_topics.yaml",
    )
    ap.add_argument(
        "--route",
        type=Path,
        default=ROOT / "tram_dr_localization" / "config" / "route_10.yaml",
        help="stop vertices for mass_door gate; empty file = ungated (synth legacy)",
    )
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    inspect_rc = inspect_main([str(args.bag)])
    if inspect_rc == 1:
        return 1
    work = args.work or (args.bag / "_tramdr_eval")
    work.mkdir(parents=True, exist_ok=True)
    aliases, notch_max, n_w, r0, twist_omega = _load_adapter(args.topics_yaml)
    rows, notes = bag_to_rows(
        args.bag,
        aliases,
        notch_max_abs=notch_max,
        n_wheels=n_w,
        wheel_radius_m=r0,
        twist_is_omega=twist_omega,
    )
    (work / "bag.jsonl").write_text(
        "".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8"
    )
    for n in notes:
        print(n)
    n_csv = rows_to_filter_csv(rows, work / "filter.csv")
    print(f"wheel_rows={n_csv}")
    if inspect_rc == 2 and n_csv == 0:
        sys.stderr.write("canonical /tram/* missing and no aliased wheel rows — fill customer_topics.yaml\n")
        return 2
    est_rows = [r for r in rows if r.get("kind") == "est"]
    if args.ukf is not None:
        if n_csv == 0:
            sys.stderr.write("no wheel rows — cannot replay UKF\n")
            return 2
        ukf_out = work / "ukf.jsonl"
        ukf_cmd = [str(args.ukf), str(work / "filter.csv"), str(ukf_out)]
        if args.vehicle is not None and args.vehicle.is_file():
            ukf_cmd.extend(["--vehicle", str(args.vehicle)])
        if args.route is not None:
            if args.route.is_file() and args.route.stat().st_size > 0:
                ukf_cmd.extend(["--route", str(args.route)])
            elif args.route.is_file():
                print("empty --route: mass_door ungated")
            else:
                sys.stderr.write(f"cannot read --route {args.route}\n")
                return 2
        subprocess.run(ukf_cmd, check=True)
        est_rows = [
            json.loads(line)
            for line in ukf_out.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    gt_rows = [r for r in rows if r.get("kind") == "gt"]
    if args.baselines:
        print_baselines(work / "filter.csv", gt_rows, est_rows)
    if not est_rows:
        print("no estimates (inspect-only until --ukf or recorded /tram/state_estimate)")
        return 0
    merged = work / "merged.jsonl"
    _merge(gt_rows, est_rows, merged)
    stream = [
        json.loads(line)
        for line in merged.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    hits = check_rows(stream, require_gt=args.require_gt)
    for n in hits.notes:
        print(n)
    if hits.hits:
        sys.stderr.write("checker dirty:\n")
        for h in hits.hits:
            sys.stderr.write(f"  {h}\n")
        return 2
    print("checker empty")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

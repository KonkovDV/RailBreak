"""M8 bench: methods, relative drift, sensor-loss, block bootstrap.

Does not import the UKF. Optional replay is a subprocess of replay_ukf.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))
sys.path.insert(0, str(ROOT / "tools" / "synth"))

from baselines import complementary, naive_wheel, plant_only  # noqa: E402
from plant_ref import PlantParams  # noqa: E402
from score import envelope_s, mean_nees, metrics  # noqa: E402

DT = 0.02
R0 = 0.35
LENGTHS = (100.0, 200.0, 400.0, 800.0, 1600.0)
HORIZONS = (2.0, 5.0, 10.0, 20.0, 30.0)


def _load_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _load_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def relative_drift(s_hat: list[float], s_gt: list[float], t: list[float],
                   lengths: tuple[float, ...] = LENGTHS) -> dict[str, float]:
    out: dict[str, float] = {}
    n = min(len(s_hat), len(s_gt), len(t))
    if n < 2:
        return {f"eps_{int(L)}": float("nan") for L in lengths}
    for L in lengths:
        acc = []
        j = 0
        for i in range(n):
            target = s_gt[i] + L
            while j < n and s_gt[j] < target:
                j += 1
            if j >= n:
                break
            ds_hat = s_hat[j] - s_hat[i]
            ds_gt = s_gt[j] - s_gt[i]
            if ds_gt > 1e-6:
                acc.append(abs(ds_hat - ds_gt) / ds_gt)
        out[f"eps_{int(L)}"] = statistics.median(acc) if acc else float("nan")
    return out


def open_loop_error(s_hat: list[float], v_hat: list[float], s_gt: list[float],
                    v_gt: list[float], t: list[float]) -> dict[str, float]:
    out: dict[str, float] = {}
    n = min(len(s_hat), len(s_gt), len(v_hat), len(v_gt), len(t))
    for H in HORIZONS:
        es = []
        ev = []
        j = 0
        for i in range(n):
            target = t[i] + H
            while j < n and t[j] < target:
                j += 1
            if j >= n:
                break
            es.append(abs(s_hat[j] - s_gt[j]))
            ev.append(abs(v_hat[j] - v_gt[j]))
        out[f"e_s_{int(H)}s"] = statistics.median(es) if es else float("nan")
        out[f"e_v_{int(H)}s"] = statistics.median(ev) if ev else float("nan")
    return out


def block_bootstrap_rmse(err: list[float], block: int = 1500, n_boot: int = 200,
                         seed: int = 42) -> dict[str, float]:
    import random
    rng = random.Random(seed)
    finite = [e for e in err if math.isfinite(e)]
    if len(finite) < 20:
        rmse = math.sqrt(sum(e * e for e in finite) / len(finite)) if finite else float("nan")
        return {"rmse": rmse, "p05": float("nan"), "p95": float("nan")}
    # Plan M8: 30 s at 50 Hz is 1500. Short clips use a 0.5 s floor so the
    # interval is defined rather than NaN.
    blk = min(block, max(25, len(finite) // 4))
    if len(finite) < blk:
        blk = max(len(finite) // 2, 10)
    blocks = [finite[i:i + blk] for i in range(0, len(finite) - blk + 1, blk)]
    if not blocks:
        blocks = [finite]
    dist = []
    for _ in range(n_boot):
        sample = []
        while len(sample) < len(finite):
            sample.extend(rng.choice(blocks))
        sample = sample[:len(finite)]
        dist.append(math.sqrt(sum(e * e for e in sample) / len(sample)))
    dist.sort()
    rmse = math.sqrt(sum(e * e for e in finite) / len(finite))
    return {"rmse": rmse, "p05": dist[int(0.05 * (len(dist) - 1))],
            "p95": dist[int(0.95 * (len(dist) - 1))], "block": blk}


def score_method(name: str, s: list[float], v: list[float], s_gt: list[float],
                 v_gt: list[float], t: list[float], p_ss: list[float] | None = None) -> dict:
    m = metrics(s, v, s_gt, v_gt)
    m["method"] = name
    m.update(relative_drift(s, s_gt, t))
    m.update(open_loop_error(s, v, s_gt, v_gt, t))
    err = [s[i] - s_gt[i] for i in range(min(len(s), len(s_gt)))]
    m["bootstrap_s"] = block_bootstrap_rmse(err)
    if p_ss is not None:
        m["nees_s"] = mean_nees(err, p_ss[:len(err)])
    return m


def _series(rows: list[dict], key: str) -> list[float]:
    out = []
    for r in rows:
        raw = r.get(key, "")
        try:
            out.append(float(raw))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return out


def methods_from_csv(rows: list[dict], dt: float = DT) -> dict[str, tuple[list[float], list[float]]]:
    notch = _series(rows, "notch")
    brake = []
    for r in rows:
        raw = r.get("brake", "")
        try:
            brake.append(0.0 if raw in ("", None) else float(raw))
        except (TypeError, ValueError):
            brake.append(0.0)
    omega_mean = []
    omega_rows = []
    for r in rows:
        w = []
        for k in ("w0", "w1", "w2", "w3", "w4", "w5"):
            if k in r and r[k] not in (None, ""):
                try:
                    val = float(r[k])
                except (TypeError, ValueError):
                    continue
                if math.isfinite(val):
                    w.append(val)
        omega_rows.append(w)
        omega_mean.append(sum(w) / len(w) if w else 0.0)
    naive = naive_wheel(omega_mean, dt, r0_m=R0)
    plant = plant_only(notch, brake, dt, PlantParams())
    comp = complementary(notch, omega_rows, dt, brake=brake, r0_m=R0)
    return {
        "naive": ([x.s_m for x in naive], [x.v_mps for x in naive]),
        "plant": ([x.s_m for x in plant], [x.v_mps for x in plant]),
        "complementary": ([x.s_m for x in comp], [x.v_mps for x in comp]),
    }


def odometry_track(bagdir: Path) -> list[tuple[float, float, float, float]]:
    """(t, s, v, p_ss) from /tram/state_estimate. One sample per header stamp.

    A frozen /clock republishes the same stamp. Those copies are not extra trials.
    """
    from rosbag2_io import decode_message, iter_messages

    seen: set[int] = set()
    out: list[tuple[float, float, float, float]] = []
    for _ts, name, typ, blob in iter_messages(bagdir):
        if not str(name).endswith("state_estimate"):
            continue
        msg = decode_message(typ, blob)
        if not msg or "s" not in msg or msg.get("v") is None:
            continue
        t = float(msg.get("stamp_s") or 0.0)
        if not math.isfinite(t):
            continue
        key = int(round(t * 1e6))
        if key in seen:
            continue
        seen.add(key)
        p = msg.get("p_ss")
        try:
            p_ss = float(p) if p is not None else float("nan")
        except (TypeError, ValueError):
            p_ss = float("nan")
        out.append((t, float(msg["s"]), float(msg["v"]), p_ss))
    out.sort(key=lambda row: row[0])
    return out


def score_ros_odometry(bagdir: Path, gt_rows: list[dict], tol_s: float = 0.03) -> dict:
    """Same score_method as the CSV bench, on a recorded Odometry topic.

    Not a seed-42 table and not an organiser result.
    """
    track = odometry_track(bagdir)
    gt: list[tuple[float, float, float]] = []
    for i, row in enumerate(gt_rows):
        try:
            tg = float(row.get("t_s", row.get("t", i * DT)))
            sg = float(row["gt_s"])
            vg = float(row["gt_v"])
        except (TypeError, ValueError, KeyError):
            continue
        if math.isfinite(tg) and math.isfinite(sg) and math.isfinite(vg):
            gt.append((tg, sg, vg))
    s: list[float] = []
    v: list[float] = []
    s_gt: list[float] = []
    v_gt: list[float] = []
    t: list[float] = []
    p: list[float] = []
    j = 0
    for ts, ss, vv, pp in track:
        while j + 1 < len(gt) and abs(gt[j + 1][0] - ts) <= abs(gt[j][0] - ts):
            j += 1
        if not gt or abs(gt[j][0] - ts) > tol_s:
            continue
        t.append(ts)
        s.append(ss)
        v.append(vv)
        p.append(pp)
        s_gt.append(gt[j][1])
        v_gt.append(gt[j][2])
    if not s:
        return {
            "method": "ros_ukf",
            "n_track": len(track),
            "n_paired": 0,
            "note": "no header stamp within tol of gt",
        }
    scored = score_method("ros_ukf", s, v, s_gt, v_gt, t, p)
    scored["n_track"] = len(track)
    scored["n_paired"] = len(s)
    scored["t_end_s"] = t[-1]
    scored["note"] = (
        "ROS /tram/state_estimate by header stamp; frozen-stamp copies dropped. "
        "Not seed-42 and not the organiser bag."
    )
    return scored


def maybe_replay(ukf: Path | None, csv_path: Path, out_jsonl: Path,
                 extra: list[str] | None = None) -> list[dict]:
    if ukf is None:
        return []
    cmd = [str(ukf), str(csv_path), str(out_jsonl)]
    if extra:
        cmd.extend(extra)
    subprocess.run(cmd, check=True)
    return _load_jsonl(out_jsonl)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path, nargs="?")
    ap.add_argument("--ukf", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--odom-bag", type=Path)
    ap.add_argument("--gt-csv", type=Path)
    args = ap.parse_args(argv)
    if args.odom_bag is not None:
        if args.gt_csv is None:
            raise SystemExit("--odom-bag needs --gt-csv")
        with args.gt_csv.open(encoding="utf-8", newline="") as f:
            gt_rows = list(csv.DictReader(f))
        scored = score_ros_odometry(args.odom_bag, gt_rows)
        report = {
            "run": str(args.odom_bag),
            "gt": str(args.gt_csv),
            "methods": [scored],
        }
        text = json.dumps(report, indent=2)
        if args.out is not None:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(text, encoding="utf-8")
        else:
            print(text)
        return 0
    if args.run_dir is None:
        raise SystemExit("run_dir is required unless --odom-bag is set")
    run = args.run_dir
    rows = _load_csv(run / "filter.csv")
    gt = _load_jsonl(run / "gt.jsonl") if (run / "gt.jsonl").is_file() else []
    s_gt = [float(r.get("s", r.get("gt_s", "nan"))) for r in gt] or _series(rows, "gt_s")
    v_gt = [float(r.get("v", r.get("gt_v", "nan"))) for r in gt] or _series(rows, "gt_v")
    t = _series(rows, "t_s") if rows and "t_s" in rows[0] else _series(rows, "t")
    if not t:
        t = [i * DT for i in range(len(rows))]
    table = []
    for name, (s, v) in methods_from_csv(rows).items():
        table.append(score_method(name, s, v, s_gt, v_gt, t))
    extra = []
    for item in args.set:
        extra.extend(["--set", item])
    if args.ukf is not None:
        est = maybe_replay(args.ukf, run / "filter.csv", run / "ukf_bench.jsonl", extra)
        s = [float(r["s"]) for r in est]
        v = [float(r["v"]) for r in est]
        p = [float(r.get("p_ss", "nan")) for r in est]
        table.append(score_method("ukf", s, v, s_gt, v_gt, t, p))
    report = {"run": str(run), "methods": table}
    text = json.dumps(report, indent=2)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        md = ["# bench", "", f"run: `{run}`", "", "| method | rmse_s | rmse_v | env_s |",
              "| --- | ---: | ---: | ---: |"]
        for m in table:
            md.append(f"| {m['method']} | {m['rmse_s']:.3f} | {m['rmse_v']:.3f} | {m['env_s']:.3f} |")
        args.out.with_suffix(".md").write_text("\n".join(md) + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

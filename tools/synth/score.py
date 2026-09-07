"""Score filter CSV/JSONL vs GT and three baselines. Does not import UKF."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from baselines import complementary, naive_wheel, plant_only  # noqa: E402
from plant_ref import PlantParams  # noqa: E402

DT = 0.02
R0 = 0.35
V_LIM = 2.0 / 3.6  # ±2 km/h below 30 km/h (calibration line)


def _load_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _rmse(a: list[float], b: list[float]) -> float:
    n = min(len(a), len(b))
    if n == 0:
        return float("nan")
    acc = 0.0
    k = 0
    for i in range(n):
        if not math.isfinite(a[i]) or not math.isfinite(b[i]):
            continue
        acc += (a[i] - b[i]) ** 2
        k += 1
    return math.sqrt(acc / k) if k else float("nan")


def mean_nees(err: list[float], var: list[float]) -> float:
    """1D NEES = e²/P. Pairs must already be time-aligned."""
    n = min(len(err), len(var))
    acc = 0.0
    k = 0
    for i in range(n):
        p = var[i]
        if p <= 1e-12 or not math.isfinite(err[i]) or not math.isfinite(p):
            continue
        acc += (err[i] * err[i]) / p
        k += 1
    return acc / k if k else float("nan")


def envelope_s(s_hat: float, s_gt: float) -> bool:
    return abs(s_hat - s_gt) <= 5.0 + 0.05 * abs(s_gt)


def metrics(s_hat: list[float], v_hat: list[float], s_gt: list[float],
            v_gt: list[float]) -> dict:
    n = min(len(s_hat), len(s_gt), len(v_hat), len(v_gt))
    n_gt = min(len(s_gt), len(v_gt))
    ok = sum(1 for i in range(n) if envelope_s(s_hat[i], s_gt[i]))
    vok = sum(1 for i in range(n) if abs(v_hat[i] - v_gt[i]) <= V_LIM)
    return {
        "n": n,
        "n_gt": n_gt,
        "n_hat": min(len(s_hat), len(v_hat)),
        "coverage": (n / n_gt) if n_gt else 0.0,
        "rmse_s": _rmse(s_hat[:n], s_gt[:n]),
        "rmse_v": _rmse(v_hat[:n], v_gt[:n]),
        "env_s": ok / n if n else 0.0,
        "env_v": vok / n if n else 0.0,
        "final_ds": abs(s_hat[n - 1] - s_gt[n - 1]) if n else float("nan"),
    }


def score_series(
    notch: list[float],
    brake: list[float],
    omega_rows: list[list[float]],
    s_gt: list[float],
    v_gt: list[float],
    dt_s: float,
    ukf_s: list[float] | None = None,
    ukf_v: list[float] | None = None,
) -> dict:
    """Same three baselines as score_run, from arrays (bag or synth)."""
    dt = dt_s if dt_s > 0.0 else DT
    omega_mean = [(sum(row) / len(row) if row else 0.0) for row in omega_rows]
    naive = naive_wheel(omega_mean, dt)
    plant = plant_only(notch, brake, dt, PlantParams())
    comp = complementary(notch, omega_rows, dt, brake=brake)
    out: dict = {
        "naive": metrics([x.s_m for x in naive], [x.v_mps for x in naive], s_gt, v_gt),
        "plant": metrics([x.s_m for x in plant], [x.v_mps for x in plant], s_gt, v_gt),
        "complementary": metrics(
            [x.s_m for x in comp], [x.v_mps for x in comp], s_gt, v_gt
        ),
    }
    if ukf_s is not None and ukf_v is not None:
        out["ukf"] = metrics(ukf_s, ukf_v, s_gt, v_gt)
    return out


def _pair_ukf_gt(
    t_e: list[float],
    s_e: list[float],
    v_e: list[float],
    p_e: list[float],
    t_gt: list[float],
    s_gt: list[float],
    v_gt: list[float],
    tol: float = 0.011,
) -> tuple[list[float], list[float], list[float], list[float], list[float]]:
    """One time join of (s, v, P) against GT. Unmatched rows are dropped, not truncated."""
    js: list[float] = []
    jv: list[float] = []
    jp: list[float] = []
    gs: list[float] = []
    gv: list[float] = []
    j = 0
    n_b = len(t_gt)
    n_e = min(len(t_e), len(s_e), len(v_e), len(p_e))
    for i in range(n_e):
        t = t_e[i]
        if not math.isfinite(t):
            continue
        while j + 1 < n_b and abs(t_gt[j + 1] - t) < abs(t_gt[j] - t):
            j += 1
        if n_b == 0 or abs(t_gt[j] - t) > tol:
            continue
        if j >= len(s_gt) or j >= len(v_gt):
            continue
        if not math.isfinite(s_e[i]) or not math.isfinite(v_e[i]):
            continue
        if not math.isfinite(s_gt[j]) or not math.isfinite(v_gt[j]):
            continue
        js.append(s_e[i])
        jv.append(v_e[i])
        jp.append(p_e[i])
        gs.append(s_gt[j])
        gv.append(v_gt[j])
    return js, jv, jp, gs, gv


def score_run(run_dir: Path) -> dict:
    raw = _load_csv(run_dir / "run.csv")
    s_gt = [float(r["gt_s"]) for r in raw]
    v_gt = [float(r["gt_v"]) for r in raw]
    t_gt = [float(r["t_s"]) for r in raw]
    notch = [float(r["notch"]) for r in raw]
    brake = [float(r["brake"]) for r in raw]
    keys = sorted(
        (k for k in raw[0] if k.startswith("w") and k[1:].isdigit()),
        key=lambda k: int(k[1:]),
    )
    omega_rows = [[float(r[k]) for k in keys] for r in raw]
    ukf_s = ukf_v = None
    ukf_pss: list[float] = []
    est_path = run_dir / "ukf.jsonl"
    if est_path.is_file():
        est = [
            json.loads(line)
            for line in est_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        t_e = [float(r["t"]) for r in est]
        s_e = [float(r["s"]) for r in est]
        v_e = [float(r["v"]) for r in est]
        p_e = [float(r["p_ss"]) if "p_ss" in r else float("nan") for r in est]
        ukf_s, ukf_v, ukf_pss, s_gt_a, v_gt_a = _pair_ukf_gt(
            t_e, s_e, v_e, p_e, t_gt, s_gt, v_gt
        )
        out = {"scenario": run_dir.name}
        out.update(score_series(notch, brake, omega_rows, s_gt, v_gt, DT, None, None))
        if ukf_s:
            out["ukf"] = metrics(ukf_s, ukf_v, s_gt_a, v_gt_a)
            out["ukf"]["nees_s"] = mean_nees(
                [ukf_s[i] - s_gt_a[i] for i in range(len(ukf_s))],
                ukf_pss,
            )
        return out
    out = {"scenario": run_dir.name}
    out.update(score_series(notch, brake, omega_rows, s_gt, v_gt, DT, None, None))
    return out


def _fmt(block: dict) -> str:
    line = (
        f"rmse_s={block['rmse_s']:.2f} m  rmse_v={block['rmse_v']:.3f} m/s  "
        f"env_s={100 * block['env_s']:.1f}%  env_v={100 * block['env_v']:.1f}%  "
        f"|ds|_end={block['final_ds']:.2f} m"
    )
    if "coverage" in block and math.isfinite(block["coverage"]) and block["coverage"] < 0.999:
        line += f"  coverage={100 * block['coverage']:.1f}%"
    if "nees_s" in block and math.isfinite(block["nees_s"]):
        line += f"  nees_s={block['nees_s']:.3g}"
    return line


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=Path, default=ROOT / "synth" / "runs")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if not args.runs.is_dir():
        sys.stderr.write(f"missing {args.runs}; run tools/synth/generate.py first\n")
        return 1
    print("RailBreak synth score (seed 42)")
    for child in sorted(p for p in args.runs.iterdir() if p.is_dir()):
        if not (child / "run.csv").is_file():
            continue
        m = score_run(child)
        print(f"\n## {m['scenario']}")
        for key in ("naive", "plant", "complementary", "ukf"):
            if key in m:
                print(f"  {key:14s} {_fmt(m[key])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

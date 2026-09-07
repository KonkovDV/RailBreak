"""Score filter CSV/JSONL vs GT and three baselines. Does not import UKF."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from bisect import bisect_left
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
        "coverage": (n / n_gt) if n_gt else float("nan"),
        "rmse_s": _rmse(s_hat[:n], s_gt[:n]),
        "rmse_v": _rmse(v_hat[:n], v_gt[:n]),
        "env_s": ok / n if n else float("nan"),
        "env_v": vok / n if n else float("nan"),
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


def _pair_ukf_gt_with_indices(
    t_e: list[float],
    s_e: list[float],
    v_e: list[float],
    p_e: list[float],
    t_gt: list[float],
    s_gt: list[float],
    v_gt: list[float],
    tol: float = 0.011,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[int]]:
    """Nearest finite GT, retaining original GT indices for coverage.

    Inputs may be unordered. Equal-distance ties prefer the earlier timestamp;
    duplicate timestamps use the first finite GT row. Reuse is allowed for
    error metrics but must not count a GT row twice toward coverage.
    """
    if not math.isfinite(tol) or tol < 0.0:
        raise ValueError("GT pairing tolerance must be finite and nonnegative")
    by_time: dict[float, int] = {}
    for j in range(min(len(t_gt), len(s_gt), len(v_gt))):
        if all(math.isfinite(x) for x in (t_gt[j], s_gt[j], v_gt[j])):
            by_time.setdefault(t_gt[j], j)
    times = sorted(by_time)
    js: list[float] = []
    jv: list[float] = []
    jp: list[float] = []
    gs: list[float] = []
    gv: list[float] = []
    indices: list[int] = []
    n_e = min(len(t_e), len(s_e), len(v_e), len(p_e))
    for i in range(n_e):
        t = t_e[i]
        if not all(math.isfinite(x) for x in (t, s_e[i], v_e[i])):
            continue
        pos = bisect_left(times, t)
        candidates = times[max(0, pos - 1):pos + 1]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda tg: (abs(tg - t), tg))
        if abs(nearest - t) > tol:
            continue
        j = by_time[nearest]
        js.append(s_e[i])
        jv.append(v_e[i])
        jp.append(p_e[i])
        gs.append(s_gt[j])
        gv.append(v_gt[j])
        indices.append(j)
    return js, jv, jp, gs, gv, indices


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
    """Compatibility wrapper returning the original five aligned arrays."""
    js, jv, jp, gs, gv, _ = _pair_ukf_gt_with_indices(
        t_e, s_e, v_e, p_e, t_gt, s_gt, v_gt, tol
    )
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
        ukf_s, ukf_v, ukf_pss, s_gt_a, v_gt_a, gt_indices = _pair_ukf_gt_with_indices(
            t_e, s_e, v_e, p_e, t_gt, s_gt, v_gt
        )
        out = {"scenario": run_dir.name}
        out.update(score_series(notch, brake, omega_rows, s_gt, v_gt, DT, None, None))
        # Keep an empty/unmatched estimator file visible. Denominators are
        # original record counts, not the lengths of the post-join arrays.
        block = metrics(ukf_s, ukf_v, s_gt_a, v_gt_a)
        n_gt_matched = len(set(gt_indices))
        block.update({
            "n_gt": len(raw),
            "n_hat": len(est),
            "n_gt_matched": n_gt_matched,
            "coverage": n_gt_matched / len(raw),
            "estimate_coverage": len(ukf_s) / len(est) if est else float("nan"),
        })
        block["nees_s"] = mean_nees(
            [ukf_s[i] - s_gt_a[i] for i in range(len(ukf_s))], ukf_pss
        )
        out["ukf"] = block
        return out
    out = {"scenario": run_dir.name}
    out.update(score_series(notch, brake, omega_rows, s_gt, v_gt, DT, None, None))
    return out


def _fmt(block: dict) -> str:
    def number(value: float, spec: str, suffix: str = "") -> str:
        return format(value, spec) + suffix if math.isfinite(value) else "N/A"

    line = (
        f"rmse_s={number(block['rmse_s'], '.2f', ' m')}  "
        f"rmse_v={number(block['rmse_v'], '.3f', ' m/s')}  "
        f"env_s={number(100 * block['env_s'], '.1f', '%')}  "
        f"env_v={number(100 * block['env_v'], '.1f', '%')}  "
        f"|ds|_end={number(block['final_ds'], '.2f', ' m')}"
    )
    if "coverage" in block:
        coverage = block["coverage"]
        if not math.isfinite(coverage) or coverage < 0.999:
            line += f"  coverage={number(100 * coverage, '.1f', '%')}"
    if "estimate_coverage" in block:
        line += f"  estimate_coverage={number(100 * block['estimate_coverage'], '.1f', '%')}"
    if "nees_s" in block:
        line += f"  nees_s={number(block['nees_s'], '.3g')}"
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

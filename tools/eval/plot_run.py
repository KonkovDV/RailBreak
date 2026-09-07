"""SVG plots of a run: s, v, confidence. No matplotlib. No UKF import.

Usage:
  python tools/eval/plot_run.py synth/runs/axle_fault
  python tools/eval/plot_run.py --runs synth/runs
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

from baselines import complementary, naive_wheel, plant_only  # noqa: E402
from plant_ref import PlantParams  # noqa: E402

DT = 0.02
R0 = 0.35


def _load_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _load_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def envelope_s(s_gt: float) -> float:
    return 5.0 + 0.05 * abs(s_gt)


def _esc(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def _poly(xs: list[float], ys: list[float], x0: float, y0: float, x1: float, y1: float,
          xmin: float, xmax: float, ymin: float, ymax: float) -> str:
    if not xs:
        return ""
    dx = max(xmax - xmin, 1e-9)
    dy = max(ymax - ymin, 1e-9)
    w = x1 - x0
    h = y1 - y0
    pts = []
    for x, y in zip(xs, ys):
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        px = x0 + (x - xmin) / dx * w
        py = y1 - (y - ymin) / dy * h
        pts.append(f"{px:.1f},{py:.1f}")
    return " ".join(pts)


def _line(pts: str, color: str, width: float = 1.5) -> str:
    if not pts:
        return ""
    return (
        f'<polyline fill="none" stroke="{color}" stroke-width="{width}" '
        f'points="{pts}"/>'
    )


CONF_COLOR = {
    "OK": "#2e7d32",
    "DEGRADED": "#ef6c00",
    "LOST": "#c62828",
    "UNINITIALIZED": "#9e9e9e",
}


def render_svg(
    name: str,
    t: list[float],
    s_gt: list[float],
    v_gt: list[float],
    s_ukf: list[float],
    v_ukf: list[float],
    conf: list[str],
    s_naive: list[float],
    s_plant: list[float],
    s_comp: list[float],
) -> str:
    n = min(len(t), len(s_gt), len(s_ukf), len(v_gt), len(v_ukf))
    t = t[:n]
    s_gt, v_gt = s_gt[:n], v_gt[:n]
    s_ukf, v_ukf = s_ukf[:n], v_ukf[:n]
    conf = (conf + ["OK"] * n)[:n]
    s_naive, s_plant, s_comp = s_naive[:n], s_plant[:n], s_comp[:n]
    if n > 400:
        step = max(1, n // 400)
        sl = slice(None, None, step)
        t, s_gt, v_gt = t[sl], s_gt[sl], v_gt[sl]
        s_ukf, v_ukf, conf = s_ukf[sl], v_ukf[sl], conf[sl]
        s_naive, s_plant, s_comp = s_naive[sl], s_plant[sl], s_comp[sl]
        n = len(t)
    env_hi = [sg + envelope_s(sg) for sg in s_gt]
    env_lo = [sg - envelope_s(sg) for sg in s_gt]

    W, H = 960, 720
    mx, my = 70, 40
    tmin, tmax = t[0], t[-1]
    series_s = env_hi + s_naive + s_plant + s_comp + s_ukf + s_gt
    series_lo = env_lo + s_naive + s_plant + s_comp + s_ukf + s_gt
    fin_s = [x for x in series_s if math.isfinite(x)]
    fin_lo = [x for x in series_lo if math.isfinite(x)]
    smax = max(fin_s) if fin_s else 1.0
    smin = min(fin_lo) if fin_lo else 0.0
    fin_v = [x for x in (v_gt + v_ukf + [0.1, 0.0]) if math.isfinite(x)]
    vmax = max(fin_v) if fin_v else 0.1
    vmin = min(fin_v) if fin_v else 0.0

    y_s0, y_s1 = my, H * 0.42
    y_v0, y_v1 = H * 0.48, H * 0.78
    y_c0, y_c1 = H * 0.82, H * 0.88
    x0, x1 = mx, W - 24

    def P(xs, ys, ya, yb, ymin, ymax):
        return _poly(xs, ys, x0, ya, x1, yb, tmin, tmax, ymin, ymax)

    def _fin(xs: list[float]) -> list[float]:
        return [x for x in xs if math.isfinite(x)]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
        f'viewBox="0 0 {W} {H}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="{mx}" y="24" font-family="Segoe UI, sans-serif" font-size="16">'
        f"{_esc(name)}  synth seed 42</text>",
        f'<text x="{mx}" y="{y_s0 - 6}" font-size="12" font-family="Segoe UI, sans-serif">'
        "s (m) — GT, UKF, naive, plant, complementary; dotted = 5 m + 5% GT</text>",
        _line(P(t, env_hi, y_s0, y_s1, smin, smax), "#bdbdbd", 1.0),
        _line(P(t, env_lo, y_s0, y_s1, smin, smax), "#bdbdbd", 1.0),
        _line(P(t, s_naive, y_s0, y_s1, smin, smax), "#90caf9", 1.2),
        _line(P(t, s_plant, y_s0, y_s1, smin, smax), "#ce93d8", 1.2),
        _line(P(t, s_comp, y_s0, y_s1, smin, smax), "#ffcc80", 1.2),
        _line(P(t, s_gt, y_s0, y_s1, smin, smax), "#424242", 2.0),
        _line(P(t, s_ukf, y_s0, y_s1, smin, smax), "#1565c0", 2.2),
        f'<text x="{mx}" y="{y_v0 - 6}" font-size="12" font-family="Segoe UI, sans-serif">'
        "v (m/s) — GT black, UKF blue</text>",
        _line(P(t, v_gt, y_v0, y_v1, vmin, vmax), "#424242", 2.0),
        _line(P(t, v_ukf, y_v0, y_v1, vmin, vmax), "#1565c0", 2.2),
        f'<text x="{mx}" y="{y_c0 - 8}" font-size="12" font-family="Segoe UI, sans-serif">'
        "confidence (OK green / DEGRADED orange / LOST red)</text>",
    ]
    dx = max(tmax - tmin, 1e-9)
    for i in range(n - 1):
        xa = x0 + (t[i] - tmin) / dx * (x1 - x0)
        xb = x0 + (t[i + 1] - tmin) / dx * (x1 - x0)
        col = CONF_COLOR.get(conf[i], "#9e9e9e")
        parts.append(
            f'<rect x="{xa:.2f}" y="{y_c0}" width="{max(xb - xa, 0.2):.2f}" '
            f'height="{y_c1 - y_c0}" fill="{col}"/>'
        )
    parts.append(
        '<text x="70" y="710" font-size="11" font-family="Segoe UI, sans-serif" fill="#555">'
        "black=GT  blue=UKF  light-blue=naive  purple=plant  orange=complementary  "
        "dotted=checker envelope. Not a KPI of route 10.</text>"
    )
    parts.append("</svg>")
    return "\n".join(parts)


def plot_dir(run_dir: Path, dest: Path | None = None) -> Path | None:
    csv_path = run_dir / "run.csv"
    if not csv_path.is_file():
        return None
    raw = _load_csv(csv_path)
    t = [float(r["t_s"]) for r in raw]
    s_gt = [float(r["gt_s"]) for r in raw]
    v_gt = [float(r["gt_v"]) for r in raw]
    notch = [float(r["notch"]) for r in raw]
    brake = [float(r["brake"]) for r in raw]
    omega = [[float(r["w0"]), float(r["w1"]), float(r["w2"]), float(r["w3"])] for r in raw]
    omega_mean = [sum(row) / 4.0 for row in omega]
    naive = naive_wheel(omega_mean, DT)
    plant = plant_only(notch, brake, DT, PlantParams())
    comp = complementary(notch, omega, DT, brake=brake)
    est = _load_jsonl(run_dir / "ukf.jsonl")
    if not est:
        s_ukf = [float("nan")] * len(t)
        v_ukf = [float("nan")] * len(t)
        conf = ["UNINITIALIZED"] * len(t)
    else:
        s_ukf = [float(r["s"]) for r in est]
        v_ukf = [float(r["v"]) for r in est]
        conf = [str(r.get("confidence", "")) for r in est]
    svg = render_svg(
        run_dir.name,
        t,
        s_gt,
        v_gt,
        s_ukf,
        v_ukf,
        conf,
        [x.s_m for x in naive],
        [x.s_m for x in plant],
        [x.s_m for x in comp],
    )
    out = dest or (run_dir / "plot.svg")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(svg, encoding="utf-8")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="?", type=Path, help="one synth/runs/<name> directory")
    ap.add_argument("--runs", type=Path, default=None)
    ap.add_argument("--out-dir", type=Path, default=None, help="copy SVGs here")
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    dirs: list[Path] = []
    if args.run:
        dirs.append(args.run)
    if args.runs:
        dirs.extend(sorted(p for p in args.runs.iterdir() if p.is_dir()))
    if not dirs:
        ap.print_help()
        return 1
    n = 0
    for d in dirs:
        out = plot_dir(d)
        if out is None:
            continue
        if args.out_dir:
            dest = args.out_dir / f"{d.name}.svg"
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_text(out.read_text(encoding="utf-8"), encoding="utf-8")
            print(dest)
        else:
            print(out)
        n += 1
    return 0 if n else 1


if __name__ == "__main__":
    raise SystemExit(main())

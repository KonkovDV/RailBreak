"""Offline bank of path hypotheses. Not the default ring filter.

H_j = (branch_j, s_j, v_j, w_j)
w_j ∝ w_j^- exp(-1/2 r_j^T S_j^{-1} r_j)

The route-10 node keeps one ring. This module does not import it.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

STEM_M = 40.0
ANGLE_DEG = 20.0
SIGMA_M = 2.0
SPEED_MPS = 10.0
DT_S = 0.5
HORIZON_S = 8.0


@dataclass(frozen=True)
class Hypothesis:
    branch: str
    s: float
    v: float
    w: float


@dataclass(frozen=True)
class Polyline:
    name: str
    xy: np.ndarray
    s_knot: np.ndarray

    @classmethod
    def from_xy(cls, name: str, xy) -> "Polyline":
        pts = np.asarray(xy, float)
        if len(pts) < 2:
            raise ValueError("a branch needs two points")
        step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        if np.any(step <= 0.0):
            raise ValueError("a branch segment has zero length")
        knots = np.concatenate([[0.0], np.cumsum(step)])
        return cls(name, pts, knots)

    @property
    def length(self) -> float:
        return float(self.s_knot[-1])

    def point(self, s: float) -> np.ndarray:
        q = float(np.clip(s, 0.0, self.length))
        return np.array(
            [np.interp(q, self.s_knot, self.xy[:, 0]), np.interp(q, self.s_knot, self.xy[:, 1])],
            float,
        )


def unnormalized_weight(w_minus: float, residual, S) -> float:
    """Gaussian hypothesis weight. S is the measurement covariance."""
    r = np.asarray(residual, float).reshape(-1)
    cov = np.asarray(S, float)
    quad = float(r @ np.linalg.solve(cov, r))
    if not math.isfinite(quad) or quad < 0.0:
        raise ValueError("residual quadratic form is not usable")
    return float(w_minus) * math.exp(-0.5 * quad)


def normalize(weights) -> np.ndarray:
    w = np.asarray(weights, float)
    total = float(np.sum(w))
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("hypothesis weights have no mass")
    return w / total


def predict(h: Hypothesis, dt: float) -> Hypothesis:
    return Hypothesis(h.branch, h.s + h.v * dt, h.v, h.w)


def update_bank(hyps: list[Hypothesis], branches: dict[str, Polyline], z, S) -> list[Hypothesis]:
    meas = np.asarray(z, float).reshape(-1)
    raw = []
    for h in hyps:
        residual = meas - branches[h.branch].point(h.s)
        raw.append(unnormalized_weight(h.w, residual, S))
    weights = normalize(raw)
    return [Hypothesis(h.branch, h.s, h.v, float(wi)) for h, wi in zip(hyps, weights)]


def synthetic_fork() -> dict[str, Polyline]:
    """Two branches share a stem, then one leaves at ANGLE_DEG."""
    ang = math.radians(ANGLE_DEG)
    run = 120.0
    main = Polyline.from_xy("main", [(0.0, 0.0), (STEM_M, 0.0), (STEM_M + run, 0.0)])
    side = Polyline.from_xy(
        "side",
        [(0.0, 0.0), (STEM_M, 0.0), (STEM_M + run * math.cos(ang), run * math.sin(ang))],
    )
    return {"main": main, "side": side}


def fork_demo() -> dict:
    """Truth follows the straight branch. Both hypotheses start equal at s = 0."""
    branches = synthetic_fork()
    cov = np.eye(2) * SIGMA_M ** 2
    hyps = [
        Hypothesis("main", 0.0, SPEED_MPS, 0.5),
        Hypothesis("side", 0.0, SPEED_MPS, 0.5),
    ]
    rows = []
    t = 0.0
    steps = int(round(HORIZON_S / DT_S))
    for k in range(steps + 1):
        if k:
            t = round(t + DT_S, 10)
            hyps = [predict(h, DT_S) for h in hyps]
        truth_s = SPEED_MPS * t
        z = branches["main"].point(truth_s)
        side_r = z - branches["side"].point(hyps[1].s)
        hyps = update_bank(hyps, branches, z, cov)
        by = {h.branch: h for h in hyps}
        rows.append({
            "t_s": t,
            "s_m": truth_s,
            "w_main": by["main"].w,
            "w_side": by["side"].w,
            "residual_side_m": float(np.linalg.norm(side_r)),
        })
    return {
        "default_filter": False,
        "route": "synthetic fork, not route 10",
        "stem_m": STEM_M,
        "angle_deg": ANGLE_DEG,
        "sigma_m": SIGMA_M,
        "v_mps": SPEED_MPS,
        "dt_s": DT_S,
        "rows": rows,
    }


def fork_svg(demo: dict) -> str:
    """Two branches and the weight of the straight hypothesis over time."""
    branches = synthetic_fork()
    rows = demo["rows"]

    def xy_of(name: str):
        b = branches[name]
        return [(float(x), float(y)) for x, y in b.xy]

    pts = xy_of("main") + xy_of("side")
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys) - 2.0, max(ys) + 2.0
    left, top, width, height = 40, 36, 520, 220

    def px(x, y):
        u = left + (x - min_x) / (max_x - min_x) * width
        v = top + (max_y - y) / (max_y - min_y) * height
        return u, v

    def poly(name: str) -> str:
        return " ".join(f"{px(x, y)[0]:.1f},{px(x, y)[1]:.1f}" for x, y in xy_of(name))

    chart_x, chart_y, chart_w, chart_h = 600, 36, 320, 220
    t_max = rows[-1]["t_s"] or 1.0

    def cx(t, w):
        u = chart_x + (t / t_max) * chart_w
        v = chart_y + chart_h - w * chart_h
        return u, v

    main_line = " ".join(f"{cx(r['t_s'], r['w_main'])[0]:.1f},{cx(r['t_s'], r['w_main'])[1]:.1f}" for r in rows)
    side_line = " ".join(f"{cx(r['t_s'], r['w_side'])[0]:.1f},{cx(r['t_s'], r['w_side'])[1]:.1f}" for r in rows)
    end = rows[-1]
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="960" height="300" viewBox="0 0 960 300">
  <rect width="960" height="300" fill="#f6f1e7"/>
  <text x="40" y="24" font-family="Arial" font-size="16" fill="#1a1814">Синтетическая стрелка, не маршрут 10</text>
  <polyline points="{poly("main")}" fill="none" stroke="#2f6f4e" stroke-width="3"/>
  <polyline points="{poly("side")}" fill="none" stroke="#8c4a3a" stroke-width="3"/>
  <text x="40" y="276" font-family="Arial" font-size="13" fill="#1a1814">общий ствол {STEM_M:.0f} м, затем {ANGLE_DEG:.0f}°. Зелёная — истина.</text>
  <rect x="{chart_x}" y="{chart_y}" width="{chart_w}" height="{chart_h}" fill="#fff" stroke="#1a1814"/>
  <polyline points="{main_line}" fill="none" stroke="#2f6f4e" stroke-width="2"/>
  <polyline points="{side_line}" fill="none" stroke="#8c4a3a" stroke-width="2"/>
  <text x="600" y="24" font-family="Arial" font-size="16" fill="#1a1814">Вес гипотезы</text>
  <text x="600" y="276" font-family="Arial" font-size="13" fill="#1a1814">к {end["t_s"]:.0f} с: прямая {end["w_main"]:.3f}, боковая {end["w_side"]:.3f}</text>
</svg>
"""


def _main() -> None:
    demo = fork_demo()
    for row in demo["rows"]:
        print(
            f"t={row['t_s']:4.1f}  s={row['s_m']:5.1f}  "
            f"w_main={row['w_main']:.6f}  w_side={row['w_side']:.6f}  "
            f"r_side={row['residual_side_m']:.3f}"
        )


if __name__ == "__main__":
    _main()

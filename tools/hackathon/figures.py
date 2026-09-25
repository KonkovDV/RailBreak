"""Offline SVG figures for T0/Phase-2 dry-run. No matplotlib, no UKF import."""

from __future__ import annotations

import math
from pathlib import Path


def _f(x: object) -> float | None:
    if x is None or x == "":
        return None
    try:
        v = float(x)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _svg(w: int, h: int, body: str) -> str:
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">\n{body}\n</svg>\n'
    )


def stanford_svg(est: list[dict], gt: list[dict], path: Path) -> None:
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    pts = []
    for e in est:
        t = round(float(e.get("t", 0.0)), 3)
        g = by_t.get(t)
        if g is None:
            continue
        s_hat = _f(e.get("s"))
        s_gt = _f(g.get("s", g.get("gt_s")))
        pl = _f(e.get("pl_s", e.get("over_m")))
        if s_hat is None or s_gt is None or pl is None or pl <= 0:
            continue
        err = abs(s_hat - s_gt)
        al = 5.0 + 0.05 * abs(s_gt)
        pts.append((pl, err, al))
    w, h, pad = 640, 480, 50
    if not pts:
        path.write_text(_svg(w, h, '<text x="20" y="40">Stanford: no matched OK/PL points</text>'),
                        encoding="utf-8")
        return
    xmax = max(max(p, e, a) for p, e, a in pts) * 1.05
    xmax = max(xmax, 1.0)
    def xy(px: float, py: float) -> tuple[float, float]:
        x = pad + (w - 2 * pad) * (px / xmax)
        y = h - pad - (h - 2 * pad) * (py / xmax)
        return x, y
    x0, y0 = xy(0, 0)
    x1, y1 = xy(xmax, xmax)
    circles = []
    for pl, err, al in pts:
        x, y = xy(pl, err)
        if err > pl and err > al:
            fill = "#b00020"
        elif err > pl:
            fill = "#e65100"
        else:
            fill = "#1565c0"
        circles.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.2" fill="{fill}" opacity="0.7"/>')
    body = [
        f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y0:.1f}" stroke="#333"/>',
        f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x0:.1f}" y2="{y1:.1f}" stroke="#333"/>',
        f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" '
        f'stroke="#666" stroke-dasharray="4 3"/>',
        f'<text x="{w/2:.0f}" y="{h-12}" text-anchor="middle" font-size="12">PL_s (m)</text>',
        f'<text x="14" y="{h/2:.0f}" transform="rotate(-90 14 {h/2:.0f})" font-size="12">'
        f'|ŝ−s| (m)</text>',
        '<text x="60" y="24" font-size="12">Stanford: blue=normal, orange=MI, red=HMI</text>',
        *circles,
    ]
    path.write_text(_svg(w, h, "\n".join(body)), encoding="utf-8")


def heatmap_svg(cells: list[dict], path: Path) -> None:
    """cells: {fault, method, rmse_s}."""
    faults = sorted({c["fault"] for c in cells})
    methods = []
    for c in cells:
        if c["method"] not in methods:
            methods.append(c["method"])
    w = 80 + 90 * max(len(methods), 1)
    h = 40 + 22 * max(len(faults), 1)
    vals = [float(c["rmse_s"]) for c in cells if math.isfinite(float(c.get("rmse_s", "nan")))]
    vmax = max(vals) if vals else 1.0
    lookup = {(c["fault"], c["method"]): c["rmse_s"] for c in cells}
    parts = ['<text x="8" y="16" font-size="12">Val inject: RMSE_s (m), test unused</text>']
    for j, m in enumerate(methods):
        parts.append(f'<text x="{90+j*90}" y="32" font-size="10">{m}</text>')
    for i, f in enumerate(faults):
        y = 48 + i * 22
        parts.append(f'<text x="8" y="{y+12}" font-size="10">{f}</text>')
        for j, m in enumerate(methods):
            v = lookup.get((f, m), float("nan"))
            if math.isfinite(v) and vmax > 0:
                t = min(max(v / vmax, 0.0), 1.0)
                fill = f"rgb({int(255*t)},{int(220*(1-t))},{int(80*(1-t))})"
                label = f"{v:.2f}"
            else:
                fill = "#eee"
                label = "n/a"
            x = 90 + j * 90
            parts.append(
                f'<rect x="{x}" y="{y}" width="86" height="20" fill="{fill}" stroke="#fff"/>'
                f'<text x="{x+43}" y="{y+14}" text-anchor="middle" font-size="10">{label}</text>'
            )
    path.write_text(_svg(max(w, 400), max(h, 80), "\n".join(parts)), encoding="utf-8")


def ride_svg(est: list[dict], gt: list[dict], path: Path) -> None:
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    series = []
    for e in est:
        t = _f(e.get("t"))
        if t is None:
            continue
        g = by_t.get(round(t, 3))
        s_hat = _f(e.get("s"))
        pl = _f(e.get("pl_s", e.get("over_m"))) or 0.0
        s_gt = _f(g.get("s", g.get("gt_s"))) if g else None
        if s_hat is None:
            continue
        series.append((t, s_hat, pl, s_gt))
    w, h, pad = 720, 280, 44
    if len(series) < 2:
        path.write_text(_svg(w, h, '<text x="20" y="40">ride: too few points</text>'),
                        encoding="utf-8")
        return
    t0, t1 = series[0][0], series[-1][0]
    ys = [p[1] for p in series] + [p[3] for p in series if p[3] is not None]
    ymin, ymax = min(ys), max(ys)
    if ymax - ymin < 1e-6:
        ymax = ymin + 1.0
    def xy(t: float, s: float) -> tuple[float, float]:
        x = pad + (w - 2 * pad) * ((t - t0) / max(t1 - t0, 1e-9))
        y = h - pad - (h - 2 * pad) * ((s - ymin) / (ymax - ymin))
        return x, y
    def poly(vals: list[tuple[float, float]], color: str, dash: str = "") -> str:
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in vals)
        extra = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline fill="none" stroke="{color}" stroke-width="1.4"{extra} points="{d}"/>'
    hat = [xy(t, s) for t, s, _pl, _g in series]
    gt_pts = [xy(t, g) for t, _s, _pl, g in series if g is not None]
    lo = [xy(t, s - pl) for t, s, pl, _g in series]
    hi = [xy(t, s + pl) for t, s, pl, _g in series]
    body = [
        poly(hat, "#1565c0"),
        poly(gt_pts, "#222", "4 3") if gt_pts else "",
        poly(lo, "#90caf9"),
        poly(hi, "#90caf9"),
        '<text x="50" y="18" font-size="12">ŝ (blue), GT (dashed), PL band (light)</text>',
    ]
    path.write_text(_svg(w, h, "\n".join(x for x in body if x)), encoding="utf-8")


def interval_svg(est: list[dict], gt: list[dict], path: Path) -> None:
    by_t = {round(float(g.get("t", g.get("t_s", 0.0))), 3): g for g in gt}
    series = []
    for e in est:
        if not e.get("interval_active"):
            continue
        t = _f(e.get("t"))
        lo = _f(e.get("v_lo"))
        hi = _f(e.get("v_hi"))
        g = by_t.get(round(t or 0.0, 3))
        vg = _f(g.get("v", g.get("gt_v"))) if g else None
        if t is None or lo is None or hi is None:
            continue
        series.append((t, lo, hi, vg))
    w, h, pad = 720, 260, 44
    if len(series) < 2:
        path.write_text(_svg(w, h, '<text x="20" y="40">interval: inactive</text>'),
                        encoding="utf-8")
        return
    t0, t1 = series[0][0], series[-1][0]
    ys = [x for p in series for x in (p[1], p[2], p[3]) if x is not None]
    ymin, ymax = min(ys), max(ys)
    if ymax - ymin < 1e-6:
        ymax = ymin + 1.0
    def xy(t: float, s: float) -> tuple[float, float]:
        x = pad + (w - 2 * pad) * ((t - t0) / max(t1 - t0, 1e-9))
        y = h - pad - (h - 2 * pad) * ((s - ymin) / (ymax - ymin))
        return x, y
    def poly(vals: list[tuple[float, float]], color: str) -> str:
        d = " ".join(f"{x:.1f},{y:.1f}" for x, y in vals)
        return f'<polyline fill="none" stroke="{color}" stroke-width="1.3" points="{d}"/>'
    body = [
        poly([xy(t, lo) for t, lo, _h, _v in series], "#2e7d32"),
        poly([xy(t, hi) for t, _l, hi, _v in series], "#c62828"),
        poly([xy(t, v) for t, _l, _h, v in series if v is not None], "#1565c0"),
        '<text x="50" y="18" font-size="12">M3: L (green), U (red), v_gt (blue)</text>',
    ]
    path.write_text(_svg(w, h, "\n".join(body)), encoding="utf-8")


def openloop_svg(series: list[dict], path: Path) -> None:
    """series: {name, e_s_2s, e_s_5s, e_s_10s, e_s_20s, e_s_30s}."""
    hs = (2, 5, 10, 20, 30)
    w, h, pad = 640, 280, 48
    vals = []
    for s in series:
        for H in hs:
            v = _f(s.get(f"e_s_{H}s"))
            if v is not None:
                vals.append(v)
    ymax = max(vals) * 1.1 if vals else 1.0
    colors = ("#1565c0", "#c62828", "#2e7d32", "#6a1b9a", "#ef6c00")
    parts = [
        '<text x="50" y="18" font-size="12">Command-only |e_s| vs horizon (m)</text>',
        f'<line x1="{pad}" y1="{h-pad}" x2="{w-pad}" y2="{h-pad}" stroke="#333"/>',
        f'<line x1="{pad}" y1="{h-pad}" x2="{pad}" y2="{pad}" stroke="#333"/>',
    ]
    for i, s in enumerate(series):
        pts = []
        for H in hs:
            v = _f(s.get(f"e_s_{H}s"))
            if v is None:
                continue
            x = pad + (w - 2 * pad) * ((H - 2) / 28.0)
            y = h - pad - (h - 2 * pad) * (v / ymax)
            pts.append(f"{x:.1f},{y:.1f}")
        if pts:
            col = colors[i % len(colors)]
            parts.append(
                f'<polyline fill="none" stroke="{col}" stroke-width="1.6" points="{" ".join(pts)}"/>'
            )
            parts.append(
                f'<text x="{w-pad-8}" y="{28+i*14}" text-anchor="end" font-size="11" '
                f'fill="{col}">{s.get("name", "?")}</text>'
            )
    path.write_text(_svg(w, h, "\n".join(parts)), encoding="utf-8")


def timing_svg(timing: dict, path: Path) -> None:
    """Three-bar p50 / p99 / max of replay_ukf tick_us."""
    w, h, pad = 480, 240, 48
    keys = ("p50_us", "p99_us", "max_us")
    vals = [_f(timing.get(k)) or 0.0 for k in keys]
    ymax = max(vals) * 1.15 if max(vals) > 0 else 1.0
    bw = (w - 2 * pad) / 5.0
    bars = []
    labels = ("p50", "p99", "max")
    colors = ("#1565c0", "#ef6c00", "#c62828")
    for i, (lab, v, col) in enumerate(zip(labels, vals, colors)):
        x = pad + (i + 0.6) * bw * 1.4
        bh = (h - 2 * pad) * (v / ymax)
        y = h - pad - bh
        bars.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bw:.1f}" height="{bh:.1f}" fill="{col}"/>')
        bars.append(f'<text x="{x + bw / 2:.1f}" y="{h - pad + 16}" text-anchor="middle" '
                    f'font-size="11">{lab}</text>')
        bars.append(f'<text x="{x + bw / 2:.1f}" y="{y - 4:.1f}" text-anchor="middle" '
                    f'font-size="10">{v:.1f}</text>')
    body = [
        '<text x="50" y="18" font-size="12">replay_ukf tick (µs)</text>',
        f'<line x1="{pad}" y1="{h-pad}" x2="{w-pad}" y2="{h-pad}" stroke="#333"/>',
        *bars,
    ]
    path.write_text(_svg(w, h, "\n".join(body)), encoding="utf-8")


def dropout_svg(points: list[dict], path: Path) -> None:
    """points: {t, e_s} seconds since dropout vs |e_s|."""
    w, h, pad = 640, 280, 48
    pts = []
    for p in points:
        t = _f(p.get("t"))
        e = _f(p.get("e_s"))
        if t is not None and e is not None:
            pts.append((t, e))
    if not pts:
        path.write_text(_svg(w, h, '<text x="20" y="40">dropout: no points</text>'),
                        encoding="utf-8")
        return
    t0, t1 = min(p[0] for p in pts), max(p[0] for p in pts)
    emax = max(p[1] for p in pts) * 1.1 or 1.0
    def xy(t: float, e: float) -> tuple[float, float]:
        x = pad + (w - 2 * pad) * ((t - t0) / max(t1 - t0, 1e-9))
        y = h - pad - (h - 2 * pad) * (e / emax)
        return x, y
    d = " ".join(f"{xy(t, e)[0]:.1f},{xy(t, e)[1]:.1f}" for t, e in pts)
    body = [
        f'<polyline fill="none" stroke="#c62828" stroke-width="1.4" points="{d}"/>',
        f'<line x1="{pad}" y1="{h-pad}" x2="{w-pad}" y2="{h-pad}" stroke="#333"/>',
        '<text x="50" y="18" font-size="12">|e_s| vs seconds after ω dropout</text>',
        f'<text x="{w/2:.0f}" y="{h-12}" text-anchor="middle" font-size="11">t since dropout (s)</text>',
    ]
    path.write_text(_svg(w, h, "\n".join(body)), encoding="utf-8")

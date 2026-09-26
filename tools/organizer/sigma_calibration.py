"""Along-track coverage of the filter sigma on one split.

Same along-track residual as eval_odometer.py: RTK status 2, projected on the
ring, kept when the fix is within 3 m, after the GNSS window. sigma_s is the
filter standard deviation interpolated to those stamps. Recordings are the
objects: the split is not a shuffle of one ride.

Prints the fraction of fixes with |e_s| <= k * sigma_s for k = 1, 2, 3,
pooled and as an unweighted mean across recordings. A Gaussian would sit near
0.683, 0.954, 0.997. This does not retune the station gate.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_model import grade_of  # noqa: E402
from eval_odometer import (  # noqa: E402
    CMD,
    FRONT,
    MFIX,
    REAR,
    build_ring,
    ring_init,
    track_reference,
)
from odometer import Odometer, Params  # noqa: E402
from reference import enu  # noqa: E402


def paired(path: Path, cl, model, stops, window: float) -> tuple[np.ndarray, np.ndarray] | None:
    z = np.load(path)
    if MFIX not in z.files or len(z[MFIX]) < 20 or FRONT not in z.files:
        return None
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    ring = build_ring(cl, stops)
    init = ring_init(z, window, ring, lat0, lon0)
    if init is None:
        return None
    s, h = ring["s"], ring["h"]
    od = Odometer(
        s, grade_of(s, h), model["table"], model["notches"], model["v_edges"],
        ring["stops"], Params(), ring_len=ring["L"],
    )
    od.init(init["s0"], max(init["d0"], 0.5))
    ev = []
    for key, kind in ((FRONT, "front"), (REAR, "rear")):
        a = z[key][np.argsort(z[key][:, 1])]
        for t, u in zip(a[:, 1], a[:, 2]):
            ev.append((float(t), kind, float(u)))
    if CMD in z.files:
        for row in z[CMD]:
            ev.append((float(row[1]), "cmd", float(row[2])))
    ev.sort(key=lambda e: e[0])
    out_t, out_s, out_sig = [], [], []
    started = False
    for t, kind, val in ev:
        if not started:
            if t < init["t0"]:
                continue
            od.t = t
            started = True
        if kind == "cmd":
            od.on_cmd(t, int(val))
        else:
            od.on_bogie(t, kind, float(val))
        ss, _, _, sg = od.state()
        out_t.append(t)
        out_s.append(ss)
        out_sig.append(sg)
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    s_ref, dist = track_reference(ring, x, y, init["s0"], t=g[:, 1])
    tr = g[:, 1]
    su = np.unwrap(np.array(out_s) * 2 * np.pi / ring["L"]) * ring["L"] / (2 * np.pi)
    si = np.interp(tr, np.array(out_t), su, left=np.nan, right=np.nan)
    sig = np.interp(tr, np.array(out_t), np.array(out_sig), left=np.nan, right=np.nan)
    m = (dist < 3.0) & np.isfinite(si) & np.isfinite(sig) & (sig > 0.0) & (tr > init["t0"] + window)
    if m.sum() < 100:
        return None
    Lr = ring["L"]
    e = (si[m] - s_ref[m] + 0.5 * Lr) % Lr - 0.5 * Lr
    return e, sig[m]


# Floor only so a zero sigma cannot divide. Scored sigmas sit well above it.
EPS_M = 1e-3
GAUSSIAN_K = (1.0, 2.0, 3.0)


def gaussian_coverage(k: float) -> float:
    """Two-sided normal probability P(|Z| <= k)."""
    return math.erf(float(k) / math.sqrt(2.0))


def ratio(e: np.ndarray, sig: np.ndarray, eps: float = EPS_M) -> np.ndarray:
    return np.abs(np.asarray(e, float)) / np.maximum(np.asarray(sig, float), eps)


def coverage_at(e: np.ndarray, sig: np.ndarray, ks=GAUSSIAN_K) -> list[float]:
    a = np.abs(np.asarray(e, float))
    s = np.asarray(sig, float)
    return [float(np.mean(a <= k * s)) for k in ks]


def weighted_quantile(values, weights, q: float) -> float:
    """Smallest value whose weighted CDF reaches q. Weights are renormalized."""
    v = np.asarray(values, float).reshape(-1)
    w = np.asarray(weights, float).reshape(-1)
    if len(v) == 0 or len(v) != len(w):
        raise ValueError("quantile needs values and matching weights")
    if not (0.0 < q <= 1.0):
        raise ValueError("q is outside (0, 1]")
    order = np.argsort(v, kind="mergesort")
    v, w = v[order], w[order]
    total = float(np.sum(w))
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("weights have no mass")
    cdf = np.cumsum(w) / total
    idx = int(np.searchsorted(cdf, q, side="left"))
    return float(v[min(idx, len(v) - 1)])


def equal_ride_quantile(rides: list[tuple[np.ndarray, np.ndarray]], q: float = 0.95) -> float:
    """Q_q of |e|/max(sigma, eps). Each ride has total weight 1, whatever its length."""
    values, weights = [], []
    for e, sig in rides:
        z = ratio(e, sig)
        if len(z) == 0:
            continue
        values.append(z)
        weights.append(np.full(len(z), 1.0 / len(z)))
    if not values:
        raise ValueError("no rides")
    return weighted_quantile(np.concatenate(values), np.concatenate(weights), q)


def equal_ride_coverage(rides: list[tuple[np.ndarray, np.ndarray]], ks=GAUSSIAN_K) -> list[float]:
    rows = [coverage_at(e, sig, ks) for e, sig in rides]
    return [float(x) for x in np.mean(np.asarray(rows, float), axis=0)]


def vehicle_of(name: str) -> str:
    return name.split("_", 1)[0]


def reliability_svg(ks, empirical, gaussian) -> str:
    """Empirical coverage against the two-sided normal. Not a certificate."""
    ks = list(ks)
    empirical = list(empirical)
    gaussian = list(gaussian)
    left, top, width, height = 70, 28, 560, 300
    k_max = max(ks)

    def px(k, cov):
        u = left + (k / k_max) * width
        v = top + (1.0 - cov) * height
        return u, v

    def line(series, color):
        pts = " ".join(f"{px(k, c)[0]:.1f},{px(k, c)[1]:.1f}" for k, c in zip(ks, series))
        return f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.5"/>'

    marks = []
    for k in GAUSSIAN_K:
        i = min(range(len(ks)), key=lambda j: abs(ks[j] - k))
        u, v = px(ks[i], empirical[i])
        marks.append(f'<circle cx="{u:.1f}" cy="{v:.1f}" r="4" fill="#8c4a3a"/>')
        marks.append(
            f'<text x="{u + 6:.1f}" y="{v - 6:.1f}" font-family="Arial" font-size="12" fill="#1a1814">'
            f'{k:.0f}σ {100 * empirical[i]:.1f}%</text>'
        )
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="720" height="380" viewBox="0 0 720 380">
  <rect width="720" height="380" fill="#f6f1e7"/>
  <text x="70" y="20" font-family="Arial" font-size="16" fill="#1a1814">Надёжность σ на val, равный вес рейса</text>
  <line x1="{left}" y1="{top}" x2="{left}" y2="{top + height}" stroke="#1a1814"/>
  <line x1="{left}" y1="{top + height}" x2="{left + width}" y2="{top + height}" stroke="#1a1814"/>
  {line(gaussian, "#2f6f4e")}
  {line(empirical, "#8c4a3a")}
  {''.join(marks)}
  <text x="70" y="360" font-family="Arial" font-size="13" fill="#1a1814">k в |e| ≤ kσ. Зелёная — нормальный закон. Бурая — измеренное покрытие. Не сертификат.</text>
</svg>
"""


def coverage(e: np.ndarray, sig: np.ndarray) -> list[float]:
    return coverage_at(e, sig)


def _round_map(keys, values) -> dict[str, float]:
    return {str(k): float(f"{v:.6f}") for k, v in zip(keys, values)}


def summarize(named: list[tuple[str, np.ndarray, np.ndarray]]) -> dict:
    rides = [(e, sig) for _n, e, sig in named]
    by_vehicle: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
    for name, e, sig in named:
        by_vehicle.setdefault(vehicle_of(name), []).append((e, sig))
    curve_k = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
    empirical = equal_ride_coverage(rides, curve_k)
    gauss = [gaussian_coverage(k) for k in curve_k]
    pooled_e = np.concatenate([e for _n, e, _s in named])
    pooled_s = np.concatenate([sig for _n, _e, sig in named])
    below = int(np.sum(pooled_s < EPS_M))
    per_vehicle = {}
    for vehicle, group in sorted(by_vehicle.items()):
        per_vehicle[vehicle] = {
            "n_rides": len(group),
            "n_fixes": int(sum(len(e) for e, _s in group)),
            "coverage_equal_ride": _round_map(GAUSSIAN_K, equal_ride_coverage(group)),
            "c_0_95": float(f"{equal_ride_quantile(group, 0.95):.6f}"),
        }
    c_ride = equal_ride_quantile(rides, 0.95)
    c_vehicle = max(row["c_0_95"] for row in per_vehicle.values())
    return {
        "name": "empirical uncertainty",
        "name_ru": "эмпирическая неопределённость",
        "certification_claim": False,
        "filter_changed": False,
        "hidden_test_used": False,
        "split": "val",
        "object": "ride",
        "preferred_object": "vehicle",
        "epsilon_m": EPS_M,
        "n_rides": len(rides),
        "n_fixes": int(len(pooled_e)),
        "n_sigma_below_epsilon": below,
        "gaussian": _round_map(GAUSSIAN_K, [gaussian_coverage(k) for k in GAUSSIAN_K]),
        "coverage_equal_ride": _round_map(GAUSSIAN_K, equal_ride_coverage(rides)),
        "coverage_pooled": _round_map(GAUSSIAN_K, coverage_at(pooled_e, pooled_s)),
        "c_0_95_equal_ride": float(f"{c_ride:.6f}"),
        "c_0_95_by_vehicle": {k: v["c_0_95"] for k, v in per_vehicle.items()},
        "c_0_95": float(f"{c_vehicle:.6f}"),
        "c_0_95_definition": (
            "Q_0.95 of |e_s|/max(sigma_s, epsilon) with each ride weight 1, "
            "computed inside each vehicle; c_0.95 is the larger vehicle value"
        ),
        "b_95": "c_0.95 * sigma_s",
        "vehicles": per_vehicle,
        "curve_k": curve_k,
        "curve_empirical_equal_ride": [float(f"{v:.6f}") for v in empirical],
        "curve_gaussian": [float(f"{v:.6f}") for v in gauss],
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org", type=Path, default=Path("local/org"))
    ap.add_argument("--splits", type=Path, default=Path("local/splits.json"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    ap.add_argument("--split", default="val")
    ap.add_argument("--gnss-window", type=float, default=3.0)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--svg", type=Path)
    args = ap.parse_args()
    if args.split == "test":
        print("hidden test is not used")
        return 2
    cl = np.load(args.map)
    model = dict(np.load(args.model / "model.npz"))
    stops = json.loads((args.model / "stops.json").read_text(encoding="utf-8"))
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    named = []
    rows = []
    for n in names:
        got = paired(args.org / f"{n}.npz", cl, model, stops, args.gnss_window)
        if got is None:
            print(f"  {n} skipped")
            continue
        e, sig = got
        cov = coverage(e, sig)
        named.append((n, e, sig))
        rows.append(cov)
        print(f"  {n} n={len(e)} 1s={cov[0]:.3f} 2s={cov[1]:.3f} 3s={cov[2]:.3f} "
              f"med|e|/s={np.median(np.abs(e) / sig):.2f}")
    if not rows:
        print("no recordings")
        return 1
    tab = np.array(rows)
    report = summarize(named)
    report["split"] = args.split
    print(f"\n{args.split} recordings={report['n_rides']} fixes={report['n_fixes']}")
    eq = report["coverage_equal_ride"]
    pooled = report["coverage_pooled"]
    print(f"pooled     1s={pooled['1.0']} 2s={pooled['2.0']} 3s={pooled['3.0']}")
    print(f"mean/rec   1s={eq['1.0']} 2s={eq['2.0']} 3s={eq['3.0']}")
    print(f"median/rec 1s={np.median(tab[:,0]):.3f} 2s={np.median(tab[:,1]):.3f} 3s={np.median(tab[:,2]):.3f}")
    print("gaussian   1s=0.683 2s=0.954 3s=0.997")
    print(f"c_0.95 equal-ride {report['c_0_95_equal_ride']}")
    print(f"c_0.95 by vehicle {report['c_0_95_by_vehicle']}")
    print(f"c_0.95 (worse vehicle) {report['c_0_95']}")
    if args.out:
        payload = {k: v for k, v in report.items() if not k.startswith("curve_")}
        args.out.write_text(json.dumps(payload, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    if args.svg:
        args.svg.write_text(
            reliability_svg(report["curve_k"], report["curve_empirical_equal_ride"], report["curve_gaussian"]),
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

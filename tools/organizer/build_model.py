"""Stops and the notch acceleration table, from the train split only.

Stops: standstill of both bogies >= 5 s, projected onto the branch with the
first RTK fixes of that dwell. Clustered at 30 m; kept when n >= 5.
Table: median wheel acceleration per (notch, 1 m/s bin) after adding back the
grade term g*i(s); cells with < 30 samples are filled from neighbours.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_map import _nearest, load_trip  # noqa: E402
from reference import enu  # noqa: E402

G = 9.80665
FRONT = "vehicle_front_bogie_velocity"
REAR = "vehicle_rear_bogie_velocity"
CMD = "vehicle_driver_position_cmd"
NOTCHES = np.arange(-15, 16)
V_EDGES = np.arange(0.0, 18.0, 1.0)


def wheel_speed(z) -> tuple[np.ndarray, np.ndarray]:
    f = z[FRONT][np.argsort(z[FRONT][:, 1])]
    r = z[REAR][np.argsort(z[REAR][:, 1])]
    rv = np.interp(f[:, 1], r[:, 1], r[:, 2], left=np.nan, right=np.nan)
    v = np.where(np.isfinite(rv), 0.5 * (f[:, 2] + rv), f[:, 2]) / 3.6
    return f[:, 1], v


def grade_of(s: np.ndarray, h: np.ndarray, win_m: float = 20.0) -> np.ndarray:
    k = max(1, int(win_m))
    pad = np.pad(h, (k, k), mode="edge")
    smooth = np.convolve(pad, np.ones(2 * k + 1) / (2 * k + 1), mode="same")[k:-k]
    return np.gradient(smooth, s)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("org", type=Path)
    ap.add_argument("splits", type=Path)
    ap.add_argument("map", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    cl = np.load(args.map)
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    names = json.loads(args.splits.read_text(encoding="utf-8"))["train"]
    stops: dict[str, list[float]] = {"SK": [], "KS": []}
    acc: dict[tuple[int, int], list[float]] = {}
    for name in names:
        path = args.org / f"{name}.npz"
        trip = load_trip(path, lat0, lon0)
        z = np.load(path)
        if FRONT not in z.files or CMD not in z.files or len(z[FRONT]) < 200:
            continue
        tw, v = wheel_speed(z)
        grade = None
        s_of_t = None
        if trip:
            key = trip["dir"]
            s, cx, cy, h = cl[f"{key}_s"], cl[f"{key}_x"], cl[f"{key}_y"], cl[f"{key}_h"]
            g = z["sensing_gnss_master_fix"]
            g = g[np.argsort(g[:, 1])]
            g = g[g[:, 5] == 2]
            x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
            j, d = _nearest(cx, cy, x, y)
            good = d < 2.0
            s_of_t = (g[good, 1], s[j[good]])
            gr = grade_of(s, h)
            grade = (s, gr)
            still = v < 0.05
            edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
            for a, b in zip(edges[::2], edges[1::2]):
                t_a, t_b = tw[a], tw[min(b, len(tw) - 1)]
                if t_b - t_a < 5.0:
                    continue
                m = (s_of_t[0] >= t_a) & (s_of_t[0] <= t_b)
                if m.sum() >= 5:
                    stops[key].append(float(np.median(s_of_t[1][m])))
        # acceleration table
        tt = np.arange(tw[0], tw[-1], 0.1)
        vi = np.interp(tt, tw, v)
        a = np.full_like(vi, np.nan)
        a[5:-5] = (vi[10:] - vi[:-10]) / 1.0
        c = z[CMD][np.argsort(z[CMD][:, 1])]
        idx = np.clip(np.searchsorted(c[:, 1], tt) - 1, 0, len(c) - 1)
        n = c[idx, 2].astype(int)
        held = np.ones(len(n), dtype=bool)
        for sh in range(1, 11):
            held &= np.roll(n, sh) == n
        corr = np.zeros_like(vi)
        if grade is not None and s_of_t is not None:
            si = np.interp(tt, s_of_t[0], s_of_t[1], left=np.nan, right=np.nan)
            ok = np.isfinite(si)
            corr[ok] = G * np.interp(si[ok], grade[0], grade[1])
            held &= ok
        else:
            held &= False
        for nn, vv, aa, cc, hh in zip(n, vi, a, corr, held):
            if hh and np.isfinite(aa) and vv > 0.3:
                acc.setdefault((int(nn), int(vv)), []).append(aa + cc)
    table = np.full((len(NOTCHES), len(V_EDGES)), np.nan)
    count = np.zeros_like(table)
    for i, nn in enumerate(NOTCHES):
        for jv in range(len(V_EDGES)):
            vals = acc.get((int(nn), jv), [])
            count[i, jv] = len(vals)
            if len(vals) >= 30:
                table[i, jv] = float(np.median(vals))
    # Fill: along speed first (nearest filled bin), then along notch.
    for i in range(len(NOTCHES)):
        row = table[i]
        ok = np.isfinite(row)
        if ok.any():
            table[i] = np.interp(np.arange(len(row)), np.flatnonzero(ok), row[ok])
    for jv in range(len(V_EDGES)):
        col = table[:, jv]
        ok = np.isfinite(col)
        if ok.any():
            table[:, jv] = np.interp(np.arange(len(col)), np.flatnonzero(ok), col[ok])
    table = np.nan_to_num(table, nan=0.0)
    stop_tab = {}
    for key, arr in stops.items():
        arr.sort()
        groups: list[list[float]] = []
        for sv in arr:
            if groups and sv - groups[-1][-1] < 30.0:
                groups[-1].append(sv)
            else:
                groups.append([sv])
        stop_tab[key] = [
            {"s": float(np.median(gp)), "sd": float(np.std(gp)), "n": len(gp)}
            for gp in groups
            if len(gp) >= 5
        ]
    args.out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out / "model.npz", notches=NOTCHES, v_edges=V_EDGES, table=table, count=count)
    (args.out / "stops.json").write_text(json.dumps(stop_tab, indent=1), encoding="utf-8")
    for key, rows in stop_tab.items():
        tight = [r for r in rows if r["sd"] < 3.0]
        print(key, "stops", len(rows), "tight", len(tight))
    print("table cells >=30 samples:", int((count >= 30).sum()), "of", count.size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Centerlines from the train split only. Output stays outside git.

Each full one-way trip is projected onto the longest same-direction trip,
then reduced to a 1 m running median. Validation trips are not read.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from reference import enu  # noqa: E402

# Termini from the published OSM stop list in route_10.yaml, not from a fit.
SHCH = (55.810004, 37.461373)
KUL = (55.800089, 37.392337)


def _nearest(cx, cy, x, y) -> tuple[np.ndarray, np.ndarray]:
    s_idx = np.empty(len(x), dtype=int)
    dist = np.empty(len(x))
    for k in range(0, len(x), 1500):
        dx = x[k : k + 1500, None] - cx[None, :]
        dy = y[k : k + 1500, None] - cy[None, :]
        d2 = dx * dx + dy * dy
        j = d2.argmin(axis=1)
        s_idx[k : k + 1500] = j
        dist[k : k + 1500] = np.sqrt(d2[np.arange(len(j)), j])
    return s_idx, dist


def load_trip(path: Path, lat0: float, lon0: float) -> dict | None:
    z = np.load(path)
    if "sensing_gnss_master_fix" not in z.files or len(z["sensing_gnss_master_fix"]) < 50:
        return None
    g = z["sensing_gnss_master_fix"]
    g = g[np.argsort(g[:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    step = np.hypot(np.diff(x), np.diff(y))
    if float(step[step < 30].sum()) < 3000:
        return None
    kx, ky = enu(np.array([KUL[0]]), np.array([KUL[1]]), lat0, lon0)
    sx, sy = enu(np.array([SHCH[0]]), np.array([SHCH[1]]), lat0, lon0)
    d0k = float(np.hypot(x[0] - kx[0], y[0] - ky[0]))
    d0s = float(np.hypot(x[0] - sx[0], y[0] - sy[0]))
    d1k = float(np.hypot(x[-1] - kx[0], y[-1] - ky[0]))
    d1s = float(np.hypot(x[-1] - sx[0], y[-1] - sy[0]))
    start = "K" if d0k < d0s else "S"
    end = "K" if d1k < d1s else "S"
    if start == end or min(d0k, d0s) > 400 or min(d1k, d1s) > 400:
        return None
    return {"name": path.stem, "dir": start + end, "x": x, "y": y, "h": g[:, 4]}


def median_line(trips: list[dict], ref_idx: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    base = trips[ref_idx]
    bx, by = base["x"], base["y"]
    ds = np.hypot(np.diff(bx), np.diff(by))
    s = np.r_[0.0, np.cumsum(np.where(ds < 30.0, ds, 0.0))]
    grid = np.arange(0.0, s[-1], 1.0)
    acc_x = [[] for _ in grid]
    acc_y = [[] for _ in grid]
    acc_h = [[] for _ in grid]
    for trip in trips:
        j, dist = _nearest(bx, by, trip["x"], trip["y"])
        for i, (jj, dd, hh) in enumerate(zip(j, dist, trip["h"])):
            bin_m = int(s[jj])
            if dd > 8.0 or bin_m < 0 or bin_m >= len(grid):
                continue
            acc_x[bin_m].append(trip["x"][i])
            acc_y[bin_m].append(trip["y"][i])
            acc_h[bin_m].append(hh)
    mx = np.array([np.median(v) if v else np.nan for v in acc_x])
    my = np.array([np.median(v) if v else np.nan for v in acc_y])
    mh = np.array([np.median(v) if v else np.nan for v in acc_h])
    ok = np.isfinite(mx) & np.isfinite(my)
    return grid[ok], mx[ok], my[ok], mh[ok]


def reparametrize(x, y, h, smooth_m: int = 7, step_m: float = 1.0):
    """Arc length of the smoothed median line itself, resampled every step_m.

    The median bins are indexed by the anchor trip's noisy arc length; using
    that as s makes the map ruler locally non-uniform.
    """
    k = max(1, smooth_m)
    ker = np.ones(2 * k + 1) / (2 * k + 1)

    def sm(a):
        pad = np.pad(a, (k, k), mode="edge")
        return np.convolve(pad, ker, mode="same")[k:-k]

    xs, ys, hs = sm(x), sm(y), sm(h)
    seg = np.hypot(np.diff(xs), np.diff(ys))
    s = np.r_[0.0, np.cumsum(seg)]
    grid = np.arange(0.0, s[-1], step_m)
    return grid, np.interp(grid, s, xs), np.interp(grid, s, ys), np.interp(grid, s, hs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("org", type=Path)
    ap.add_argument("splits", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--day", help="only trips starting on this UTC date (YYYY-MM-DD)")
    ap.add_argument("--name", default="centerlines.npz")
    args = ap.parse_args()
    names = json.loads(args.splits.read_text(encoding="utf-8"))["train"]
    if args.day:
        import datetime as dt

        def day_of(n: str) -> str:
            z = np.load(args.org / f"{n}.npz")
            t = float(z["vehicle_front_bogie_velocity"][0, 0])
            return dt.datetime.fromtimestamp(t, dt.UTC).date().isoformat()

        names = [n for n in names if day_of(n) == args.day]
    lat0, lon0 = 55.8040, 37.4250
    by_dir: dict[str, list[dict]] = {"SK": [], "KS": []}
    for name in names:
        trip = load_trip(args.org / f"{name}.npz", lat0, lon0)
        if trip:
            by_dir[trip["dir"]].append(trip)
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {"lat0": lat0, "lon0": lon0, "dirs": {}}
    payload = {"lat0": lat0, "lon0": lon0}
    for key, trips in by_dir.items():
        if not trips:
            continue
        longest = int(np.argmax([len(t["x"]) for t in trips]))
        s, x, y, h = median_line(trips, longest)
        s, x, y, h = reparametrize(x, y, h)
        payload[f"{key}_s"] = s
        payload[f"{key}_x"] = x
        payload[f"{key}_y"] = y
        payload[f"{key}_h"] = h
        summary["dirs"][key] = {"n_trips": len(trips), "length_m": float(s[-1]), "anchor": trips[longest]["name"]}
        print(key, "trips", len(trips), "length_m", round(float(s[-1]), 1), "anchor", trips[longest]["name"])
    np.savez_compressed(args.out / args.name, **payload)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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


def coverage(e: np.ndarray, sig: np.ndarray) -> list[float]:
    a = np.abs(e)
    return [float(np.mean(a <= k * sig)) for k in (1.0, 2.0, 3.0)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org", type=Path, default=Path("local/org"))
    ap.add_argument("--splits", type=Path, default=Path("local/splits.json"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    ap.add_argument("--split", default="val")
    ap.add_argument("--gnss-window", type=float, default=3.0)
    args = ap.parse_args()
    cl = np.load(args.map)
    model = dict(np.load(args.model / "model.npz"))
    stops = json.loads((args.model / "stops.json").read_text(encoding="utf-8"))
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    rows = []
    parts_e, parts_s = [], []
    for n in names:
        got = paired(args.org / f"{n}.npz", cl, model, stops, args.gnss_window)
        if got is None:
            print(f"  {n} skipped")
            continue
        e, sig = got
        cov = coverage(e, sig)
        rows.append(cov)
        parts_e.append(e)
        parts_s.append(sig)
        print(f"  {n} n={len(e)} 1s={cov[0]:.3f} 2s={cov[1]:.3f} 3s={cov[2]:.3f} "
              f"med|e|/s={np.median(np.abs(e) / sig):.2f}")
    if not rows:
        print("no recordings")
        return 1
    tab = np.array(rows)
    e = np.concatenate(parts_e)
    sig = np.concatenate(parts_s)
    micro = coverage(e, sig)
    print(f"\n{args.split} recordings={len(rows)} fixes={len(e)}")
    print(f"pooled     1s={micro[0]:.3f} 2s={micro[1]:.3f} 3s={micro[2]:.3f}")
    print(f"mean/rec   1s={tab[:,0].mean():.3f} 2s={tab[:,1].mean():.3f} 3s={tab[:,2].mean():.3f}")
    print(f"median/rec 1s={np.median(tab[:,0]):.3f} 2s={np.median(tab[:,1]):.3f} 3s={np.median(tab[:,2]):.3f}")
    print("gaussian   1s=0.683 2s=0.954 3s=0.997")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

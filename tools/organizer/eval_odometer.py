"""Run the reference odometer on a split and score it with the TZ metrics.

The estimate reads GNSS only in the first `--gnss-window` seconds after the
first fix, to choose the branch (master->rover azimuth) and s0. Everything
after that is bogie speed and notch.

The default map is the arc-length ring exported into
`railbreak_backup_odometry/assets` (`july27_arc.npz` + `model_arc`).
`local/map/july27.npz` with `local/model` is an earlier centreline: the same
val split then scores about 6.7 m along-track, not the published table.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_map import _nearest  # noqa: E402
from build_model import grade_of  # noqa: E402
import faults  # noqa: E402
from odometer import Odometer, Params  # noqa: E402
from reference import enu  # noqa: E402

FRONT = "vehicle_front_bogie_velocity"
REAR = "vehicle_rear_bogie_velocity"
CMD = "vehicle_driver_position_cmd"
MFIX = "sensing_gnss_master_fix"
RFIX = "sensing_gnss_rover_fix"
MVEL = "sensing_gnss_master_vel"


def branch_from_heading(z, window, cl, lat0, lon0):
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    t0 = g[0, 1]
    w = g[g[:, 1] <= t0 + window]
    w = w[w[:, 5] >= 0]
    if len(w) == 0:
        return None
    mlat, mlon = float(np.median(w[:, 2])), float(np.median(w[:, 3]))
    mx, my = enu(np.array([mlat]), np.array([mlon]), lat0, lon0)
    heading = None
    if RFIX in z.files and len(z[RFIX]):
        r = z[RFIX][np.argsort(z[RFIX][:, 1])]
        rw = r[(r[:, 1] <= t0 + window) & (r[:, 5] >= 0)]
        if len(rw):
            rx, ry = enu(np.array([np.median(rw[:, 2])]), np.array([np.median(rw[:, 3])]), lat0, lon0)
            heading = np.array([rx[0] - mx[0], ry[0] - my[0]])
    hyps = []
    for key in ("SK", "KS"):
        s, cx, cy = cl[f"{key}_s"], cl[f"{key}_x"], cl[f"{key}_y"]
        j, d = _nearest(cx, cy, mx, my)
        jj = int(j[0])
        tx = cx[min(jj + 5, len(cx) - 1)] - cx[max(jj - 5, 0)]
        ty = cy[min(jj + 5, len(cy) - 1)] - cy[max(jj - 5, 0)]
        aligned = True
        if heading is not None:
            cosang = (tx * heading[0] + ty * heading[1]) / (np.hypot(tx, ty) * np.hypot(*heading) + 1e-9)
            aligned = cosang >= 0
        hyps.append({"key": key, "s0": float(s[jj]), "d0": float(d[0]),
                     "remaining": float(s[-1] - s[jj]), "aligned": aligned})
    # The terminal loop is shared by the end of one branch and the start of the
    # other: position and azimuth fit both. A tram at a terminus is about to
    # depart, so among fitting hypotheses take the one with more path ahead.
    fit = [h for h in hyps if h["aligned"] and h["d0"] < 3.0]
    if not fit:
        fit = [min(hyps, key=lambda h: h["d0"] + (0.0 if h["aligned"] else 100.0))]
    best = max(fit, key=lambda h: h["remaining"])
    return {"branch": best["key"], "s0": best["s0"], "d0": best["d0"], "t0": float(t0),
            "has_rover": heading is not None, "n_fit": len(fit)}


def build_ring(cl, stops):
    """S->K then K->S as one closed coordinate. The K->S end meets the S->K
    start at Shchukinskaya; the short gap at Kulakova is bridged straight."""
    s1, x1, y1, h1 = cl["SK_s"], cl["SK_x"], cl["SK_y"], cl["SK_h"]
    s2, x2, y2, h2 = cl["KS_s"], cl["KS_x"], cl["KS_y"], cl["KS_h"]
    gap1 = float(np.hypot(x2[0] - x1[-1], y2[0] - y1[-1]))
    off2 = float(s1[-1]) + gap1
    gap2 = float(np.hypot(x1[0] - x2[-1], y1[0] - y2[-1]))
    s = np.r_[s1, off2 + s2]
    x = np.r_[x1, x2]
    y = np.r_[y1, y2]
    h = np.r_[h1, h2]
    L = float(off2 + s2[-1] + gap2)
    ring_stops = [dict(st) for st in stops.get("SK", [])]
    ring_stops += [{**st, "s": st["s"] + off2} for st in stops.get("KS", [])]
    return {"s": s, "x": x, "y": y, "h": h, "L": L, "stops": ring_stops, "off_ks": off2}


def ring_init(z, window, ring, lat0, lon0):
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    t0 = g[0, 1]
    w = g[(g[:, 1] <= t0 + window) & (g[:, 5] >= 0)]
    if len(w) == 0:
        return None
    mx, my = enu(np.array([np.median(w[:, 2])]), np.array([np.median(w[:, 3])]), lat0, lon0)
    heading = None
    if RFIX in z.files and len(z[RFIX]):
        r = z[RFIX][np.argsort(z[RFIX][:, 1])]
        rw = r[(r[:, 1] <= t0 + window) & (r[:, 5] >= 0)]
        if len(rw):
            rx, ry = enu(np.array([np.median(rw[:, 2])]), np.array([np.median(rw[:, 3])]), lat0, lon0)
            heading = np.array([rx[0] - mx[0], ry[0] - my[0]])
    x, y, s = ring["x"], ring["y"], ring["s"]
    d = np.hypot(x - mx[0], y - my[0])
    cand = np.flatnonzero(d < max(3.0, float(d.min()) + 0.5))
    best = None
    for j in cand:
        tx = x[min(j + 5, len(x) - 1)] - x[max(j - 5, 0)]
        ty = y[min(j + 5, len(y) - 1)] - y[max(j - 5, 0)]
        penalty = 0.0
        if heading is not None:
            cosang = (tx * heading[0] + ty * heading[1]) / (np.hypot(tx, ty) * np.hypot(*heading) + 1e-9)
            penalty = 0.0 if cosang >= 0 else 100.0
        score = float(d[j]) + penalty
        if best is None or score < best[0]:
            best = (score, int(j))
    j = best[1]
    return {"s0": float(s[j]), "d0": float(d[j]), "t0": float(t0), "has_rover": heading is not None}


def track_reference(ring, x, y, s_start, t=None, win=60.0, v_max=20.0):
    """Project each reference fix ahead of the previous locked reference s.

    The window grows with the time since the last lock, so an RTK gap on the
    bridge does not freeze the reference. Uses the reference only.
    """
    s_r, xr, yr = ring["s"], ring["x"], ring["y"]
    L = ring["L"]
    out = np.empty(len(x)); dist = np.empty(len(x))
    prev = s_start
    t_lock = t[0] if t is not None else 0.0
    for i in range(len(x)):
        w = win + (v_max * max(0.0, t[i] - t_lock) if t is not None else 0.0)
        ds = (s_r - prev + 0.5 * L) % L - 0.5 * L
        sel = np.flatnonzero((ds >= -win) & (ds <= w))
        if len(sel) == 0:
            sel = np.arange(len(s_r))
        dd = np.hypot(xr[sel] - x[i], yr[sel] - y[i])
        k = int(np.argmin(dd))
        out[i] = s_r[sel[k]]; dist[i] = dd[k]
        if dd[k] < 3.0:
            prev = out[i]
            if t is not None:
                t_lock = t[i]
    return out, dist


def run(path: Path, cl, model, stops, window: float, p: Params, fault: str = "none"):
    z = np.load(path)
    if MFIX not in z.files or len(z[MFIX]) < 20 or FRONT not in z.files:
        return None
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    ring = build_ring(cl, stops)
    init = ring_init(z, window, ring, lat0, lon0)
    if init is None:
        return None
    s, cx, cy, h = ring["s"], ring["x"], ring["y"], ring["h"]
    key = "SK" if init["s0"] < ring["off_ks"] else "KS"
    od = Odometer(s, grade_of(s, h), model["table"], model["notches"], model["v_edges"],
                  ring["stops"], p, ring_len=ring["L"])
    od.init(init["s0"], max(init["d0"], 0.5))
    ev = []
    streams = {}
    for key_t, kind in ((FRONT, "front"), (REAR, "rear")):
        a = z[key_t][np.argsort(z[key_t][:, 1])]
        streams[kind] = (a[:, 1], a[:, 2])
    t_on = None
    if fault != "none":
        tf, uf = streams["front"]
        t_on = faults.pick_moving_time(tf, uf, faults.fault_window(fault) + 5.0)
        for kind in ("front", "rear"):
            streams[kind] = faults.apply(fault, kind, *streams[kind], t_on)
    for kind, (tt, uu) in streams.items():
        for a_t, a_u in zip(tt, uu):
            ev.append((a_t, kind, a_u))
    for row in z[CMD]:
        ev.append((row[1], "cmd", row[2]))
    ev.sort(key=lambda e: e[0])
    out_t, out_s, out_v, out_k, out_sig, out_slip = [], [], [], [], [], []
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
        ss, vv, kk, sg = od.state()
        out_t.append(t); out_s.append(ss); out_v.append(vv); out_k.append(kk); out_sig.append(sg); out_slip.append(od.slip)
    est = {"t": np.array(out_t), "s": np.array(out_s), "v": np.array(out_v), "k": np.array(out_k),
           "sig": np.array(out_sig), "slip": np.array(out_slip)}
    # Reference: RTK fixes projected onto the same branch; GNSS speed.
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    s_ref, d = track_reference(ring, x, y, init["s0"], t=g[:, 1])
    ok = d < 3.0
    tr = g[:, 1]
    # Unwrapped estimate so interpolation does not cross the ring seam.
    su = np.unwrap(est["s"] * 2 * np.pi / ring["L"]) * ring["L"] / (2 * np.pi)
    si = np.interp(tr, est["t"], su, left=np.nan, right=np.nan)
    m = ok & np.isfinite(si) & (tr > init["t0"] + window)
    if m.sum() < 100:
        return None
    Lr = ring["L"]
    e_along = (si[m] - s_ref[m] + 0.5 * Lr) % Lr - 0.5 * Lr
    ref_u = np.unwrap(s_ref[m] * 2 * np.pi / Lr) * Lr / (2 * np.pi)
    L = float(abs(ref_u[-1] - ref_u[0]))
    si = si % Lr
    # position in ENU of the first fix (jury frame guess) to report 3D too
    ex = np.interp(si[m] % Lr, s, cx); ey = np.interp(si[m] % Lr, s, cy); ez = np.interp(si[m] % Lr, s, h)
    e3 = np.sqrt((ex - x[m]) ** 2 + (ey - y[m]) ** 2 + (ez - g[m, 4]) ** 2)
    prof = {}
    for sv, ev_ in zip(s_ref[m], e_along):
        prof.setdefault(int(sv // 250) * 250, []).append(ev_)
    res = {
        "profile": {str(k2): float(np.median(v2)) for k2, v2 in sorted(prof.items())},
        "bag": path.stem, "branch": key, "has_rover": init["has_rover"], "L": L,
        "along_mean": float(np.mean(np.abs(e_along))),
        "along_max": float(np.max(np.abs(e_along))),
        "along_rmse": float(np.sqrt(np.mean(e_along ** 2))),
        "end_pct": float(100 * abs(e_along[-1]) / max(L, 1.0)),
        "rmse_3d": float(np.sqrt(np.mean(e3 ** 2))),
        "k_end": float(est["k"][-1]),
        "n_anchor": od.n_anchor,
        "n_unique_far": int(sum(1 for row in od.anchor_log if row.get("reason") == "unique_far")),
        "slip_frac": float(np.mean(est["slip"])),
        "finite": bool(np.all(np.isfinite(est["s"])) and np.all(np.isfinite(est["v"]))),
    }
    if t_on is not None:
        dur = faults.fault_window(fault)
        tm = tr[m]
        inside = (tm >= t_on) & (tm <= t_on + dur)
        after = (tm > t_on + dur) & (tm <= t_on + dur + 60.0)
        before = (tm >= t_on - 60.0) & (tm < t_on)
        for tag, sel in (("before", before), ("during", inside), ("after", after)):
            res[f"along_{tag}_max"] = float(np.max(np.abs(e_along[sel]))) if sel.any() else float("nan")
        slip_t = est["t"][est["slip"]]
        res["slip_flag_in_window"] = bool(np.any((slip_t >= t_on) & (slip_t <= t_on + dur + 2.0)))
    if MVEL in z.files and len(z[MVEL]) > 50:
        mv = z[MVEL][np.argsort(z[MVEL][:, 1])]
        sp = np.hypot(mv[:, 2], mv[:, 3])  # module, not v · t(s)
        vi = np.interp(mv[:, 1], est["t"], est["v"], left=np.nan, right=np.nan)
        dtv = np.r_[1.0, np.diff(mv[:, 1])]
        mm = np.isfinite(vi) & (dtv < 0.15)
        err = vi[mm] - sp[mm]
        res["v_rmse"] = float(np.sqrt(np.mean(err ** 2)))
        res["v_mae"] = float(np.mean(np.abs(err)))
        acc = np.gradient(sp, mv[:, 1])[mm]
        for name, sel in (("acc", acc > 0.2), ("brk", acc < -0.2), ("stop", sp[mm] < 0.1)):
            res[f"v_bias_{name}"] = float(np.mean(err[sel])) if sel.any() else float("nan")
    return res


def summarize(rows):
    keys = ["along_rmse", "along_max", "along_mean", "end_pct", "rmse_3d", "v_rmse", "v_mae",
            "v_bias_acc", "v_bias_brk", "v_bias_stop"]
    out = {"n": len(rows)}
    for k in keys:
        a = np.array([r.get(k, np.nan) for r in rows], float)
        out[k] = {"median": float(np.nanmedian(a)), "p95": float(np.nanpercentile(a, 95))}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--org", type=Path, default=Path("local/org"))
    ap.add_argument("--splits", type=Path, default=Path("local/splits.json"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    ap.add_argument("--split", default="val")
    ap.add_argument("--gnss-window", type=float, default=3.0)
    ap.add_argument("--no-anchor", action="store_true")
    ap.add_argument("--fault", default="none", choices=sorted(faults.FAULTS))
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    cl = np.load(args.map)
    model = dict(np.load(args.model / "model.npz"))
    stops = json.loads((args.model / "stops.json").read_text(encoding="utf-8"))
    if args.no_anchor:
        stops = {k: [] for k in stops}
    names = json.loads(args.splits.read_text(encoding="utf-8"))[args.split]
    rows = [r for n in names
            if (r := run(args.org / f"{n}.npz", cl, model, stops, args.gnss_window, Params(), args.fault))]
    summ = summarize(rows)
    if args.out:
        args.out.write_text(json.dumps({"summary": summ, "rows": rows}, indent=1), encoding="utf-8")
    print(f"{args.split} n={summ['n']}")
    for k, v in summ.items():
        if k != "n":
            print(f"  {k:12s} med {v['median']:8.3f}  p95 {v['p95']:8.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

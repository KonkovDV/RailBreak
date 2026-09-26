"""Run the fault campaign on the Python twin. The filter is not modified.

  python tools/organizer/fault_campaign.py --list
  python tools/organizer/fault_campaign.py --split val --out local/fault_campaign.json
  python tools/organizer/fault_campaign.py --cases front_scale_p5 --limit 1 --out local/fault_smoke.json

Single-bogie attacks are applied once on the front bogie and once on the rear.
Scale and bias hold from the moving instant to the end of the run. A frozen
bogie, a zero, a NaN burst and a dropout are windows. The spike is one sample
of +40 km/h, scored over the following 1 s. Delay shifts that bogie's stamps. `--grid rear` is the magnitude-by-duration
table for the heatmap: scale 0.90 … 1.10 and durations 5 s, 15 s, 30 s, and
the rest of the run. `--report` writes one JSON record per cell and
`fault_heatmap.svg`.

  python tools/organizer/fault_campaign.py --grid rear --jobs 4 --out local/fault_grid.json --report docs/solution

Both-bogie slide is −20 % for 8 s, spin is +20 % for 8 s, the shared freeze
is 30 s. Simultaneous dropouts use the same durations as the single-bogie
list. The opposite-sign case is front +10 % and rear −10 % for 30 s. The
sequential case silences the front for 5 s and then the rear for 5 s. The
notch case starts a front +10 % for 10 s at the first notch change while moving.

Stamp gaps of 29 s sit inside max_gap_s (30 s), so the twin integrates.
A gap of 31 s is past that limit, so the twin resets the clock. Callback
reorder and the stamp regression are delivered in arrival order; every other
attack is sorted by stamp before replay.

GNSS attacks change only the fixes used to choose s0. Rover height, a fix
kept only inside the start window, and a last fix placed off the ring do not
enter this twin; those rows are still run and marked changes_filter false.
The scored reference is always the clean master track.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_map import _nearest  # noqa: E402
from build_model import grade_of  # noqa: E402
from eval_odometer import (  # noqa: E402
    CMD,
    FRONT,
    MFIX,
    MVEL,
    REAR,
    RFIX,
    build_ring,
    track_reference,
)
from faults import pick_moving_time  # noqa: E402
from odometer import Odometer, Params  # noqa: E402
from reference import enu  # noqa: E402

CTX: dict = {}


@dataclass(frozen=True)
class Case:
    id: str
    group: str
    kind: str
    which: str = ""
    value: float = 0.0
    dur: float = 0.0
    changes_filter: bool = True
    localized: bool = True


def _cases() -> list[Case]:
    out: list[Case] = []
    scales = (
        ("p1", 1.01), ("p3", 1.03), ("p5", 1.05), ("p10", 1.10),
        ("m1", 0.99), ("m3", 0.97), ("m5", 0.95), ("m10", 0.90),
    )
    biases = (
        ("p0_2", 0.2), ("p0_5", 0.5), ("p1", 1.0),
        ("m0_2", -0.2), ("m0_5", -0.5), ("m1", -1.0),
    )
    drops = (("0_5", 0.5), ("2", 2.0), ("5", 5.0), ("15", 15.0), ("30", 30.0))
    delays = (("50ms", 0.05), ("100ms", 0.10), ("200ms", 0.20), ("400ms", 0.40))
    for which in ("front", "rear"):
        for tag, factor in scales:
            out.append(Case(f"{which}_scale_{tag}", "single", "scale", which, factor, 0.0))
        for tag, mps in biases:
            out.append(Case(f"{which}_bias_{tag}", "single", "bias", which, mps, 0.0))
        out.append(Case(f"{which}_frozen", "single", "frozen", which, dur=30.0))
        out.append(Case(f"{which}_zero", "single", "zero", which, dur=10.0))
        out.append(Case(f"{which}_nan", "single", "nan", which, dur=10.0))
        out.append(Case(f"{which}_spike", "single", "spike", which, value=40.0, dur=1.0))
        for tag, dur in drops:
            out.append(Case(f"{which}_drop_{tag}", "single", "dropout", which, dur=dur))
        for tag, sec in delays:
            out.append(Case(f"{which}_delay_{tag}", "single", "delay", which, value=sec, dur=0.0))
    for tag, factor in scales:
        out.append(Case(f"both_scale_{tag}", "both", "scale", "both", factor, 0.0))
    out.append(Case("both_slide_m20", "both", "scale", "both", 0.80, 8.0))
    out.append(Case("both_spin_p20", "both", "scale", "both", 1.20, 8.0))
    out.append(Case("both_frozen", "both", "both_frozen", "both", dur=30.0))
    for tag, dur in drops:
        out.append(Case(f"both_drop_{tag}", "both", "dropout", "both", dur=dur))
    out.append(Case("both_opposite", "both", "opposite", "both", dur=30.0))
    out.append(Case("both_front_then_rear", "both", "sequential", "both", dur=10.0))
    out.append(Case("both_after_notch", "both", "after_notch", "front", value=1.10, dur=10.0))
    out.append(Case("time_reorder", "time", "reorder", dur=10.0))
    out.append(Case("time_regress", "time", "regress", dur=2.0))
    out.append(Case("time_equal_stamp", "time", "equal_stamp", dur=2.0))
    for tag, dur in (("0_3", 0.3), ("1", 1.0), ("5", 5.0), ("29", 29.0), ("31", 31.0)):
        out.append(Case(f"time_gap_{tag}", "time", "gap", "both", dur=dur))
    out.append(Case("time_rate_split", "time", "rate", "rear", dur=0.0, localized=False))
    out.append(Case("time_burst", "time", "burst", "both", dur=1.0))
    for kind, reaches in (
        ("gnss_master_rover", True),
        ("gnss_master_only", True),
        ("gnss_rover_only", True),
        ("gnss_none", True),
        ("gnss_nofix_after_start", True),
        ("gnss_window_only", False),
        ("gnss_master_later", True),
        ("gnss_rover_later", True),
        ("gnss_chord_80m", True),
        ("gnss_rover_up_3m", False),
        ("gnss_late_vs_wheels", True),
    ):
        out.append(Case(kind, "gnss", kind, changes_filter=reaches, localized=False))
    for kind in (
        "map_stop_missed", "map_stop_extra", "map_stop_at_light", "map_wrong_branch",
        "map_shared_terminus", "map_start_moving", "map_last_fix_off_ring",
    ):
        reaches = kind != "map_last_fix_off_ring"
        out.append(Case(kind, "map", kind, changes_filter=reaches, localized=False))
    return out


CASES = _cases()


def attack_mask(t: np.ndarray, t_on: float, dur: float) -> np.ndarray:
    """dur <= 0 holds from t_on through the end of the run."""
    if dur <= 0.0:
        return t >= t_on
    return (t >= t_on) & (t <= t_on + dur)


def outside_window(t: np.ndarray, t_on: float, dur: float) -> np.ndarray:
    """Keep the samples at the edges. The open interval is the dropout or the gap."""
    return (t <= t_on) | (t >= t_on + dur)


def offset_east(fix: np.ndarray, meters: float) -> np.ndarray:
    out = np.array(fix, copy=True)
    lat = out[:, 2]
    out[:, 3] = out[:, 3] + meters / (111320.0 * np.cos(np.deg2rad(lat)))
    return out


def apply_streams(case: Case, front, rear, t_on: float):
    """Return (front, rear, applied). Streams are (t, u_kmh)."""
    ft, fu = np.array(front[0], copy=True), np.array(front[1], copy=True)
    rt, ru = np.array(rear[0], copy=True), np.array(rear[1], copy=True)
    kind = case.kind
    if kind == "scale":
        factor = case.value

        def scale(pair):
            t, u = pair
            u = np.array(u, copy=True)
            u[attack_mask(t, t_on, case.dur)] *= factor
            return t, u

        ft, fu = scale((ft, fu)) if case.which in ("front", "both") else (ft, fu)
        rt, ru = scale((rt, ru)) if case.which in ("rear", "both") else (rt, ru)
        return (ft, fu), (rt, ru), True
    if kind == "bias":
        add = case.value * 3.6

        def bias(t, u):
            u = np.array(u, copy=True)
            u[attack_mask(t, t_on, case.dur)] += add
            return t, u

        if case.which in ("front", "both"):
            ft, fu = bias(ft, fu)
        if case.which in ("rear", "both"):
            rt, ru = bias(rt, ru)
        return (ft, fu), (rt, ru), True
    if kind == "frozen":
        t, u = (ft, fu) if case.which == "front" else (rt, ru)
        i = int(np.searchsorted(t, t_on))
        if i >= len(u):
            return (ft, fu), (rt, ru), False
        held = float(u[i])
        u = np.array(u, copy=True)
        u[attack_mask(t, t_on, case.dur)] = held
        if case.which == "front":
            return (t, u), (rt, ru), True
        return (ft, fu), (t, u), True
    if kind == "both_frozen":
        i = int(np.searchsorted(ft, t_on))
        if i >= len(fu):
            return (ft, fu), (rt, ru), False
        held = float(fu[i])
        for t, u in ((ft, fu), (rt, ru)):
            u[attack_mask(t, t_on, case.dur)] = held
        return (ft, fu), (rt, ru), True
    if kind == "zero":
        t, u = (ft, fu) if case.which == "front" else (rt, ru)
        u = np.array(u, copy=True)
        u[attack_mask(t, t_on, case.dur)] = 0.0
        if case.which == "front":
            return (t, u), (rt, ru), True
        return (ft, fu), (t, u), True
    if kind == "nan":
        t, u = (ft, fu) if case.which == "front" else (rt, ru)
        u = np.array(u, dtype=float, copy=True)
        u[attack_mask(t, t_on, case.dur)] = np.nan
        if case.which == "front":
            return (t, u), (rt, ru), True
        return (ft, fu), (t, u), True
    if kind == "spike":
        t, u = (ft, fu) if case.which == "front" else (rt, ru)
        i = int(np.searchsorted(t, t_on))
        if i >= len(u):
            return (ft, fu), (rt, ru), False
        u = np.array(u, copy=True)
        u[i] = u[i] + case.value
        if case.which == "front":
            return (t, u), (rt, ru), True
        return (ft, fu), (t, u), True
    if kind == "dropout":
        def cut(t, u):
            keep = outside_window(t, t_on, case.dur)
            return t[keep], u[keep]

        if case.which in ("front", "both"):
            ft, fu = cut(ft, fu)
        if case.which in ("rear", "both"):
            rt, ru = cut(rt, ru)
        return (ft, fu), (rt, ru), True
    if kind == "delay":
        def lag(t, u):
            t = np.array(t, copy=True)
            t[t >= t_on] += case.value
            return t, u

        if case.which in ("front", "both"):
            ft, fu = lag(ft, fu)
        if case.which in ("rear", "both"):
            rt, ru = lag(rt, ru)
        return (ft, fu), (rt, ru), True
    if kind == "opposite":
        fu = np.array(fu, copy=True)
        ru = np.array(ru, copy=True)
        fu[attack_mask(ft, t_on, case.dur)] *= 1.10
        ru[attack_mask(rt, t_on, case.dur)] *= 0.90
        return (ft, fu), (rt, ru), True
    if kind == "sequential":
        fu_keep = outside_window(ft, t_on, 5.0)
        ru_keep = outside_window(rt, t_on + 5.0, 5.0)
        return (ft[fu_keep], fu[fu_keep]), (rt[ru_keep], ru[ru_keep]), True
    if kind == "after_notch":
        fu = np.array(fu, copy=True)
        fu[attack_mask(ft, t_on, case.dur)] *= case.value
        return (ft, fu), (rt, ru), True
    if kind == "gap":
        fk, rk = outside_window(ft, t_on, case.dur), outside_window(rt, t_on, case.dur)
        return (ft[fk], fu[fk]), (rt[rk], ru[rk]), True
    if kind == "rate":
        if len(rt) >= 5:
            rt, ru = rt[::5], ru[::5]
        return (ft, fu), (rt, ru), True
    if kind == "burst":
        def bunch(t, u):
            t = np.array(t, copy=True)
            t[attack_mask(t, t_on, case.dur)] = t_on + case.dur
            return t, u

        return bunch(ft, fu), bunch(rt, ru), True
    if kind == "equal_stamp":
        rt = np.array(rt, copy=True)
        m = attack_mask(rt, t_on, case.dur)
        for i in np.flatnonzero(m):
            j = int(np.argmin(np.abs(ft - rt[i])))
            if abs(ft[j] - rt[i]) <= 0.2:
                rt[i] = ft[j]
        return (ft, fu), (rt, ru), True
    return (ft, fu), (rt, ru), True


def notch_time(cmd: np.ndarray, front_t: np.ndarray, front_u: np.ndarray) -> float | None:
    if cmd is None or len(cmd) < 2 or len(front_t) == 0:
        return None
    c = cmd[np.argsort(cmd[:, 1])]
    start = float(front_t[0] + 0.25 * (front_t[-1] - front_t[0]))
    for i in range(1, len(c)):
        if c[i, 1] < start or c[i, 2] == c[i - 1, 2]:
            continue
        j = int(np.searchsorted(front_t, c[i, 1]))
        j = min(j, len(front_u) - 1)
        if front_u[j] > 20.0:
            return float(c[i, 1])
    return None


def events_of(front, rear, cmd, t_on: float | None = None, gap: float = 0.0) -> list:
    ev = []
    for kind, (tt, uu) in (("front", front), ("rear", rear)):
        for a, b in zip(tt, uu):
            ev.append((float(a), kind, float(b)))
    if cmd is not None and len(cmd):
        use = cmd
        if gap > 0.0 and t_on is not None:
            keep = outside_window(cmd[:, 1], t_on, gap)
            use = cmd[keep]
        for row in use:
            ev.append((float(row[1]), "cmd", float(row[2])))
    ev.sort(key=lambda e: e[0])
    return ev


def reorder_events(ev: list, t_on: float, dur: float) -> tuple[list, bool]:
    out = list(ev)
    i = 0
    landed = False
    while i < len(out) - 1:
        t0, k0, _v0 = out[i]
        t1, k1, _v1 = out[i + 1]
        if t_on <= t0 <= t_on + dur and k0 != k1 and 0.0 < (t1 - t0) < 0.05:
            out[i], out[i + 1] = out[i + 1], out[i]
            landed = True
            i += 2
        else:
            i += 1
    return out, landed


def regress_events(ev: list, t_on: float) -> tuple[list, bool]:
    out = list(ev)
    for i in range(1, len(out)):
        t, kind, val = out[i]
        if kind == "front" and t_on <= t <= t_on + 2.0:
            out[i] = (out[i - 1][0] - 0.2, kind, val)
            return out, True
    return out, False


def init_arrays(master, rover, window: float, ring, lat0: float, lon0: float):
    if master is None or len(master) == 0:
        return None
    g = master[np.argsort(master[:, 1])]
    t0 = float(g[0, 1])
    w = g[(g[:, 1] <= t0 + window) & (g[:, 5] >= 0)]
    if len(w) == 0:
        return None
    mx, my = enu(np.array([np.median(w[:, 2])]), np.array([np.median(w[:, 3])]), lat0, lon0)
    heading = None
    if rover is not None and len(rover):
        r = rover[np.argsort(rover[:, 1])]
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
    return {"s0": float(s[j]), "d0": float(d[j]), "t0": float(t0), "has_rover": heading is not None,
            "mx": float(mx[0]), "my": float(my[0])}


def mutate_gnss(kind: str, master, rover, window: float):
    master = None if master is None else np.array(master, copy=True)
    rover = None if rover is None else np.array(rover, copy=True)
    early = 0.0
    applied = True
    if kind == "gnss_master_only":
        rover = None
    elif kind == "gnss_rover_only":
        master[:, 5] = -1.0
    elif kind == "gnss_none":
        master[:, 5] = -1.0
        if rover is not None:
            rover[:, 5] = -1.0
    elif kind == "gnss_nofix_after_start":
        t0 = float(master[np.argsort(master[:, 1])][0, 1])
        master[master[:, 1] > t0 + 1.0, 5] = -1.0
        if rover is not None and len(rover):
            rover[rover[:, 1] > t0 + 1.0, 5] = -1.0
    elif kind == "gnss_window_only":
        t0 = float(master[np.argsort(master[:, 1])][0, 1])
        master = master[(master[:, 1] >= t0) & (master[:, 1] <= t0 + window)]
        if rover is not None and len(rover):
            rover = rover[(rover[:, 1] >= t0) & (rover[:, 1] <= t0 + window)]
    elif kind == "gnss_master_later":
        t0 = float(master[np.argsort(master[:, 1])][0, 1])
        master = master[master[:, 1] >= t0 + 2.0]
        applied = len(master) > 0
    elif kind == "gnss_rover_later":
        if rover is None or len(rover) == 0:
            applied = False
        else:
            t0 = float(master[np.argsort(master[:, 1])][0, 1])
            rover = rover[rover[:, 1] >= t0 + 2.0]
            applied = len(rover) > 0
    elif kind == "gnss_chord_80m":
        if rover is None or len(rover) == 0:
            applied = False
        else:
            rover = offset_east(rover, 80.0)
    elif kind == "gnss_rover_up_3m":
        if rover is None or len(rover) == 0:
            applied = False
        else:
            rover[:, 4] += 3.0
    elif kind == "gnss_late_vs_wheels":
        master[:, 1] += 1.0
        if rover is not None and len(rover):
            rover[:, 1] += 1.0
        early = 1.0
    return master, rover, early, applied


def branch_fits(master, rover, window: float, cl, lat0: float, lon0: float) -> list[dict]:
    g = master[np.argsort(master[:, 1])]
    if len(g) == 0:
        return []
    t0 = float(g[0, 1])
    w = g[(g[:, 1] <= t0 + window) & (g[:, 5] >= 0)]
    if len(w) == 0:
        return []
    mlat, mlon = float(np.median(w[:, 2])), float(np.median(w[:, 3]))
    mx, my = enu(np.array([mlat]), np.array([mlon]), lat0, lon0)
    heading = None
    if rover is not None and len(rover):
        r = rover[np.argsort(rover[:, 1])]
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
                     "remaining": float(s[-1] - s[jj]), "aligned": bool(aligned)})
    return [h for h in hyps if h["aligned"] and h["d0"] < 3.0]


def _ring_delta(a: float, b: float, length: float) -> float:
    d = a - b
    if length > 0:
        d = (d + 0.5 * length) % length - 0.5 * length
    return float(d)


def moving_start(master, front_t, front_u):
    """First valid master fix whose nearest front sample is above 5 m/s."""
    g = master[np.argsort(master[:, 1])]
    g = g[g[:, 5] >= 0]
    if len(g) == 0 or len(front_t) == 0:
        return None
    for row in g:
        j = min(int(np.searchsorted(front_t, row[1])), len(front_u) - 1)
        if front_u[j] > 18.0:
            return float(row[1])
    return None


def dwell_s(front_t, front_u, s0: float, t0: float, stops: list, length: float) -> float | None:
    m = front_t >= t0
    tt, uu = front_t[m], front_u[m] / 3.6
    if len(tt) < 10:
        return None
    dt = np.diff(tt, prepend=tt[0])
    dt[0] = 0.0
    dt = np.clip(dt, 0.0, 0.2)
    s = np.mod(s0 + np.cumsum(np.maximum(uu, 0.0) * dt), length)
    still = uu < 0.05
    edges = np.flatnonzero(np.diff(np.r_[0, still.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        b = min(int(b), len(tt))
        if b <= a:
            continue
        if tt[b - 1] - tt[a] < 3.0 or tt[a] < t0 + 30.0:
            continue
        sm = float(s[(a + b - 1) // 2])
        near = any(abs(_ring_delta(st["s"], sm, length)) < 40.0 for st in stops)
        if not near:
            return sm
    return None


def recovery_s(st, sf, i0: int, t_end: float) -> float | None:
    """Seconds after the fault ends until this slip episode stays false for 1 s.

    A later rise is another episode. A quiet start before the fault ends is 0.
    """
    quiet_since = None
    for i in range(i0, len(st)):
        if sf[i]:
            quiet_since = None
            continue
        if quiet_since is None:
            quiet_since = float(st[i])
        if float(st[i]) - quiet_since >= 1.0 or i == len(st) - 1:
            return max(0.0, quiet_since - t_end)
    return None


def _blank(bag: str, case: Case, applied: bool, started: bool, **extra) -> dict:
    row = {
        "bag": bag, "applied": applied, "started": started, "changes_filter": case.changes_filter,
        "finite": False, "along_rmse": None, "end_pct": None, "v_rmse": None,
        "along_during_max": None, "along_after_max": None, "slip_flag": None,
        "slip_edges": None, "n_behind": None, "n_anchor": None, "s0_delta": None,
    }
    row.update(extra)
    return row


def _score(prep, est, slip, t_on: float, dur: float, localized: bool) -> dict:
    tr = prep["tr"]
    su = np.unwrap(est["s"] * 2 * np.pi / prep["L"]) * prep["L"] / (2 * np.pi)
    si = np.interp(tr, est["t"], su, left=np.nan, right=np.nan)
    m = prep["on_track"] & np.isfinite(si)
    if int(m.sum()) < 100:
        return {}
    length = prep["L"]
    e = (si[m] - prep["s_ref"][m] + 0.5 * length) % length - 0.5 * length
    ref_u = np.unwrap(prep["s_ref"][m] * 2 * np.pi / length) * length / (2 * np.pi)
    travelled = float(abs(ref_u[-1] - ref_u[0]))
    out = {
        "finite": bool(np.all(np.isfinite(est["s"])) and np.all(np.isfinite(est["v"]))),
        "along_rmse": float(np.sqrt(np.mean(e ** 2))),
        "end_pct": float(100.0 * abs(e[-1]) / max(travelled, 1.0)),
        "n_anchor": int(est["n_anchor"]),
    }
    tm = tr[m]
    if localized:
        if dur > 0.0:
            inside = (tm >= t_on) & (tm <= t_on + dur)
            after = (tm > t_on + dur) & (tm <= t_on + dur + 60.0)
        else:
            inside = tm >= t_on
            after = np.zeros(len(tm), dtype=bool)
        out["along_during_max"] = float(np.max(np.abs(e[inside]))) if inside.any() else None
        out["along_after_max"] = float(np.max(np.abs(e[after]))) if after.any() else None
        # Published flag: the attack plus 2 s. A fault with no end uses that 2 s
        # probe, not every later slip on the ride.
        det_hi = t_on + dur + 2.0 if dur > 0.0 else t_on + 2.0
        st, sf = slip
        det = (st >= t_on) & (st <= det_hi) & sf
        out["slip_flag"] = bool(det.any())
        window = (st >= t_on) & (st <= det_hi)
        flags = sf[window]
        if len(flags):
            rise = np.flatnonzero(flags[1:] & ~flags[:-1])
            out["slip_edges"] = int(len(rise) + (1 if flags[0] else 0))
        else:
            out["slip_edges"] = 0
        if det.any():
            i0 = int(np.flatnonzero(det)[0])
            out["detect_latency_s"] = float(st[i0] - t_on)
            out["recovery_s"] = recovery_s(st, sf, i0, t_on + dur) if dur > 0.0 else None
        else:
            out["detect_latency_s"] = None
            out["recovery_s"] = None
    if prep["vel"] is not None:
        vt, sp = prep["vel"]
        vi = np.interp(vt, est["t"], est["v"], left=np.nan, right=np.nan)
        dtv = np.r_[1.0, np.diff(vt)]
        mm = np.isfinite(vi) & (dtv < 0.15)
        if mm.any():
            err = vi[mm] - sp[mm]
            out["v_rmse"] = float(np.sqrt(np.mean(err ** 2)))
    return out


def _replay(prep, events, init, stops, start_t: float) -> dict:
    od = Odometer(
        prep["s"], prep["grade"], prep["table"], prep["notches"], prep["v_edges"],
        stops, Params(), ring_len=prep["L"],
    )
    od.init(init["s0"], max(init["d0"], 0.5))
    out_t, out_s, out_v, out_slip = [], [], [], []
    started = False
    n_behind = 0
    last_stamp = None
    for t, kind, val in events:
        if not started:
            if t < start_t:
                continue
            od.t = t
            started = True
        if last_stamp is not None and t < last_stamp:
            n_behind += 1
        else:
            last_stamp = t
        t_before = od.t
        if kind == "cmd":
            od.on_cmd(t, int(val))
        else:
            od.on_bogie(t, kind, float(val))
        if t_before is not None and t < t_before:
            continue
        ss, vv, _kk, _sg = od.state()
        out_t.append(t)
        out_s.append(ss)
        out_v.append(vv)
        out_slip.append(bool(od.slip))
    return {
        "t": np.array(out_t, float), "s": np.array(out_s, float), "v": np.array(out_v, float),
        "slip_t": np.array(out_t, float), "slip": np.array(out_slip, bool),
        "n_anchor": od.n_anchor, "n_behind": n_behind,
    }


def run_case(prep, case: Case) -> dict:
    bag = prep["bag"]
    if case.kind == "after_notch":
        t_on = notch_time(prep["cmd"], prep["front"][0], prep["front"][1])
        if t_on is None:
            return _blank(bag, case, False, True)
    elif case.localized:
        need = (case.dur if case.dur > 0.0 else 0.0) + 5.0
        t_on = pick_moving_time(prep["front"][0], prep["front"][1], need)
    else:
        t_on = prep["t0"]
    front, rear, applied = apply_streams(case, prep["front"], prep["rear"], t_on)
    if not applied:
        return _blank(bag, case, False, True)
    gap = case.dur if case.kind == "gap" else 0.0
    ev = events_of(front, rear, prep["cmd"], t_on, gap)
    keep_order = False
    if case.kind == "reorder":
        ev, applied = reorder_events(ev, t_on, case.dur)
        keep_order = True
    elif case.kind == "regress":
        ev, applied = regress_events(ev, t_on)
        keep_order = True
    if not applied:
        return _blank(bag, case, False, True)
    if not keep_order:
        ev.sort(key=lambda e: e[0])
    master, rover = prep["master"], prep["rover"]
    early = 0.0
    if case.kind == "map_last_fix_off_ring":
        master = np.array(prep["master"], copy=True)
        tail = np.argsort(master[:, 1])[-5:]
        master[tail] = offset_east(master[tail], 50.0)
    if case.group == "gnss":
        master, rover, early, applied = mutate_gnss(case.kind, master, rover, prep["window"])
        if not applied:
            return _blank(bag, case, False, True)
    init = init_arrays(master, rover, prep["window"], prep["ring"], prep["lat0"], prep["lon0"])
    stops = [dict(st) for st in prep["stops"]]
    start_t = prep["t0"]
    if case.kind == "gnss_late_vs_wheels" and init is not None:
        start_t = init["t0"] - early
    if case.kind == "map_start_moving":
        t_move = moving_start(prep["master"], prep["front"][0], prep["front"][1])
        if t_move is None:
            return _blank(bag, case, False, True)
        master = prep["master"][prep["master"][:, 1] >= t_move]
        rover = None if prep["rover"] is None else prep["rover"][prep["rover"][:, 1] >= t_move]
        init = init_arrays(master, rover, prep["window"], prep["ring"], prep["lat0"], prep["lon0"])
        start_t = t_move
    if case.kind == "map_shared_terminus":
        fits = branch_fits(prep["master"], prep["rover"], prep["window"], prep["cl"], prep["lat0"], prep["lon0"])
        if len(fits) < 2:
            return _blank(bag, case, False, True)
        worst = min(fits, key=lambda h: h["remaining"])
        s_worst = worst["s0"] if worst["key"] == "SK" else prep["off_ks"] + worst["s0"]
        if abs(_ring_delta(s_worst, prep["s0"], prep["L"])) < 1.0:
            return _blank(bag, case, False, True)
        init = dict(init)
        init["s0"] = float(s_worst)
        init["d0"] = abs(_ring_delta(s_worst, prep["s0"], prep["L"]))
    if case.kind == "map_wrong_branch" and init is not None:
        x = np.interp(prep["s0"], prep["s"], prep["ring"]["x"])
        y = np.interp(prep["s0"], prep["s"], prep["ring"]["y"])
        d = np.hypot(prep["ring"]["x"] - x, prep["ring"]["y"] - y)
        ds = np.array([_ring_delta(float(sv), prep["s0"], prep["L"]) for sv in prep["s"]])
        cand = np.flatnonzero((d < 25.0) & (np.abs(ds) > 50.0))
        if len(cand) == 0:
            return _blank(bag, case, False, True)
        j = int(cand[np.argmin(d[cand])])
        init = dict(init)
        init["s0"] = float(prep["s"][j])
        init["d0"] = float(d[j])
    if case.kind == "map_stop_missed" and init is not None:
        fwd = np.array([ (st["s"] - prep["s0"]) % prep["L"] for st in stops ])
        near = np.flatnonzero((fwd > 1.0) & (fwd < 2000.0))
        if len(near) == 0:
            return _blank(bag, case, False, True)
        drop = int(near[np.argmin(fwd[near])])
        stops = [st for i, st in enumerate(stops) if i != drop]
    if case.kind == "map_stop_extra" and init is not None:
        stops.append({"s": float((prep["s0"] + 150.0) % prep["L"]), "sd": 0.5, "n": 10})
    if case.kind == "map_stop_at_light" and init is not None:
        sm = dwell_s(prep["front"][0], prep["front"][1], prep["s0"], prep["t0"], stops, prep["L"])
        if sm is None:
            return _blank(bag, case, False, True)
        stops.append({"s": sm, "sd": 0.5, "n": 10})
    if init is None:
        return _blank(bag, case, True, False)
    played = _replay(prep, ev, init, stops, start_t)
    if len(played["t"]) < 10:
        return _blank(bag, case, True, True, n_behind=played["n_behind"])
    row = _blank(bag, case, True, True, n_behind=played["n_behind"],
                 s0_delta=float(init["s0"] - prep["s0"]),
                 start_s=float(t_on - float(prep["front"][0][0])))
    est = {"t": played["t"], "s": played["s"], "v": played["v"], "n_anchor": played["n_anchor"]}
    row.update(_score(prep, est, (played["slip_t"], played["slip"]), t_on, case.dur, case.localized))
    row["started"] = True
    row["applied"] = True
    return row


def prepare(path: Path, cl, model, stops, window: float):
    z = np.load(path)
    if MFIX not in z.files or len(z[MFIX]) < 20 or FRONT not in z.files or REAR not in z.files:
        return None
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    ring = build_ring(cl, stops)
    master = np.array(z[MFIX], copy=True)
    rover = np.array(z[RFIX], copy=True) if RFIX in z.files and len(z[RFIX]) else None
    init = init_arrays(master, rover, window, ring, lat0, lon0)
    if init is None:
        return None
    g = master[np.argsort(master[:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    s_ref, dist = track_reference(ring, x, y, init["s0"], t=g[:, 1])
    on_track = (dist < 3.0) & (g[:, 1] > init["t0"] + window)
    if int(on_track.sum()) < 100:
        return None
    front = z[FRONT][np.argsort(z[FRONT][:, 1])]
    rear = z[REAR][np.argsort(z[REAR][:, 1])]
    cmd = z[CMD][np.argsort(z[CMD][:, 1])] if CMD in z.files and len(z[CMD]) else np.zeros((0, 3))
    vel = None
    if MVEL in z.files and len(z[MVEL]) > 50:
        mv = z[MVEL][np.argsort(z[MVEL][:, 1])]
        vel = (mv[:, 1], np.hypot(mv[:, 2], mv[:, 3]))
    return {
        "bag": path.stem, "front": (front[:, 1], front[:, 2]), "rear": (rear[:, 1], rear[:, 2]),
        "cmd": cmd, "master": master, "rover": rover, "ring": ring, "cl": cl,
        "lat0": lat0, "lon0": lon0, "window": window, "t0": init["t0"], "s0": init["s0"],
        "stops": ring["stops"], "off_ks": ring["off_ks"], "L": ring["L"],
        "s": ring["s"], "grade": grade_of(ring["s"], ring["h"]),
        "table": model["table"], "notches": model["notches"], "v_edges": model["v_edges"],
        "tr": g[:, 1], "s_ref": s_ref, "on_track": on_track, "vel": vel,
    }


def _init(payload: dict) -> None:
    CTX.clear()
    CTX["cl"] = np.load(payload["map"])
    CTX["model"] = dict(np.load(Path(payload["model"]) / "model.npz"))
    CTX["stops"] = json.loads((Path(payload["model"]) / "stops.json").read_text(encoding="utf-8"))
    CTX["org"] = payload["org"]
    CTX["window"] = payload["window"]
    CTX["cases"] = list(payload["cases"])


def one_bag(name: str) -> list[dict]:
    path = Path(CTX["org"]) / f"{name}.npz"
    prep = prepare(path, CTX["cl"], CTX["model"], CTX["stops"], CTX["window"])
    rows = []
    for case in CTX["cases"]:
        if prep is None:
            row = _blank(name, case, False, False)
            row["case"] = case.id
            rows.append(row)
            continue
        try:
            row = run_case(prep, case)
        except Exception as exc:  # one attack must not drop the rest of the bag
            row = _blank(name, case, False, False, error=f"{type(exc).__name__}: {exc}")
        row["case"] = case.id
        rows.append(row)
    return rows


def _med(vals) -> float | None:
    a = [float(v) for v in vals if v is not None and np.isfinite(v)]
    if not a:
        return None
    return float(np.median(np.array(a, float)))


def summarize(case: Case, rows: list[dict]) -> dict:
    used = [r for r in rows if r.get("applied") and r.get("started") and "error" not in r]
    flags = [r["slip_flag"] for r in used if r.get("slip_flag") is not None]
    return {
        "group": case.group,
        "kind": case.kind,
        "which": case.which,
        "value": case.value,
        "dur": case.dur,
        "changes_filter": case.changes_filter,
        "localized": case.localized,
        "n": len(rows),
        "n_applied": int(sum(bool(r.get("applied")) for r in rows)),
        "n_started": int(sum(bool(r.get("started")) for r in rows)),
        "n_error": int(sum("error" in r for r in rows)),
        "all_finite": bool(used) and all(r.get("finite") for r in used),
        "along_rmse_med": _med([r.get("along_rmse") for r in used]),
        "end_pct_med": _med([r.get("end_pct") for r in used]),
        "v_rmse_med": _med([r.get("v_rmse") for r in used]),
        "during_max_med": _med([r.get("along_during_max") for r in used]),
        "after_max_med": _med([r.get("along_after_max") for r in used]),
        "slip_flag_frac": (float(np.mean(flags)) if flags else None),
        "detect_latency_med": _med([r.get("detect_latency_s") for r in used]),
        "recovery_med": _med([r.get("recovery_s") for r in used]),
        "slip_edges_med": _med([r.get("slip_edges") for r in used]),
        "n_behind_med": _med([r.get("n_behind") for r in used]),
        "n_anchor_med": _med([r.get("n_anchor") for r in used]),
        "s0_delta_med": _med([r.get("s0_delta") for r in used]),
        "start_s_med": _med([r.get("start_s") for r in used]),
    }


def _selected(ids: list[str] | None, groups: list[str] | None) -> list[Case]:
    out = CASES
    if groups:
        want = set(groups)
        out = [c for c in out if c.group in want]
    if ids:
        want = set(ids)
        missing = want.difference(c.id for c in CASES)
        if missing:
            raise SystemExit(f"unknown case: {', '.join(sorted(missing))}")
        out = [c for c in out if c.id in want]
    return out


GRID_MAGNITUDES = (0.90, 0.95, 0.97, 0.99, 1.01, 1.03, 1.05, 1.10)
GRID_DURATIONS = (5.0, 15.0, 30.0, 0.0)


def scale_grid(which: str = "rear") -> list[Case]:
    """Magnitude by duration for one bogie. Duration 0 runs to the end of the ride."""
    out = []
    for mag in GRID_MAGNITUDES:
        for dur in GRID_DURATIONS:
            tail = "rest" if dur <= 0.0 else f"{dur:g}s"
            out.append(Case(
                f"grid_{which}_scale_{mag:.2f}_{tail}", "single", "scale", which, mag, dur,
            ))
    return out


def _num(value, digits: int = 4):
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(number):
        return None
    return round(number, digits)


def head_commit(root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def dataset_version(splits: Path, split: str, map_path: Path) -> str:
    names = json.loads(Path(splits).read_text(encoding="utf-8"))[split]
    digest = hashlib.sha256("\n".join(names).encode()).hexdigest()[:12]
    return f"{split}:{Path(map_path).name}:{digest}"


def to_record(case: Case, summary: dict, commit: str, dataset: str) -> dict:
    """One jury record. pass means every applied bag started and stayed finite."""
    started = int(summary.get("n_started") or 0)
    applied = int(summary.get("n_applied") or 0)
    passed = bool(summary.get("all_finite")) and started > 0 and started == applied and int(summary.get("n_error") or 0) == 0
    return {
        "commit": commit,
        "dataset_version": dataset,
        "fault": {
            "type": f"{case.which}_{case.kind}" if case.which else case.kind,
            "magnitude": case.value if case.value else None,
            "start_s": _num(summary.get("start_s_med"), 3),
            "duration_s": "rest_of_run" if case.dur <= 0.0 else case.dur,
        },
        "metrics": {
            "along_rmse_m": _num(summary.get("along_rmse_med"), 3),
            "speed_rmse_mps": _num(summary.get("v_rmse_med"), 4),
            "detection_rate": _num(summary.get("slip_flag_frac"), 4),
            "detection_latency_s": _num(summary.get("detect_latency_med"), 3),
            "max_error_m": _num(summary.get("during_max_med"), 3),
            "recovery_s": _num(summary.get("recovery_med"), 3),
        },
        "pass": passed,
        "pass_rule": "every applied bag started and the state stayed finite",
        "n": started,
    }


def _isolines(grid: np.ndarray, level: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Segments in index space. grid[row, col] sits on the cell centre."""
    ny, nx = grid.shape
    segs = []
    for y in range(ny - 1):
        for x in range(nx - 1):
            corners = [grid[y, x], grid[y, x + 1], grid[y + 1, x + 1], grid[y + 1, x]]
            if not np.all(np.isfinite(corners)):
                continue
            pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
            hits = []
            for i in range(4):
                v0, v1 = float(corners[i]), float(corners[(i + 1) % 4])
                if (v0 - level) * (v1 - level) >= 0.0:
                    continue
                t = (level - v0) / (v1 - v0)
                p0, p1 = pts[i], pts[(i + 1) % 4]
                hits.append((x + p0[0] + t * (p1[0] - p0[0]), y + p0[1] + t * (p1[1] - p0[1])))
            if len(hits) >= 2:
                segs.append((hits[0], hits[1]))
    return segs


def _rgb(t: float) -> str:
    t = min(1.0, max(0.0, t))
    stops = ((0.0, (29, 78, 137)), (0.5, (242, 193, 78)), (1.0, (192, 57, 43)))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            rgb = tuple(int(a + u * (b - a)) for a, b in zip(c0, c1))
            return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"
    return "#c0392b"


def heatmap_svg(records: list[dict]) -> str | None:
    """X is scale, Y is duration, colour is along RMSE, lines are detection rate."""
    cells = []
    for rec in records:
        fault = rec["fault"]
        if fault.get("type") != "rear_scale":
            continue
        mag = fault.get("magnitude")
        dur = fault.get("duration_s")
        rmse = rec["metrics"].get("along_rmse_m")
        rate = rec["metrics"].get("detection_rate")
        if mag is None or rmse is None or rate is None:
            continue
        cells.append((float(mag), dur, float(rmse), float(rate)))
    mags = sorted({c[0] for c in cells})
    durs = []
    for dur in GRID_DURATIONS:
        key = "rest_of_run" if dur <= 0.0 else dur
        if any(c[1] == key or c[1] == dur for c in cells):
            durs.append(key)
    if len(mags) < 2 or len(durs) < 2:
        return None
    lookup = {(c[0], c[1]): c for c in cells}

    nx, ny = len(mags), len(durs)
    rmse = np.full((ny, nx), np.nan)
    rate = np.full((ny, nx), np.nan)
    for iy, dur in enumerate(durs):
        for ix, mag in enumerate(mags):
            got = lookup.get((mag, dur))
            if got is None:
                continue
            rmse[iy, ix] = got[2]
            rate[iy, ix] = got[3]
    positive = rmse[np.isfinite(rmse) & (rmse > 0)]
    if len(positive) == 0:
        return None
    lo, hi = float(np.min(positive)), float(np.max(positive))
    if hi == lo:
        hi = lo + 1.0
    width, height = 760, 460
    left, right, top, bottom = 88, 150, 36, 64
    plot_w, plot_h = width - left - right, height - top - bottom
    cw, ch = plot_w / nx, plot_h / ny
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7f4ee"/>',
        '<text x="88" y="24" font-family="sans-serif" font-size="16" fill="#1c1917">Задняя тележка, масштаб</text>',
    ]
    for iy, dur in enumerate(durs):
        for ix, mag in enumerate(mags):
            value = rmse[iy, ix]
            if not np.isfinite(value) or value <= 0:
                colour = "#d6d3d1"
            else:
                colour = _rgb((np.log10(value) - np.log10(lo)) / (np.log10(hi) - np.log10(lo)))
            x = left + ix * cw
            y = top + (ny - 1 - iy) * ch
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{cw:.1f}" height="{ch:.1f}" fill="{colour}" stroke="#f7f4ee" stroke-width="2"/>')
    for level in (0.25, 0.5, 0.75):
        if not (np.nanmin(rate) <= level <= np.nanmax(rate)):
            continue
        for (x0, y0), (x1, y1) in _isolines(rate, level):
            px0 = left + (x0 + 0.5) * cw
            py0 = top + plot_h - (y0 + 0.5) * ch
            px1 = left + (x1 + 0.5) * cw
            py1 = top + plot_h - (y1 + 0.5) * ch
            parts.append(
                f'<line x1="{px0:.1f}" y1="{py0:.1f}" x2="{px1:.1f}" y2="{py1:.1f}" '
                f'stroke="#1c1917" stroke-width="1.6" fill="none"/>'
            )
    for ix, mag in enumerate(mags):
        x = left + (ix + 0.5) * cw
        parts.append(
            f'<text x="{x:.1f}" y="{height - 28}" text-anchor="middle" font-family="sans-serif" '
            f'font-size="11" fill="#1c1917">{mag:.2f}</text>'
        )
    parts.append(
        f'<text x="{left + plot_w / 2:.1f}" y="{height - 8}" text-anchor="middle" '
        f'font-family="sans-serif" font-size="12" fill="#1c1917">масштаб</text>'
    )
    for iy, dur in enumerate(durs):
        y = top + plot_h - (iy + 0.5) * ch
        label = "весь рейс" if dur == "rest_of_run" else f"{dur:g} с"
        parts.append(
            f'<text x="{left - 8}" y="{y + 4:.1f}" text-anchor="end" font-family="sans-serif" '
            f'font-size="11" fill="#1c1917">{label}</text>'
        )
    parts.append(
        f'<text x="16" y="{top + plot_h / 2:.1f}" text-anchor="middle" font-family="sans-serif" '
        f'font-size="12" fill="#1c1917" transform="rotate(-90 16 {top + plot_h / 2:.1f})">длительность</text>'
    )
    bar_x, bar_y, bar_w, bar_h = width - 120, top, 16, plot_h
    for k in range(40):
        t = k / 39
        y = bar_y + bar_h - (k + 1) * bar_h / 40
        parts.append(f'<rect x="{bar_x}" y="{y:.1f}" width="{bar_w}" height="{bar_h / 40 + 0.5:.1f}" fill="{_rgb(t)}"/>')
    parts.append(
        f'<text x="{bar_x + 22}" y="{bar_y + 12}" font-family="sans-serif" font-size="11" fill="#1c1917">{hi:.0f} м</text>'
    )
    parts.append(
        f'<text x="{bar_x + 22}" y="{bar_y + bar_h}" font-family="sans-serif" font-size="11" fill="#1c1917">{lo:.1f} м</text>'
    )
    parts.append(
        f'<text x="{bar_x}" y="{bar_y + bar_h + 28}" font-family="sans-serif" font-size="11" fill="#1c1917">цвет — RMSE вдоль пути, лог</text>'
    )
    parts.append(
        f'<text x="{left}" y="{height - 48}" font-family="sans-serif" font-size="11" fill="#44403c">'
        f'линии — вероятность обнаружения 0.25, 0.50, 0.75</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run the fault campaign on the Python twin.")
    ap.add_argument("--org", default="local/org")
    ap.add_argument("--splits", default="local/splits.json")
    ap.add_argument("--map", default="local/map/july27_arc.npz")
    ap.add_argument("--model", default="local/model_arc")
    ap.add_argument("--split", default="val")
    ap.add_argument("--window", type=float, default=3.0)
    ap.add_argument("--cases", nargs="*", default=None)
    ap.add_argument("--group", nargs="*", default=None)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--grid", choices=("rear", "front"), default=None,
                    help="magnitude by duration for one bogie, instead of the named catalog")
    ap.add_argument("--report", type=Path, default=None,
                    help="directory for fault_report.json and fault_heatmap.svg")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    chosen = scale_grid(args.grid) if args.grid else _selected(args.cases, args.group)
    if args.list:
        for case in chosen:
            print(f"{case.group:8} {case.id}")
        print(f"{len(chosen)} cases")
        return 0
    if args.out is None:
        ap.error("--out is required unless --list")
    names = json.loads(Path(args.splits).read_text(encoding="utf-8"))[args.split]
    if args.limit > 0:
        names = names[: args.limit]
    payload = {
        "org": args.org, "map": args.map, "model": args.model, "window": args.window,
        "cases": chosen,
    }
    bag_rows: list[list[dict]] = []
    if args.jobs == 1:
        _init(payload)
        for name in names:
            bag_rows.append(one_bag(name))
            print(name, flush=True)
    else:
        with ProcessPoolExecutor(initializer=_init, initargs=(payload,)) as ex:
            bag_rows = list(ex.map(one_bag, names))
    by_case = {c.id: [] for c in chosen}
    for rows in bag_rows:
        for row in rows:
            by_case[row["case"]].append(row)
    table = {c.id: summarize(c, by_case[c.id]) for c in chosen}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(table, indent=1), encoding="utf-8")
    root = Path(__file__).resolve().parents[2]
    commit = head_commit(root)
    dataset = dataset_version(Path(args.splits), args.split, Path(args.map))
    records = [to_record(case, table[case.id], commit, dataset) for case in chosen]
    if args.report is not None:
        args.report.mkdir(parents=True, exist_ok=True)
        (args.report / "fault_report.json").write_text(json.dumps(records, indent=1, ensure_ascii=False), encoding="utf-8")
        svg = heatmap_svg(records)
        if svg is not None:
            (args.report / "fault_heatmap.svg").write_text(svg, encoding="utf-8")
    for case in chosen:
        t = table[case.id]
        along = t["along_rmse_med"]
        along_s = f"{along:7.2f}" if along is not None else "   none"
        print(
            f"{case.id:32} n={t['n_started']:2d}/{t['n']:<2d} applied={t['n_applied']:<2d} "
            f"finite={t['all_finite']} along={along_s} reach={case.changes_filter}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

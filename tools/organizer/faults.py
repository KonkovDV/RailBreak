"""Fault injection on bogie speed streams (km/h, as recorded).

Each fault acts on (t, u) arrays of one bogie and returns new arrays. Times
are header stamps; `t_mid` is where the fault starts (seconds after the first
sample of the run, chosen on a moving stretch by `pick_moving_time`).
Dropouts remove samples (the node must bridge with the model); NaN keeps the
message but poisons the value.
"""

from __future__ import annotations

import numpy as np

FAULTS = {
    "none": "no change",
    "drop_front_20s": "front bogie silent 20 s",
    "drop_both_5s": "both bogies silent 5 s",
    "drop_both_20s": "both bogies silent 20 s",
    "freeze_rear_30s": "rear bogie repeats its last value 30 s",
    "spike_front": "front bogie +40 km/h single-sample spikes every 5 s for 60 s",
    "noise_x5": "both bogies Gaussian noise sigma 2 km/h",
    "slip_front_10pct": "front bogie reads +10 % for 30 s (wheel spin under traction)",
    "slide_both_20pct": "both bogies read -20 % for 8 s (wheel slide under braking)",
    "nan_front_10s": "front bogie NaN for 10 s",
    "scale_rear_5pct": "rear bogie reads +5 % for the whole run",
}


def pick_moving_time(t: np.ndarray, u_kmh: np.ndarray, dur: float, min_kmh: float = 20.0) -> float:
    """First time after 25 % of the run where speed stays above min_kmh for dur."""
    start = t[0] + 0.25 * (t[-1] - t[0])
    fast = u_kmh > min_kmh
    for i in np.flatnonzero((t >= start) & fast):
        j = np.searchsorted(t, t[i] + dur)
        if j < len(t) and fast[i:j].all():
            return float(t[i])
    return float(start)


def apply(name: str, which: str, t: np.ndarray, u: np.ndarray, t_on: float, rng=None):
    """Return (t, u) for one bogie ('front' or 'rear') under fault `name`."""
    rng = rng or np.random.default_rng(0)
    t = t.copy()
    u = u.astype(float).copy()
    if name == "none":
        return t, u
    if name == "drop_front_20s" and which == "front":
        keep = (t < t_on) | (t > t_on + 20.0)
        return t[keep], u[keep]
    if name in ("drop_both_5s", "drop_both_20s"):
        d = 5.0 if name.endswith("5s") else 20.0
        keep = (t < t_on) | (t > t_on + d)
        return t[keep], u[keep]
    if name == "freeze_rear_30s" and which == "rear":
        m = (t >= t_on) & (t <= t_on + 30.0)
        if m.any():
            u[m] = u[np.flatnonzero(m)[0]]
        return t, u
    if name == "spike_front" and which == "front":
        for k in range(12):
            i = np.searchsorted(t, t_on + 5.0 * k)
            if i < len(u):
                u[i] += 40.0
        return t, u
    if name == "noise_x5":
        return t, u + rng.normal(0.0, 2.0, len(u))
    if name == "slip_front_10pct" and which == "front":
        m = (t >= t_on) & (t <= t_on + 30.0)
        u[m] *= 1.10
        return t, u
    if name == "slide_both_20pct":
        m = (t >= t_on) & (t <= t_on + 8.0)
        u[m] *= 0.80
        return t, u
    if name == "nan_front_10s" and which == "front":
        m = (t >= t_on) & (t <= t_on + 10.0)
        u[m] = np.nan
        return t, u
    if name == "scale_rear_5pct" and which == "rear":
        return t, u * 1.05
    return t, u


def fault_window(name: str) -> float:
    return {
        "drop_front_20s": 20.0, "drop_both_5s": 5.0, "drop_both_20s": 20.0,
        "freeze_rear_30s": 30.0, "spike_front": 60.0, "slip_front_10pct": 30.0,
        "slide_both_20pct": 8.0, "nan_front_10s": 10.0,
    }.get(name, 0.0)

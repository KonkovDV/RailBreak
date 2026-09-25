"""GNSS reference track. Offline only: never an estimator input."""

from __future__ import annotations

from pathlib import Path

import numpy as np

R_EARTH = 6_378_137.0
FIX = "sensing_gnss_master_fix"
VEL = "sensing_gnss_master_vel"


def enu(lat_deg: np.ndarray, lon_deg: np.ndarray, lat0: float, lon0: float):
    x = np.radians(lon_deg - lon0) * R_EARTH * np.cos(np.radians(lat0))
    y = np.radians(lat_deg - lat0) * R_EARTH
    return x, y


def master_reference(z: np.lib.npyio.NpzFile, *, min_status: int = 0) -> dict | None:
    """ENU metres from the first master fix. z is metres above that altitude.

    Columns of the extractor: time_bag, stamp, lat, lon, alt, status, ...
    """
    if FIX not in z.files or len(z[FIX]) < 2:
        return None
    g = z[FIX]
    order = np.argsort(g[:, 1])
    g = g[order]
    keep = g[:, 5] >= min_status
    if keep.sum() < 2:
        return None
    g = g[keep]
    lat0, lon0, alt0 = float(g[0, 2]), float(g[0, 3]), float(g[0, 4])
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    speed = np.full(len(g), np.nan)
    if VEL in z.files and len(z[VEL]) > 2:
        v = z[VEL]
        vo = np.argsort(v[:, 1])
        sp = np.hypot(v[vo, 2], v[vo, 3])
        speed = np.interp(g[:, 1], v[vo, 1], sp, left=np.nan, right=np.nan)
    return {
        "t": g[:, 1],
        "x": x,
        "y": y,
        "z": g[:, 4] - alt0,
        "v": speed,
        "status": g[:, 5],
        "origin": (lat0, lon0, alt0),
    }


def load_reference(npz: Path, **kwargs) -> dict | None:
    with np.load(npz) as z:
        return master_reference(z, **kwargs)

"""Pair an estimate to a reference the way the README describes the jury.

Nearest header stamp, tolerance 0.05 s. One reference sample takes at most
one estimate. RMSE is over paired samples only; coverage is paired/reference.
"""

from __future__ import annotations

import numpy as np

TOL_S = 0.05


def pair(t_est: np.ndarray, t_ref: np.ndarray, tol_s: float = TOL_S) -> np.ndarray:
    """Index of the estimate for each reference sample, or -1."""
    if len(t_est) == 0 or len(t_ref) == 0:
        return np.full(len(t_ref), -1, dtype=int)
    order = np.argsort(t_est)
    te = t_est[order]
    pos = np.searchsorted(te, t_ref)
    out = np.full(len(t_ref), -1, dtype=int)
    for i, p in enumerate(pos):
        cands = []
        if p < len(te):
            cands.append(p)
        if p > 0:
            cands.append(p - 1)
        best = min(cands, key=lambda j: abs(te[j] - t_ref[i]))
        if abs(te[best] - t_ref[i]) <= tol_s:
            out[i] = order[best]
    return out


def score(est: dict, ref: dict, *, tol_s: float = TOL_S) -> dict:
    idx = pair(np.asarray(est["t"], float), np.asarray(ref["t"], float), tol_s)
    m = idx >= 0
    n_ref = int(len(ref["t"]))
    n_pair = int(m.sum())
    out = {
        "n_ref": n_ref,
        "n_est": int(len(est["t"])),
        "n_pair": n_pair,
        "coverage": float(n_pair / n_ref) if n_ref else float("nan"),
    }
    if n_pair == 0:
        for k in ("rmse_x", "rmse_y", "rmse_z", "rmse_3d", "rmse_v"):
            out[k] = float("nan")
        return out
    ex = np.asarray(est["x"], float)[idx[m]]
    ey = np.asarray(est["y"], float)[idx[m]]
    ez = np.asarray(est["z"], float)[idx[m]]
    dx = ex - np.asarray(ref["x"], float)[m]
    dy = ey - np.asarray(ref["y"], float)[m]
    dz = ez - np.asarray(ref["z"], float)[m]
    out["rmse_x"] = float(np.sqrt(np.mean(dx * dx)))
    out["rmse_y"] = float(np.sqrt(np.mean(dy * dy)))
    out["rmse_z"] = float(np.sqrt(np.mean(dz * dz)))
    out["rmse_3d"] = float(np.sqrt(np.mean(dx * dx + dy * dy + dz * dz)))
    ev = np.asarray(est["v"], float)[idx[m]]
    rv = np.asarray(ref["v"], float)[m]
    ok = np.isfinite(ev) & np.isfinite(rv)
    out["rmse_v"] = float(np.sqrt(np.mean((ev[ok] - rv[ok]) ** 2))) if ok.any() else float("nan")
    return out

"""Pair an estimate to a reference the way the README describes the jury.

Nearest header stamp, tolerance 0.05 s. One reference sample takes at most
one estimate. There is no mask of an already used estimate, so the converse
is false: one output can cover several references. At 5 Hz out and 20 Hz
reference the coverage is 54 % (108/200), not the 25 % rate ratio, and one
sample is used up to three times. A 10 m error is then RMSE 1.41 m against a
matched 5 Hz reference and 1.67 m against the 20 Hz reference; two extra
references 40 ms from that sample raise it to 2.40 m. Pairing is left as it
is, so the published rides do not move.
"""

from __future__ import annotations

import numpy as np

TOL_S = 0.05


def pair(t_est: np.ndarray, t_ref: np.ndarray, tol_s: float = TOL_S) -> np.ndarray:
    """Index of the estimate for each reference sample, or -1.

    This is the published matcher. One estimate may be chosen for several
    references. pair_one_to_one is the diagnostic alternative.
    """
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


def pair_one_to_one(t_est: np.ndarray, t_ref: np.ndarray, tol_s: float = TOL_S) -> np.ndarray:
    """Index of the estimate for each reference sample, or -1.

    Monotonic one-to-one match. References and estimates are walked in time
    order. Each estimate is used at most once, and a later reference cannot
    take an earlier estimate than the one already matched. The estimate taken
    for a reference is the earliest one still inside the tolerance. This does
    not replace pair().
    """
    t_est = np.asarray(t_est, float)
    t_ref = np.asarray(t_ref, float)
    out = np.full(len(t_ref), -1, dtype=int)
    if len(t_est) == 0 or len(t_ref) == 0:
        return out
    order_e = np.argsort(t_est, kind="mergesort")
    order_r = np.argsort(t_ref, kind="mergesort")
    te = t_est[order_e]
    e = 0
    n_e = len(te)
    for r_i in order_r:
        t = t_ref[r_i]
        while e < n_e and te[e] < t - tol_s:
            e += 1
        if e < n_e and te[e] <= t + tol_s:
            out[r_i] = int(order_e[e])
            e += 1
    return out


def score(est: dict, ref: dict, *, tol_s: float = TOL_S, pair_fn=None) -> dict:
    match = pair if pair_fn is None else pair_fn
    idx = match(np.asarray(est["t"], float), np.asarray(ref["t"], float), tol_s)
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


def compare_pairings(est: dict, ref: dict, *, tol_s: float = TOL_S) -> dict:
    """Legacy nearest coverage beside a monotonic one-to-one coverage.

    The legacy fields are score() with pair(). They are not a replacement
    for a previously published rmse_3d.
    """
    legacy = score(est, ref, tol_s=tol_s)
    oto = score(est, ref, tol_s=tol_s, pair_fn=pair_one_to_one)
    return {
        "rmse_3d_nearest_legacy": legacy["rmse_3d"],
        "rmse_3d_one_to_one": oto["rmse_3d"],
        "coverage_nearest_legacy": legacy["coverage"],
        "coverage_one_to_one": oto["coverage"],
    }

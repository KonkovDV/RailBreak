"""Fit the empirical along-track bound on the train split.

Val is the independent check. Refitting q on val would cover that split by
construction and is not a calibration. The statement on the written file is
"empirical bound, not certified protection level".

q_0.99 is the pooled 99th percentile of |e_s| / sigma_s on clean train fixes
whose shadow status is NOMINAL and sigma_s is at least 0.05 m. Validation is
scored with those coefficients and is not used to choose them. The hidden
test split is not read.

B_mode for a degraded status is the 99th percentile of max(0, |e_s| - q*sigma)
on clean train fixes in that status, plus the same percentile on a whole-run
rear scale of +5 % (train only). NOMINAL stays 0. Scale samples the monitor
still calls NOMINAL are counted in the notes and are not folded into q.

B_time is the 99th percentile of the remaining excess on train fixes from a
5 s both-bogie dropout whose newer bogie is older than the stale gate.
B_map is 0: the scored reference is the ring itself, so a map term is not
separated.
"""

from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
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
import faults  # noqa: E402
from integrity import STATUSES, IntegrityMonitor, coeff_path, obs_from_odometer  # noqa: E402
from odometer import Odometer, Params  # noqa: E402
from reference import enu  # noqa: E402

CTX: dict = {}
SIGMA_FLOOR_M = 0.05
STALE_S = 0.35
HEADER = (
    Path(__file__).resolve().parents[2]
    / "railbreak_backup_odometry"
    / "include"
    / "railbreak_backup_odometry"
    / "integrity_bound.hpp"
)


def _init(payload: dict) -> None:
    CTX.clear()
    CTX["cl"] = np.load(payload["map"])
    CTX["model"] = dict(np.load(Path(payload["model"]) / "model.npz"))
    CTX["stops"] = json.loads((Path(payload["model"]) / "stops.json").read_text(encoding="utf-8"))
    CTX["org"] = payload["org"]
    CTX["window"] = payload["window"]


def _one(job: tuple[str, str]) -> dict | None:
    name, fault = job
    try:
        return _replay(Path(CTX["org"]) / f"{name}.npz", fault)
    except Exception as exc:  # one bag must not drop the split
        return {"error": f"{name} {fault}: {type(exc).__name__}: {exc}"}


def _replay(path: Path, fault: str) -> dict | None:
    z = np.load(path)
    if MFIX not in z.files or len(z[MFIX]) < 20 or FRONT not in z.files or REAR not in z.files:
        return None
    cl, model, stops = CTX["cl"], CTX["model"], CTX["stops"]
    lat0, lon0 = float(cl["lat0"]), float(cl["lon0"])
    ring = build_ring(cl, stops)
    init = ring_init(z, CTX["window"], ring, lat0, lon0)
    if init is None:
        return None
    od = Odometer(
        ring["s"], grade_of(ring["s"], ring["h"]), model["table"], model["notches"],
        model["v_edges"], ring["stops"], Params(), ring_len=ring["L"],
    )
    od.init(init["s0"], max(init["d0"], 0.5))
    streams = {}
    for key, kind in ((FRONT, "front"), (REAR, "rear")):
        a = z[key][np.argsort(z[key][:, 1])]
        streams[kind] = (a[:, 1], a[:, 2])
    if fault != "none":
        tf, uf = streams["front"]
        t_on = faults.pick_moving_time(tf, uf, faults.fault_window(fault) + 5.0)
        for kind in ("front", "rear"):
            streams[kind] = faults.apply(fault, kind, *streams[kind], t_on)
    ev = []
    for kind, (tt, uu) in streams.items():
        for a_t, a_u in zip(tt, uu):
            ev.append((float(a_t), kind, float(a_u)))
    if CMD not in z.files:
        return None
    for row in z[CMD]:
        ev.append((float(row[1]), "cmd", float(row[2])))
    ev.sort(key=lambda item: item[0])
    mon = IntegrityMonitor()
    et, es, esig, estatus, egap = [], [], [], [], []
    started = False
    for t, kind, val in ev:
        if not started:
            if t < init["t0"]:
                continue
            od.t = t
            started = True
        regressed = od.t is not None and t < od.t
        if kind == "cmd":
            od.on_cmd(t, int(val))
        else:
            od.on_bogie(t, kind, val)
        if od.t is None:
            continue
        snap = obs_from_odometer(
            od, float(od.t), stamp_regressed=regressed, absolute_start=True, map_in_domain=True,
        )
        report = mon.update(snap)
        if report.status not in STATUSES:
            continue
        ss, _vv, _kk, sg = od.state()
        et.append(float(od.t))
        es.append(ss)
        esig.append(sg)
        estatus.append(STATUSES.index(report.status))
        egap.append(min(snap.front_age_s, snap.rear_age_s))
    if len(et) < 2:
        return None
    g = z[MFIX][np.argsort(z[MFIX][:, 1])]
    g = g[g[:, 5] == 2]
    if len(g) < 50:
        return None
    x, y = enu(g[:, 2], g[:, 3], lat0, lon0)
    s_ref, d = track_reference(ring, x, y, init["s0"], t=g[:, 1])
    tr = g[:, 1]
    est_t = np.asarray(et, float)
    su = np.unwrap(np.asarray(es, float) * 2 * np.pi / ring["L"]) * ring["L"] / (2 * np.pi)
    idx = np.searchsorted(est_t, tr, side="right") - 1
    m = (d < 3.0) & (idx >= 0) & (tr > init["t0"] + CTX["window"])
    if int(m.sum()) < 100:
        return None
    Lr = ring["L"]
    si = np.interp(tr, est_t, su, left=np.nan, right=np.nan)
    e = (si - s_ref + 0.5 * Lr) % Lr - 0.5 * Lr
    keep = m & np.isfinite(e) & np.isfinite(si)
    if int(keep.sum()) < 100:
        return None
    ii = idx[keep]
    return {
        "bag": path.stem,
        "e": np.abs(e[keep]).astype(np.float32),
        "sig": np.asarray(esig, np.float32)[ii],
        "status": np.asarray(estatus, np.int8)[ii],
        "gap": np.asarray(egap, np.float32)[ii],
    }


def _stack(rows: list[dict]) -> dict[str, np.ndarray]:
    return {
        "e": np.concatenate([r["e"] for r in rows]),
        "sig": np.concatenate([r["sig"] for r in rows]),
        "status": np.concatenate([r["status"] for r in rows]),
        "gap": np.concatenate([r["gap"] for r in rows]),
    }


def _q99(values: np.ndarray) -> float:
    if len(values) == 0:
        return 0.0
    return float(f"{float(np.quantile(values, 0.99)):.6f}")


def _run(names: list[str], fault: str, jobs: int) -> tuple[list[dict], list[str]]:
    jobs_in = [(n, fault) for n in names]
    rows, errors = [], []
    if jobs <= 1:
        for item in jobs_in:
            row = _one(item)
            if row is None:
                continue
            if "error" in row:
                errors.append(row["error"])
            else:
                rows.append(row)
        return rows, errors
    with ProcessPoolExecutor(max_workers=jobs, initializer=_init, initargs=(CTX["payload"],)) as ex:
        for row in ex.map(_one, jobs_in):
            if row is None:
                continue
            if "error" in row:
                errors.append(row["error"])
            else:
                rows.append(row)
    return rows, errors


def _header(q: float, b: dict[str, float], b_time: float, b_map: float) -> str:
    return f"""#pragma once

#include "railbreak_backup_odometry/integrity_monitor.hpp"

namespace railbreak {{

// Written by tools/organizer/calibrate_integrity.py from the train split.
// Empirical along-track integrity bound. Not a certified protection level.
inline BoundCoeff empirical_bound() {{
  BoundCoeff c;
  c.calibrated = true;
  c.q99 = {q:.6f};
  c.b_nominal = {b["NOMINAL"]:.6f};
  c.b_single = {b["DEGRADED_SINGLE_BOGIE"]:.6f};
  c.b_model = {b["DEGRADED_MODEL_CARRY"]:.6f};
  c.b_common = {b["DEGRADED_COMMON_MODE_UNOBSERVABLE"]:.6f};
  c.b_no_map = {b["DEGRADED_NO_MAP"]:.6f};
  c.b_relative = {b["DEGRADED_RELATIVE_ONLY"]:.6f};
  c.b_untrusted = {b["POSITION_UNTRUSTED"]:.6f};
  c.b_time_m = {b_time:.6f};
  c.b_map_m = {b_map:.6f};
  c.coverage = "0.99";
  c.split = "train";
  return c;
}}

}}  // namespace railbreak
"""


def _fraction(pack: dict[str, np.ndarray], q: float, b_mode: dict[str, float], b_time: float) -> float:
    status = pack["status"]
    margin = np.zeros(len(status), np.float64)
    for i, name in enumerate(STATUSES):
        margin[status == i] = b_mode[name]
    time = np.where(pack["gap"] > STALE_S, b_time, 0.0)
    bound = q * pack["sig"].astype(np.float64) + margin + time
    return float(np.mean(pack["e"].astype(np.float64) <= bound))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--org", type=Path, default=Path("local/org"))
    ap.add_argument("--splits", type=Path, default=Path("local/splits.json"))
    ap.add_argument("--map", type=Path, default=Path("local/map/july27_arc.npz"))
    ap.add_argument("--model", type=Path, default=Path("local/model_arc"))
    ap.add_argument("--window", type=float, default=3.0)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()
    splits = json.loads(args.splits.read_text(encoding="utf-8"))
    train = list(splits["train"])
    val = list(splits["val"])
    if args.limit:
        train = train[: args.limit]
        val = val[: args.limit]
    payload = {
        "map": str(args.map), "model": str(args.model), "org": str(args.org), "window": args.window,
    }
    _init(payload)
    CTX["payload"] = payload
    print(f"train {len(train)} val {len(val)} jobs {args.jobs}", flush=True)
    clean, err_c = _run(train, "none", args.jobs)
    print(f"clean rides {len(clean)} errors {len(err_c)}", flush=True)
    scale, err_s = _run(train, "scale_rear_5pct", args.jobs)
    print(f"scale rides {len(scale)} errors {len(err_s)}", flush=True)
    drop, err_d = _run(train, "drop_both_5s", args.jobs)
    print(f"dropout rides {len(drop)} errors {len(err_d)}", flush=True)
    val_rows, err_v = _run(val, "none", args.jobs)
    print(f"val rides {len(val_rows)} errors {len(err_v)}", flush=True)
    if len(clean) < 5:
        print("too few clean train rides")
        return 1
    cpack = _stack(clean)
    nominal = (cpack["status"] == STATUSES.index("NOMINAL")) & (cpack["sig"] >= SIGMA_FLOOR_M)
    if int(nominal.sum()) < 100:
        print("too few nominal train fixes")
        return 1
    ratio = cpack["e"][nominal].astype(np.float64) / cpack["sig"][nominal].astype(np.float64)
    q = _q99(ratio)
    spack = _stack(scale) if scale else None
    notes = {}
    b_mode = {name: 0.0 for name in STATUSES}
    counts = {}
    for i, name in enumerate(STATUSES):
        parts = [cpack["e"][cpack["status"] == i].astype(np.float64)]
        sigs = [cpack["sig"][cpack["status"] == i].astype(np.float64)]
        if name != "NOMINAL" and spack is not None:
            m = spack["status"] == i
            parts.append(spack["e"][m].astype(np.float64))
            sigs.append(spack["sig"][m].astype(np.float64))
        ee = np.concatenate(parts) if parts else np.zeros(0)
        ss = np.concatenate(sigs) if sigs else np.zeros(0)
        counts[name] = int(len(ee))
        if name == "NOMINAL" or len(ee) < 30:
            b_mode[name] = 0.0
            continue
        excess = np.maximum(0.0, ee - q * ss)
        b_mode[name] = _q99(excess)
    if spack is not None:
        missed = spack["status"] == STATUSES.index("NOMINAL")
        notes["scale_rear_5pct_nominal_fixes"] = int(missed.sum())
        notes["scale_rear_5pct_nominal_abs_e_p99_m"] = (
            _q99(spack["e"][missed].astype(np.float64)) if missed.any() else None
        )
    dpack = _stack(drop) if drop else None
    b_time = 0.0
    n_drop = 0
    if dpack is not None:
        stale = dpack["gap"] > STALE_S
        n_drop = int(stale.sum())
        if n_drop >= 30:
            margin = np.array([b_mode[STATUSES[i]] for i in dpack["status"][stale]], np.float64)
            excess = np.maximum(
                0.0,
                dpack["e"][stale].astype(np.float64) - q * dpack["sig"][stale].astype(np.float64) - margin,
            )
            b_time = _q99(excess)
    b_map = 0.0
    vpack = _stack(val_rows) if val_rows else None
    val_fraction = _fraction(vpack, q, b_mode, b_time) if vpack is not None else None
    per_ride = []
    for row in val_rows:
        per_ride.append(_fraction(
            {"e": row["e"], "sig": row["sig"], "status": row["status"], "gap": row["gap"]},
            q, b_mode, b_time,
        ))
    doc = {
        "name": "empirical along-track integrity bound",
        "name_ru": "экспериментальная граница ошибки",
        "certification_claim": False,
        "calibrated": True,
        "calibrated_coverage": "0.99",
        "calibration_split": "train",
        "q_0_99": q,
        "q_definition": (
            "pooled 99th percentile of |e_s|/sigma_s on clean train fixes with "
            "status NOMINAL and sigma_s >= 0.05 m"
        ),
        "sigma_floor_m": SIGMA_FLOOR_M,
        "b_mode_m": b_mode,
        "b_mode_counts": counts,
        "b_time_m": b_time,
        "b_time_stale_fixes": n_drop,
        "b_map_m": b_map,
        "b_map_identified": False,
        "formula": "q_0.99 * sigma_s + B_mode + B_time + B_map",
        "statement": "empirical bound, not certified protection level",
        "coefficient_fit_split": "train",
        "independent_check_split": "val",
        "n_train_rides": len(clean),
        "n_train_nominal_fixes": int(nominal.sum()),
        "n_val_rides": len(val_rows),
        "val_fraction_inside": None if val_fraction is None else float(f"{val_fraction:.6f}"),
        "val_fraction_inside_median_ride": (
            None if not per_ride else float(f"{float(np.median(per_ride)):.6f}")
        ),
        "notes": notes,
        "errors": err_c + err_s + err_d + err_v,
    }
    out = coeff_path()
    out.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    HEADER.write_text(_header(q, b_mode, b_time, b_map), encoding="utf-8", newline="\n")
    print(json.dumps({k: doc[k] for k in (
        "q_0_99", "b_mode_m", "b_time_m", "b_map_m", "n_train_rides",
        "n_train_nominal_fixes", "val_fraction_inside", "val_fraction_inside_median_ride",
        "notes",
    )}, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

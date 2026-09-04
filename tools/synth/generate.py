"""Deterministic synthetic runs for RailBreak.

Body: Newton III F_∥ = Σ F_adh,i. Filter plant stays Coulomb on the command.
Wheels: Jω̇ = T − r F_adh. F_adh is a 1D Wear 2005 (11)+(4)+(9) slice
(Q_i=(m/n)g, ε from (4), μ(w) from (9), Table 2 kA,kS). Not CONTACT, no spin.

Usage:
  python tools/synth/generate.py
  python tools/synth/generate.py --only slip_accel
  python tools/synth/generate.py --noise-sigma 0.05 --quantize 256
  (noise/quantisation hit the measured omega columns only; physics state and
  gt stay clean; RNG is random.Random(SEED) — bitwise repeatable)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "eval"))

from plant_ref import PlantParams, VehicleState, generator_step  # noqa: E402
from plant_ref import (  # noqa: E402
    G,
    POLACH9_A,
    POLACH9_B_DRY_S_PER_M,
    POLACH9_B_WET_S_PER_M,
    POLACH11_KA_DRY,
    POLACH11_KA_WET,
    POLACH11_KS_DRY,
    POLACH11_KS_WET,
)

SEED = 42
DT = 0.02
R0 = 0.35

# Strogino approach estimate (not a survey). Peak |i| = 0.032 is the centre
# of 25–40‰ (OSINT 04.09.2026 v2). Steep variant 0.040. NEVER copy into the
# filter node (i(s) lives in F_bias). Trapezoid 450 m: rise 100 + hold 250 + fall 100.
STROGINO_I_PEAK = 0.032
STROGINO_I_STEEP = 0.040
STROGINO_S_FLAT_M = 60.0
STROGINO_S_RISE_M = 100.0
STROGINO_S_HOLD_M = 250.0
STROGINO_S_FALL_M = 100.0


def strogino_grade_mag(s_m: float, peak: float = STROGINO_I_PEAK) -> float:
    """Non-negative |i|: flat → rise → hold → fall (450 m trapezoid)."""
    u = s_m - STROGINO_S_FLAT_M
    if u <= 0.0:
        return 0.0
    if u < STROGINO_S_RISE_M:
        return peak * (u / STROGINO_S_RISE_M)
    hold_end = STROGINO_S_RISE_M + STROGINO_S_HOLD_M
    if u < hold_end:
        return peak
    fall = u - hold_end
    if fall < STROGINO_S_FALL_M:
        return peak * (1.0 - fall / STROGINO_S_FALL_M)
    return 0.0


@dataclass
class Cmd:
    notch: float
    brake: float
    mu: float
    mass_kg: float
    f_bias_n: float
    d: list[float]
    freeze: list[bool]
    wsp_clip: bool
    polach_A: float
    polach_B: float
    polach_kA: float
    polach_kS: float


def scenario_cmd(name: str, t: float, _v: float, s_m: float = 0.0) -> Cmd:
    d = [1.0, 1.0, 1.0, 1.0]
    freeze = [False, False, False, False]
    mu = 0.35  # dry class (~0.35–0.4)
    mass = 28000.0
    f_bias = 0.0
    notch = 0.0
    brake = 0.0
    wsp = False
    polach_A = POLACH9_A
    polach_B = POLACH9_B_DRY_S_PER_M
    polach_kA = POLACH11_KA_DRY
    polach_kS = POLACH11_KS_DRY

    if name in ("slip_accel", "mismatch_jerk_slip"):
        notch = 1.0 if t < 18.0 else 0.15
        mu = 0.06  # contaminated wet film (leaves/oil), not rain
        wsp = True
        polach_B = POLACH9_B_WET_S_PER_M
        polach_kA = POLACH11_KA_WET
        polach_kS = POLACH11_KS_WET
    elif name == "slide_brake":
        if t < 8.0:
            notch = 0.8
        else:
            notch = 0.0
            brake = 0.85
            mu = 0.06  # contaminated film; WSP lock ≤0.4 s in generator
            polach_B = POLACH9_B_WET_S_PER_M
            polach_kA = POLACH11_KA_WET
            polach_kS = POLACH11_KS_WET
    elif name == "slide_on_grade":
        # Talerddig-class: brake + slide + unmapped grade 1-in-56
        # (RAIB Report 08/2026 Fig. 46 / para 245). Not a distance "1080 m"
        # (MH1080 is a block marker).
        if t < 8.0:
            notch = 0.8
        else:
            notch = 0.0
            brake = 0.85
            mu = 0.06
            f_bias = mass * G / 56.0
            wsp = False
            polach_B = POLACH9_B_WET_S_PER_M
            polach_kA = POLACH11_KA_WET
            polach_kS = POLACH11_KS_WET
    elif name == "snow_ice":
        notch = 1.0
        cycle = s_m % 80.0
        mu = 0.06 if cycle < 20.0 else 0.35
        wsp = mu < 0.15
        if mu < 0.15:
            polach_B = POLACH9_B_WET_S_PER_M
            polach_kA = POLACH11_KA_WET
            polach_kS = POLACH11_KS_WET
    elif name == "six_axle":
        # 6 axles, 37 t tare (Витязь-М). r0 still Combino until bag.
        d = [1.0] * 6
        freeze = [False] * 6
        mass = 37000.0
        notch = 0.5
    elif name == "axle_fault":
        # Encoder 3 holds its last *measurement* after t=8 s (wired in simulate).
        # A physical freeze of ω keeps rω≈v via Newton III, so SCA never wakes.
        # Sensor consensus is about a lying encoder, not a seized axle.
        notch = 0.85 if t < 25.0 else 0.1
    elif name == "diameter_wear":
        notch = 0.5
        d = [1.03, 1.025, 1.028, 1.03]
    elif name == "grade_unmapped":
        notch = 0.55
        f_bias = mass * G * math.sin(0.02)  # ~2 % grade, not in filter map
    elif name == "tight_curve":
        notch = 0.4
        if 80.0 < t * 6.0 < 220.0:  # rough s-window via time
            f_bias = 2500.0  # stand-in for R_curve
    elif name == "zupt_dwell":
        if t < 6.0:
            notch = 0.5
        elif t < 10.0:
            brake = 0.7
        else:
            notch = 0.0
            brake = 0.3
    elif name == "jagged_notch":
        k = int(t / 1.5)
        notch = [0.9, -0.2, 0.6, 0.1, 0.8, -0.4, 0.3][k % 7]
    elif name == "heavy_pax":
        mass = 28000.0 + 12000.0
        notch = 0.7
    elif name == "coast_no_wire":
        if t < 10.0:
            notch = 0.8
        else:
            notch = 0.0  # autonomous coast, Davis only
    elif name == "model_mismatch":
        # Generator plant ≠ filter defaults (Davis×1.3, F*×0.7, 1.5% grade).
        # Wrong r0 is not mixed in: that silent-OK leak is a YAML/identify issue.
        notch = 0.7
        f_bias = mass * G * 0.015
    elif name == "mismatch_r0":
        notch = 0.5
        d = [0.88, 0.88, 0.88, 0.88]
    elif name == "mismatch_jerk":
        notch = 0.7
    elif name == "wet_clean":
        notch = 0.6
        mu = 0.20  # clean wet rail, not contaminated film
        polach_B = POLACH9_B_WET_S_PER_M
        polach_kA = POLACH11_KA_WET
        polach_kS = POLACH11_KS_WET
    elif name == "coast_grade_route10":
        # Sign test: coast on estimated Strogino descent. Filter has no i(s).
        # F_bias must absorb; large κ on coast is model error, not slide.
        mag = strogino_grade_mag(s_m)
        f_bias = -mass * G * mag
        if t < 10.0:
            notch = 0.8
        else:
            notch = 0.0
    elif name == "slide_on_grade_route10":
        # Estimate ~1.8× Talerddig 1:56 at 32‰. Contaminated film, WSP off.
        mag = strogino_grade_mag(s_m)
        f_bias = -mass * G * mag
        if t < 8.0:
            notch = 0.8
        else:
            notch = 0.0
            brake = 0.85
            mu = 0.06
            wsp = False
            polach_B = POLACH9_B_WET_S_PER_M
            polach_kA = POLACH11_KA_WET
            polach_kS = POLACH11_KS_WET
    elif name == "slide_on_grade_route10_steep":
        mag = strogino_grade_mag(s_m, STROGINO_I_STEEP)
        f_bias = -mass * G * mag
        if t < 8.0:
            notch = 0.8
        else:
            notch = 0.0
            brake = 0.85
            mu = 0.06
            wsp = False
            polach_B = POLACH9_B_WET_S_PER_M
            polach_kA = POLACH11_KA_WET
            polach_kS = POLACH11_KS_WET
    elif name == "grade_traction_route10":
        # Climb (Kulakova → Shchukinskaya). Contaminated film → slip under traction.
        mag = strogino_grade_mag(s_m)
        f_bias = mass * G * mag
        notch = 1.0
        mu = 0.06
        wsp = True
        polach_B = POLACH9_B_WET_S_PER_M
        polach_kA = POLACH11_KA_WET
        polach_kS = POLACH11_KS_WET
    else:
        raise ValueError(name)

    return Cmd(
        notch, brake, mu, mass, f_bias, d, freeze, wsp,
        polach_A, polach_B, polach_kA, polach_kS,
    )


SCENARIOS = [
    "slip_accel",
    "slide_brake",
    "axle_fault",
    "diameter_wear",
    "grade_unmapped",
    "tight_curve",
    "zupt_dwell",
    "jagged_notch",
    "heavy_pax",
    "coast_no_wire",
    "model_mismatch",
    "mismatch_r0",
    "mismatch_jerk",
    "mismatch_jerk_slip",
    "snow_ice",
    "slide_on_grade",
    "six_axle",
    "wet_clean",
    "coast_grade_route10",
    "slide_on_grade_route10",
    "slide_on_grade_route10_steep",
    "grade_traction_route10",
]


def scenario_duration_s(name: str) -> float:
    if "route10" in name:
        return 50.0
    return 30.0


def simulate(name: str, duration_s: float = 30.0, noise_sigma: float = 0.0,
             quantize: int = 0) -> list[dict]:
    p = PlantParams()
    x = VehicleState()
    c0 = scenario_cmd(name, 0.0, 0.0, 0.0)
    nw = max(len(c0.d), 4)
    omega = [0.0] * nw
    lock_s = [0.0] * nw
    x.d = list(c0.d)
    x.m_eff_kg = c0.mass_kg
    rng = random.Random(SEED)
    q_step = (2.0 * math.pi / quantize) if quantize > 0 else 0.0
    rows: list[dict] = []
    t = 0.0
    n = int(duration_s / DT)
    hold_w3: float | None = None
    f_filt = [0.0]
    for _ in range(n):
        p = PlantParams()
        if name == "model_mismatch":
            p.A_d *= 1.3
            p.B_d *= 1.3
            p.C_d *= 1.3
            p.a_trac_max *= 0.7
        if name in ("mismatch_jerk", "mismatch_jerk_slip"):
            p.j_max_mps3 = 0.7
            p.tau_drv_s = 0.3
        if name == "six_axle":
            p.m0_kg = 37000.0
        c = scenario_cmd(name, t, x.v_mps, x.s_m)
        x.mu_hat = c.mu
        x.m_eff_kg = c.mass_kg
        x.f_bias_n = c.f_bias_n
        x.d = list(c.d)
        if len(omega) < len(c.d):
            omega.extend([0.0] * (len(c.d) - len(omega)))
            lock_s.extend([0.0] * (len(c.d) - len(lock_s)))
        p.polach_A = c.polach_A
        p.polach_B_s_per_m = c.polach_B
        p.polach_kA = c.polach_kA
        p.polach_kS = c.polach_kS
        _, omega, _f_adh = generator_step(
            x,
            omega,
            c.notch,
            c.brake,
            DT,
            freeze=c.freeze,
            wsp=c.wsp_clip,
            p=p,
            f_trac_filt=f_filt,
            lock_s=lock_s,
        )
        measured = list(omega)
        if noise_sigma > 0.0:
            measured = [w + rng.gauss(0.0, noise_sigma) for w in measured]
        if q_step > 0.0:
            measured = [round(w / q_step) * q_step for w in measured]
        if name == "axle_fault" and t > 8.0:
            if hold_w3 is None:
                hold_w3 = measured[3]
            measured[3] = hold_w3
        if name == "tight_curve":
            # Independently rotating wheels, 1524 mm gauge, R ≈ 25 m.
            # Bogie L/R pair_lr should cancel this before SCA.
            delta = 0.5 * 1.524 / 25.0
            measured[0] *= 1.0 - delta
            measured[2] *= 1.0 - delta
            measured[1] *= 1.0 + delta
            measured[3] *= 1.0 + delta
        rec = {
            "t_s": round(t, 4),
            "notch": c.notch,
            "brake": c.brake,
            "gt_s": x.s_m,
            "gt_v": x.v_mps,
        }
        for i, w in enumerate(measured):
            rec[f"w{i}"] = w
        rows.append(rec)
        t += DT
    return rows


def _wheel_keys(row: dict) -> list[str]:
    return sorted(
        (k for k in row if k.startswith("w") and k[1:].isdigit()),
        key=lambda k: int(k[1:]),
    )


def write_run(name: str, dest: Path, noise_sigma: float = 0.0,
              quantize: int = 0) -> Path:
    dest.mkdir(parents=True, exist_ok=True)
    rows = simulate(name, duration_s=scenario_duration_s(name),
                    noise_sigma=noise_sigma, quantize=quantize)
    wkeys = _wheel_keys(rows[0]) if rows else ["w0", "w1", "w2", "w3"]
    fields = ["t_s", "notch", "brake", *wkeys, "gt_s", "gt_v"]
    csv_path = dest / "run.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    gt_path = dest / "gt.jsonl"
    with gt_path.open("w", encoding="utf-8") as f:
        for row in rows:
            f.write(
                json.dumps(
                    {
                        "kind": "gt",
                        "t": row["t_s"],
                        "s": row["gt_s"],
                        "v": row["gt_v"],
                    }
                )
                + "\n"
            )
    filt = dest / "filter.csv"
    filt_fields = ["t_s", "notch", "brake", *wkeys]
    with filt.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=filt_fields)
        w.writeheader()
        for row in rows:
            w.writerow({k: row[k] for k in filt_fields})
    (dest / "meta.json").write_text(
        json.dumps(
            {
                "scenario": name,
                "seed": SEED,
                "dt_s": DT,
                "n": len(rows),
                "n_wheels": len(wkeys),
                "noise_sigma_rad_s": noise_sigma,
                "quantize_pulses_per_rev": quantize,
                "note": (
                    "synthetic; Newton III body; Coulomb plant in UKF. "
                    "route10 grades are Strogino-approach estimates (25–40‰, centre 32‰), not i(s) in the node."
                    if "route10" in name
                    else "synthetic; not route 10; Newton III body; Coulomb plant in UKF"
                ),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return dest


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ROOT / "synth" / "runs")
    ap.add_argument("--only", choices=SCENARIOS, default=None)
    ap.add_argument("--noise-sigma", type=float, default=0.0,
                    help="Gaussian sigma on measured omega, rad/s; 0 = off")
    ap.add_argument("--quantize", type=int, default=0,
                    help="encoder pulses per wheel revolution; 0 = off")
    args = ap.parse_args(argv)
    names = [args.only] if args.only else SCENARIOS
    for name in names:
        path = write_run(name, args.out / name, noise_sigma=args.noise_sigma,
                         quantize=args.quantize)
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

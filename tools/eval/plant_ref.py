"""Longitudinal plant and SCA formulas twin of C++ libtram_dr.

Used by baselines and tests. The envelope checker must not import ukf.
Keep this file free of Kalman updates.

Filter twin (`plant_step`): Newton + Davis + Coulomb cap on net contact
(traction minus brake). Optional PT1 on F* and rotary γ, both default 0
(C++ parity). No wheel ODE. No μ(w). No Polach (11).

Generator only (`generator_step`): wheel ODE + Newton III F_body = Σ F_adh,i.
Default F_adh is a 1D slice of Wear 2005 (11)+(4)+(9): Q_i = (m/n)g,
ε from (4) with estimated a,b,c11, μ(w) from (9). No spin, no Hertz from
profiles, not CONTACT. Optional creep_force='tanh' keeps the old cap.
Implicit Euler on ω for (11): Kalker τ ~ 0.3 ms is not resolved at 20 ms.
Do not use this path in the UKF.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

N_WHEELS = 6
G = 9.81
MASS_MIN_KG = 20000.0
MASS_MAX_KG = 70000.0
# Rotational inertia of one wheelset.
J_WHEEL_KG_M2 = 60.0
# tanh scale on slip velocity rω−v.
W_CREEP_MPS = 0.4
WSP_SLIP_MPS = 1.5
WSP_TORQUE_SCALE = 0.45
# EN 15595 / UIC 541-05: lock duration bound used in the generator, not a WSP.
WSP_LOCK_MAX_S = 0.4
OMEGA_MAX = 80.0
# Wear 2005 Table 2 typical locomotive A,B. Eq. (9) only.
# A is μ∞/μ0, not Davis A_d. B is s/m, not Davis B_d.
POLACH9_A = 0.40
POLACH9_B_DRY_S_PER_M = 0.60
POLACH9_B_WET_S_PER_M = 0.20
# Wear 2005 Table 2 kA, kS. Eq. (11).
POLACH11_KA_DRY = 1.00
POLACH11_KS_DRY = 0.40
POLACH11_KA_WET = 0.30
POLACH11_KS_WET = 0.10
# Ellipse and Kalker for ε_x in eq. (4). Not from measured profiles.
HERTZ_A_M = 6.0e-3
HERTZ_B_M = 4.0e-3
KALKER_C11 = 4.3
# Steel shear modulus (E=210 GPa, ν=0.3). Not gravity G.
SHEAR_MOD_PA = 8.1e10


@dataclass
class PlantParams:
    m0_kg: float = 28000.0
    a_trac_max: float = 1.3
    a_svc: float = 1.2
    v_base_mps: float = 6.0
    A_d: float = 800.0
    B_d: float = 40.0
    C_d: float = 6.0
    r0_m: float = 0.35
    v_eps: float = 0.1
    polach_A: float = POLACH9_A
    polach_B_s_per_m: float = POLACH9_B_DRY_S_PER_M
    polach_kA: float = POLACH11_KA_DRY
    polach_kS: float = POLACH11_KS_DRY
    hertz_a_m: float = HERTZ_A_M
    hertz_b_m: float = HERTZ_B_M
    kalker_c11: float = KALKER_C11
    shear_mod_pa: float = SHEAR_MOD_PA
    # 'polach11' = Wear 2005 (11); 'tanh' = engineering cap (tests / rollback).
    creep_force: str = "polach11"
    # Filter twin only. 0 = algebraic F=F* (C++ default). Generator ignores these.
    tau_drv_s: float = 0.0
    gamma_rot: float = 0.0
    brake_nonadhesive_frac: float = 0.0
    notch_as_accel: bool = False
    i_grade: float = 0.0
    # |dF*/dt| ≤ m j_max. 0 = off (filter/synth twin). Not a PT1.
    j_max_mps3: float = 0.0
    mass_min_kg: float = MASS_MIN_KG
    mass_max_kg: float = MASS_MAX_KG


@dataclass
class VehicleState:
    s_m: float = 0.0
    v_mps: float = 0.0
    a_mps2: float = 0.0
    f_bias_n: float = 0.0
    m_eff_kg: float = 28000.0
    k_trac: float = 1.0
    d: list[float] = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0])
    mu_hat: float = 0.35


def davis_resistance_n(v_mps: float, p: PlantParams | None = None) -> float:
    p = p or PlantParams()
    return p.A_d + p.B_d * abs(v_mps) + p.C_d * v_mps * v_mps


def traction_star_n(notch: float, v_mps: float, p: PlantParams | None = None) -> float:
    p = p or PlantParams()
    f_max = p.m0_kg * p.a_trac_max
    av = abs(v_mps)
    mag = f_max if av <= p.v_base_mps else f_max * p.v_base_mps / max(av, p.v_base_mps)
    cmd = max(-1.0, min(1.0, notch))
    return cmd * mag


def polach9_mu(mu0: float, w_mps: float, A: float, B: float) -> float:
    """Wear 2005 eq. (9): μ = μ0 [(1−A) exp(−B |w|) + A].

    Not eq. (11). A,B here are Polach friction parameters, not Davis.
    """
    a = min(max(A, 0.0), 1.0)
    b = max(B, 0.0)
    mu0c = max(float(mu0), 1e-4)
    mu = mu0c * ((1.0 - a) * math.exp(-b * abs(w_mps)) + a)
    return max(mu, 1e-4)


def polach11_force_n(
    Q_n: float,
    mu: float,
    slip_mps: float,
    v_mps: float,
    p: PlantParams | None = None,
) -> float:
    """1D Wear 2005 eq. (11) with ε from eq. (4). Force on the body (sign of slip).

    Not FASTSIM, not CONTACT, no spin. a,b,c11 are estimates, not Hertz-from-profile.
    """
    p = p or PlantParams()
    if Q_n <= 0.0 or mu <= 0.0:
        return 0.0
    sx = slip_mps / max(abs(v_mps), p.v_eps)
    denom = max(Q_n * mu, 1e-6)
    eps_x = (
        0.25
        * p.shear_mod_pa
        * math.pi
        * max(p.hertz_a_m, 1e-6)
        * max(p.hertz_b_m, 1e-6)
        * max(p.kalker_c11, 0.0)
        / denom
        * sx
    )
    eps = abs(eps_x)
    k_a = min(max(p.polach_kA, 0.0), 1.0)
    k_s = min(max(p.polach_kS, 0.0), k_a)
    term = (k_a * eps) / (1.0 + (k_a * eps) ** 2) + math.atan(k_s * eps)
    mag = (2.0 * Q_n * mu / math.pi) * term
    if slip_mps == 0.0 or mag == 0.0:
        return 0.0
    return math.copysign(mag, slip_mps)


def adhesion_force_n(
    Q_n: float,
    mu0: float,
    slip_mps: float,
    v_mps: float,
    p: PlantParams | None = None,
) -> float:
    """Per-axle body force: (9) then (11), or tanh rollback."""
    p = p or PlantParams()
    mu_i = polach9_mu(mu0, slip_mps, p.polach_A, p.polach_B_s_per_m)
    if p.creep_force == "tanh":
        return Q_n * mu_i * math.tanh(slip_mps / W_CREEP_MPS)
    return polach11_force_n(Q_n, mu_i, slip_mps, v_mps, p)


def _implicit_wheel_omega(
    omega: float,
    v: float,
    r: float,
    torque: float,
    dt: float,
    J: float,
    Q_n: float,
    mu0: float,
    p: PlantParams,
) -> tuple[float, float]:
    """Backward Euler ω⁺ = ω + (dt/J)(T − r F(rω⁺ − v)).

    Eq. (4)+(11) restores Kalker-scale stiffness (τ ~ 0.3 ms). Explicit Euler
    at the 20 ms generator step chatters sign(F). This is an ODE discretisation,
    not CONTACT.
    """
    J = max(J, 1.0)

    def force_at(w: float) -> float:
        return adhesion_force_n(Q_n, mu0, r * w - v, v, p)

    def residual(w: float) -> float:
        return w - omega - (dt / J) * (torque - r * force_at(w))

    if dt <= 0.0:
        return omega, force_at(omega)

    w = omega
    F = force_at(w)
    for _ in range(20):
        F = force_at(w)
        g = w - omega - (dt / J) * (torque - r * F)
        if abs(g) <= 1e-10:
            return max(-OMEGA_MAX, min(OMEGA_MAX, w)), F
        dw = 1e-6
        dF = (force_at(w + dw) - F) / dw
        gp = 1.0 + (dt / J) * (r * r) * dF
        if abs(gp) < 1e-12:
            break
        step = g / gp
        if abs(step) > 25.0:
            step = math.copysign(25.0, step)
        w = max(-OMEGA_MAX, min(OMEGA_MAX, w - step))

    lo, hi = -OMEGA_MAX, OMEGA_MAX
    glo, ghi = residual(lo), residual(hi)
    if glo * ghi <= 0.0:
        for _ in range(48):
            mid = 0.5 * (lo + hi)
            gm = residual(mid)
            if glo * gm <= 0.0:
                hi = mid
                ghi = gm
            else:
                lo = mid
                glo = gm
            if abs(hi - lo) < 1e-10:
                break
        w = 0.5 * (lo + hi)
    w = max(-OMEGA_MAX, min(OMEGA_MAX, w))
    return w, force_at(w)


def plant_step_from_contact(
    x: VehicleState,
    f_contact_n: float,
    dt_s: float,
    p: PlantParams | None = None,
) -> VehicleState:
    """Newton + Davis with a prescribed contact force. Generator only.

    No extra Coulomb clip: per-axle (11) saturates at Q_i μ as ε→∞.
    Davis A is a moving-vehicle term; at |v| < v_eps it is not applied, so
    standstill does not roll backward while wheels spin up. The filter twin
    `plant_step` keeps A at rest (C++ parity).
    """
    p = p or PlantParams()
    if dt_s <= 0.0:
        return x
    m = max(x.m_eff_kg, 1000.0)
    f_run = 0.0 if abs(x.v_mps) < p.v_eps else davis_resistance_n(x.v_mps, p)
    a = (f_contact_n - f_run - x.f_bias_n) / m
    x.a_mps2 = a
    x.s_m += x.v_mps * dt_s + 0.5 * a * dt_s * dt_s
    x.v_mps += a * dt_s
    return x


def drive_force_cmd(
    f_star: float,
    dt_s: float,
    p: PlantParams,
    m: float,
    f_trac_filt: list[float] | None,
) -> float:
    """PT1 and/or jerk limiter on F*. Persistent f_trac_filt is [F_prev]."""
    f_prev = f_trac_filt[0] if f_trac_filt else 0.0
    f_cmd = f_star
    if p.tau_drv_s > 1e-12:
        f_cmd = (p.tau_drv_s * f_prev + dt_s * f_star) / (p.tau_drv_s + dt_s)
    if p.j_max_mps3 > 1e-12:
        df_max = max(m, 1000.0) * p.j_max_mps3 * dt_s
        f_cmd = max(f_prev - df_max, min(f_prev + df_max, f_cmd))
    if f_trac_filt is not None:
        if f_trac_filt:
            f_trac_filt[0] = f_cmd
        else:
            f_trac_filt.append(f_cmd)
    return f_cmd


def plant_step(x: VehicleState, notch: float, brake: float, dt_s: float,
               p: PlantParams | None = None,
               f_trac_filt: list[float] | None = None) -> VehicleState:
    p = p or PlantParams()
    if dt_s <= 0.0:
        return x
    m = max(x.m_eff_kg, 1000.0)
    m_dyn = m * (1.0 + max(p.gamma_rot, 0.0))
    mu = max(0.05, min(0.5, x.mu_hat))
    f_adh = m * G * mu
    if p.notch_as_accel:
        a_cmd = max(-1.0, min(1.0, notch)) * p.a_trac_max
        av = abs(x.v_mps)
        if av > p.v_base_mps:
            a_cmd *= p.v_base_mps / max(av, p.v_base_mps)
        f_star = x.k_trac * m * a_cmd
    else:
        f_star = x.k_trac * traction_star_n(notch, x.v_mps, p)
    f_trac_cmd = drive_force_cmd(f_star, dt_s, p, m, f_trac_filt)
    sgn = 1.0 if x.v_mps >= 0.0 else -1.0
    br = max(0.0, min(1.0, brake))
    f_brake = 0.0 if abs(x.v_mps) < p.v_eps else br * m * p.a_svc * sgn
    frac_rail = max(0.0, min(1.0, p.brake_nonadhesive_frac))
    f_brake_adh = (1.0 - frac_rail) * f_brake
    f_brake_rail = frac_rail * f_brake
    f_contact = max(-f_adh, min(f_adh, f_trac_cmd - f_brake_adh))
    f_run = davis_resistance_n(x.v_mps, p)
    f_grade = m * G * p.i_grade
    a = (f_contact - f_brake_rail - f_run - x.f_bias_n - f_grade) / m_dyn
    x.a_mps2 = a
    x.s_m += x.v_mps * dt_s + 0.5 * a * dt_s * dt_s
    x.v_mps += a * dt_s
    x.m_eff_kg = min(p.mass_max_kg, max(p.mass_min_kg, x.m_eff_kg))
    x.k_trac = min(1.5, max(0.5, x.k_trac))
    x.mu_hat = min(0.5, max(0.05, x.mu_hat))
    x.d = [min(1.05, max(0.85, float(di))) for di in x.d]
    return x


@dataclass
class WheelContact:
    """One generator wheel step. f_adh_n[i] is the force on the body from axle i."""

    omega: list[float]
    f_adh_n: list[float]


def wheel_contact_step(
    omega: Sequence[float],
    v: float,
    notch: float,
    brake: float,
    mu: float,
    mass: float,
    d: Sequence[float],
    dt: float,
    freeze: Sequence[bool] | None = None,
    wsp: bool = False,
    p: PlantParams | None = None,
    J: float = J_WHEEL_KG_M2,
    k_trac: float = 1.0,
    f_trac_cmd: float | None = None,
    lock_s: list[float] | None = None,
) -> WheelContact:
    """Engineering wheel ODE Jω̇ = T − r F_adh. Not in the UKF; generator only.

    Frozen axles keep ω (encoder stuck) but still contribute F_adh from that ω.
    Default (11) uses implicit Euler on ω; tanh rollback stays explicit.
    """
    p = p or PlantParams()
    n = min(len(omega), N_WHEELS)
    out = [float(omega[i]) for i in range(n)]
    f_adh_n = [0.0] * n
    frozen = list(freeze) if freeze is not None else [False] * n
    if n <= 0 or dt <= 0.0:
        return WheelContact(omega=out, f_adh_n=f_adh_n)
    f_star = f_trac_cmd if f_trac_cmd is not None else k_trac * traction_star_n(notch, v, p)
    mu0 = max(0.05, min(0.5, mu))
    m = max(mass, 1000.0)
    n_force = float(n)
    f_trac_i = f_star / n_force
    br = max(0.0, min(1.0, brake))
    sgn = 1.0 if v >= 0.0 else -1.0
    f_brake_i = 0.0 if abs(v) < p.v_eps else br * m * p.a_svc * sgn / n_force
    for i in range(n):
        r = max(d[i] if i < len(d) else 1.0, 0.9) * p.r0_m
        slip = r * out[i] - v
        q_i = (m / n_force) * G
        if i < len(frozen) and frozen[i]:
            f_adh_n[i] = adhesion_force_n(q_i, mu0, slip, v, p)
            continue
        torque = (f_trac_i - f_brake_i) * r
        if wsp and abs(slip) > WSP_SLIP_MPS:
            torque *= WSP_TORQUE_SCALE
        if p.creep_force == "tanh":
            f_adh_n[i] = adhesion_force_n(q_i, mu0, slip, v, p)
            omega_dot = (torque - r * f_adh_n[i]) / max(J, 1.0)
            w_new = out[i] + omega_dot * dt
        else:
            w_new, f_adh_n[i] = _implicit_wheel_omega(
                out[i], v, r, torque, dt, J, q_i, mu0, p
            )
        # Sign-change under service brake → lock. EN 15595-class: ≤0.4 s then release.
        # Strict crossing (not ≤): ω=0 would otherwise relock forever.
        locked = False
        held = 0.0 if lock_s is None or i >= len(lock_s) else lock_s[i]
        crossing = (
            brake > 0.1
            and abs(notch) < 0.05
            and out[i] * w_new < 0.0
            and abs(out[i]) > 1e-4
        )
        holding = held > 0.0 and held < WSP_LOCK_MAX_S and brake > 0.1
        if crossing or holding:
            if held < WSP_LOCK_MAX_S:
                w_new = 0.0
                locked = True
                if lock_s is not None and i < len(lock_s):
                    lock_s[i] = held + dt
            elif lock_s is not None and i < len(lock_s):
                lock_s[i] = 0.0
        elif lock_s is not None and i < len(lock_s):
            lock_s[i] = 0.0
        clipped = max(-OMEGA_MAX, min(OMEGA_MAX, w_new))
        if p.creep_force != "tanh" and (locked or clipped != w_new):
            f_adh_n[i] = adhesion_force_n(q_i, mu0, r * clipped - v, v, p)
        out[i] = clipped
    return WheelContact(omega=out, f_adh_n=f_adh_n)


def wheel_speeds_step(
    omega: Sequence[float],
    v: float,
    notch: float,
    brake: float,
    mu: float,
    mass: float,
    d: Sequence[float],
    dt: float,
    freeze: Sequence[bool] | None = None,
    wsp: bool = False,
    p: PlantParams | None = None,
    J: float = J_WHEEL_KG_M2,
) -> list[float]:
    """Backward-compatible wrapper: return only ω."""
    return wheel_contact_step(
        omega, v, notch, brake, mu, mass, d, dt,
        freeze=freeze, wsp=wsp, p=p, J=J,
    ).omega


def generator_step(
    x: VehicleState,
    omega: Sequence[float],
    notch: float,
    brake: float,
    dt_s: float,
    freeze: Sequence[bool] | None = None,
    wsp: bool = False,
    p: PlantParams | None = None,
    J: float = J_WHEEL_KG_M2,
    f_trac_filt: list[float] | None = None,
    lock_s: list[float] | None = None,
) -> tuple[VehicleState, list[float], list[float]]:
    """One coupled generator step: F_∥ = Σ F_adh,i.

    Cap is Wear 2005 (11)×(4)×(9) per axle unless creep_force='tanh'.
    Not the UKF plant.
    """
    p = p or PlantParams()
    f_star = x.k_trac * traction_star_n(notch, x.v_mps, p)
    f_cmd = drive_force_cmd(f_star, dt_s, p, x.m_eff_kg, f_trac_filt)
    contact = wheel_contact_step(
        omega,
        x.v_mps,
        notch,
        brake,
        x.mu_hat,
        x.m_eff_kg,
        x.d,
        dt_s,
        freeze=freeze,
        wsp=wsp,
        p=p,
        J=J,
        k_trac=x.k_trac,
        f_trac_cmd=f_cmd,
        lock_s=lock_s,
    )
    plant_step_from_contact(x, sum(contact.f_adh_n), dt_s, p)
    return x, contact.omega, contact.f_adh_n


PAIR_AGREE_REL = 0.12
CURVE_SIGMA_REL = 0.08
D_MIN = 0.85


def sca_analyze(omega: Sequence[float], d_scale: Sequence[float] | None = None,
                r0_m: float = 0.35, sigma_v: float = 0.15, z_thresh: float = 2.5,
                inflate_max: float = 100.0, pair_lr: bool = True,
                axle_role: Sequence[int] | None = None,
                traction: bool = False) -> dict:
    """Median + z-inflate. pair_lr averages agreeing L/R before the median."""
    m = min(len(omega), N_WHEELS)
    empty = {
        "n_inflated": 0,
        "n_inflated_motor": 0,
        "n_inflated_trailer": 0,
        "n_trailer_ok": 0,
        "used_trailer_consensus": False,
        "inflate": [],
        "r_omega": [],
        "v_consensus_mps": 0.0,
        "common_mode": False,
    }
    if m <= 0:
        return empty
    d = list(d_scale) if d_scale is not None else [1.0] * m
    role = list(axle_role) if axle_role is not None else [0] * m
    ok = []
    v_raw = []
    n_trailer_ok = 0
    for i in range(m):
        w = float(omega[i])
        finite = math.isfinite(w) and abs(w) <= OMEGA_MAX
        ok.append(finite)
        r = max(d[i] if i < len(d) else 1.0, D_MIN) * r0_m
        v_raw.append(w * r if finite else 0.0)
        if finite and i < len(role) and role[i] == 1:
            n_trailer_ok += 1
    if not any(ok):
        return empty
    v_work = list(v_raw)
    if pair_lr and m >= 4:
        pairs = [(0, 1), (2, 3)]
        if m >= 6:
            pairs.append((4, 5))
        for a, b in pairs:
            if a >= m or b >= m or not (ok[a] and ok[b]):
                continue
            mean = 0.5 * (v_raw[a] + v_raw[b])
            scale = max(abs(mean), 1.0)
            if abs(v_raw[a] - v_raw[b]) <= PAIR_AGREE_REL * scale:
                v_work[a] = v_work[b] = mean
    use_trailers = traction and n_trailer_ok >= 1
    fin = []
    for i in range(m):
        if not ok[i]:
            continue
        if use_trailers and (i >= len(role) or role[i] != 1):
            continue
        fin.append(v_work[i])
    if not fin:
        fin = [v_work[i] for i in range(m) if ok[i]]
        use_trailers = False
    ordered = sorted(fin)
    nf = len(ordered)
    if nf == 1:
        cons = ordered[0]
    elif nf % 2:
        cons = ordered[nf // 2]
    else:
        cons = 0.5 * (ordered[nf // 2 - 1] + ordered[nf // 2])
    sig0 = max(sigma_v, 1e-4)
    sig = max(sig0, CURVE_SIGMA_REL * abs(cons)) if (pair_lr and m >= 4) else sig0
    inflate = []
    r_omega = []
    n_inf = 0
    n_inf_m = 0
    n_inf_t = 0
    for i in range(m):
        r = max(d[i] if i < len(d) else 1.0, D_MIN) * r0_m
        trailer = i < len(role) and role[i] == 1
        if not ok[i]:
            inflate.append(inflate_max)
            r_omega.append(1e6)
            n_inf += 1
            if trailer:
                n_inf_t += 1
            else:
                n_inf_m += 1
            continue
        resid = abs(v_raw[i] - cons)
        inf = 1.0
        sigma = sig
        z = resid / max(sigma, 1e-9)
        while z > z_thresh and inf < inflate_max:
            inf *= 1.5
            sigma = sig * inf
            z = resid / sigma
        if inf > 1.0 + 1e-9:
            n_inf += 1
            if trailer:
                n_inf_t += 1
            else:
                n_inf_m += 1
        inflate.append(inf)
        so = sigma / max(r, 1e-4)
        r_omega.append(so * so)
    return {
        "n_inflated": n_inf,
        "n_inflated_motor": n_inf_m,
        "n_inflated_trailer": n_inf_t,
        "n_trailer_ok": n_trailer_ok,
        "used_trailer_consensus": use_trailers,
        "inflate": inflate,
        "r_omega": r_omega,
        "v_consensus_mps": cons,
        "common_mode": False,
    }

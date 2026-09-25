#include "tram_dr_localization/plant.hpp"

#include <algorithm>
#include <cmath>

namespace tram_dr {

PlantParams default_plant_params() { return {}; }

namespace {

// 1 in the constant-force region, v_b/|v| on the power hyperbola,
// v_b v_2 / v² past the field-weakening knee. v2_mps <= v_base disables the knee.
double traction_speed_scale(double av, const PlantParams& p) {
  const double vb = std::max(p.v_base_mps, 1e-9);
  if (!(av > vb)) {
    return 1.0;
  }
  if (!(p.v2_mps > vb) || av <= p.v2_mps) {
    return vb / std::max(av, vb);
  }
  return (vb * p.v2_mps) / (av * av);
}

}  // namespace

double davis_resistance_n(double v_mps, const PlantParams& p) {
  // Odd in v: resistance opposes motion. Zero in the standstill dead zone
  // so A_d does not roll a parked car backward (ZUPT is not a substitute).
  if (!std::isfinite(v_mps) || std::fabs(v_mps) < p.v_eps) {
    return 0.0;
  }
  const double av = std::fabs(v_mps);
  const double mag = p.A_d + p.B_d * av + p.C_d * av * av;
  return std::copysign(mag, v_mps);
}

double traction_star_n(double notch, double v_mps, const PlantParams& p) {
  const double f_max = p.m0_kg * p.a_trac_max;
  const double av = std::fabs(v_mps);
  const double mag = f_max * traction_speed_scale(av, p);
  const double cmd = std::clamp(notch, -1.0, 1.0);
  return cmd * mag;
}

PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p) {
  if (p.notch_as_accel) {
    const double m = std::max(x.m_eff_kg, 1000.0);
    double a_cmd = std::clamp(commanded_notch(u), -1.0, 1.0) * p.a_trac_max;
    const double av = std::fabs(x.v_mps);
    a_cmd *= traction_speed_scale(av, p);
    return plant_forces(x, u, p, x.k_trac * m * a_cmd);
  }
  return plant_forces(x, u, p, x.k_trac * traction_star_n(commanded_notch(u), x.v_mps, p));
}

PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p,
                        double f_trac_cmd) {
  PlantDeriv d;
  const double m = std::max(x.m_eff_kg, 1000.0);
  const double gamma = std::max(p.gamma_rot, 0.0);
  const double m_dyn = m * (1.0 + gamma);
  const double mu = std::clamp(x.mu_hat, p.mu_min, p.mu_max);
  d.f_adh_cap_n = m * p.g * mu;
  const double sgn_v = (x.v_mps >= 0.0) ? 1.0 : -1.0;
  const double brake = std::clamp(commanded_brake(u), 0.0, 1.0);
  double frac_rail = std::clamp(p.brake_nonadhesive_frac, 0.0, 1.0);
  if (u.emergency) {
    frac_rail = std::max(frac_rail, std::clamp(p.emergency_nonadhesive_frac, 0.0, 1.0));
  }
  const double f_brake =
      (std::fabs(x.v_mps) < p.v_eps) ? 0.0 : brake * m * p.a_svc * sgn_v;
  const double f_brake_adh = (1.0 - frac_rail) * f_brake;
  const double f_brake_rail = frac_rail * f_brake;
  double f_contact = f_trac_cmd - f_brake_adh;
  f_contact = std::clamp(f_contact, -d.f_adh_cap_n, d.f_adh_cap_n);
  d.f_trac_n = f_contact;
  d.f_run_n = davis_resistance_n(x.v_mps, p);
  const double f_grade = m * p.g * p.i_grade;
  const ResidualForce resid =
      residual_force(p.residual, x.v_mps, commanded_notch(u), brake);
  const double f_net =
      f_contact - f_brake_rail - d.f_run_n - x.f_bias_n - f_grade + resid.force_n;
  d.a_mps2 = f_net / m_dyn;
  return d;
}

void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p,
                double* f_trac_filt, bool clip_params) {
  if (dt_s <= 0.0) {
    return;
  }
  double f_star = 0.0;
  if (p.notch_as_accel) {
    const double m = std::max(x.m_eff_kg, 1000.0);
    double a_cmd = std::clamp(commanded_notch(u), -1.0, 1.0) * p.a_trac_max;
    const double av = std::fabs(x.v_mps);
    a_cmd *= traction_speed_scale(av, p);
    f_star = x.k_trac * m * a_cmd;
  } else {
    f_star = x.k_trac * traction_star_n(commanded_notch(u), x.v_mps, p);
  }
  const double f_prev = (f_trac_filt != nullptr) ? *f_trac_filt : 0.0;
  double f_cmd = f_star;
  if (f_trac_filt != nullptr && p.tau_drv_s > 1e-12) {
    f_cmd = (p.tau_drv_s * f_prev + dt_s * f_star) / (p.tau_drv_s + dt_s);
  }
  if (f_trac_filt != nullptr && p.j_max_mps3 > 1e-12) {
    const double m = std::max(x.m_eff_kg, 1000.0);
    const double df_max = m * p.j_max_mps3 * dt_s;
    f_cmd = std::clamp(f_cmd, f_prev - df_max, f_prev + df_max);
  }
  if (f_trac_filt != nullptr) {
    *f_trac_filt = f_cmd;
  }
  const double s0 = x.s_m;
  const double v0 = x.v_mps;
  const PlantDeriv d = plant_forces(x, u, p, f_cmd);
  x.a_mps2 = d.a_mps2;
  x.s_m = s0 + v0 * dt_s + 0.5 * d.a_mps2 * dt_s * dt_s;
  x.v_mps = v0 + d.a_mps2 * dt_s;
  // Constant-a integration must not turn passive resistance into propulsion
  // after a zero crossing. Preserve the existing passive-brake stop, and also
  // handle pure coast. The pure-coast extension excludes grade, bias and
  // residual drive force; the pre-existing brake-stop heuristic is unchanged.
  const bool no_traction = std::fabs(commanded_notch(u)) < 0.05;
  const bool braking = commanded_brake(u) > 1e-9;
  const bool passive_coast = f_cmd == 0.0 && x.f_bias_n == 0.0 && p.i_grade == 0.0;
  if (no_traction && (braking || passive_coast) && v0 * x.v_mps < 0.0 &&
      std::fabs(d.a_mps2) > 1e-18) {
    const double t_stop = -v0 / d.a_mps2;
    if (t_stop > 0.0 && t_stop <= dt_s) {
      x.s_m = s0 + v0 * t_stop + 0.5 * d.a_mps2 * t_stop * t_stop;
      x.v_mps = 0.0;
    }
  }
  if (clip_params) {
    x.m_eff_kg = std::clamp(x.m_eff_kg, p.mass_min_kg, p.mass_max_kg);
    x.k_trac = std::clamp(x.k_trac, kKtracMin, kKtracMax);
    x.mu_hat = std::clamp(x.mu_hat, p.mu_min, p.mu_max);
    for (double& di : x.d) {
      di = std::clamp(di, kDMin, kDMax);
    }
  }
}

void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p,
                double* f_trac_filt) {
  plant_step(x, u, dt_s, p, f_trac_filt, true);
}

void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p) {
  plant_step(x, u, dt_s, p, nullptr, true);
}

void plant_step(State& x, const Input& u, double dt_s) {
  plant_step(x, u, dt_s, default_plant_params());
}

void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p,
                       double* f_trac_filt, bool clip_params) {
  State s = unpack_state(x);
  plant_step(s, u, dt_s, p, f_trac_filt, clip_params);
  pack_state(s, x);
}

void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p,
                       double* f_trac_filt) {
  plant_step_packed(x, u, dt_s, p, f_trac_filt, true);
}

void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p) {
  plant_step_packed(x, u, dt_s, p, nullptr, true);
}

}  // namespace tram_dr

#include "tram_dr_localization/plant.hpp"

#include <algorithm>
#include <cmath>

namespace tram_dr {

PlantParams default_plant_params() { return {}; }

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
  const double mag = (av <= p.v_base_mps)
                         ? f_max
                         : f_max * p.v_base_mps / std::max(av, p.v_base_mps);
  const double cmd = std::clamp(notch, -1.0, 1.0);
  return cmd * mag;
}

PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p) {
  if (p.notch_as_accel) {
    const double m = std::max(x.m_eff_kg, 1000.0);
    double a_cmd = std::clamp(u.notch, -1.0, 1.0) * p.a_trac_max;
    const double av = std::fabs(x.v_mps);
    if (av > p.v_base_mps) {
      a_cmd *= p.v_base_mps / std::max(av, p.v_base_mps);
    }
    return plant_forces(x, u, p, x.k_trac * m * a_cmd);
  }
  return plant_forces(x, u, p, x.k_trac * traction_star_n(u.notch, x.v_mps, p));
}

PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p,
                        double f_trac_cmd) {
  PlantDeriv d;
  const double m = std::max(x.m_eff_kg, 1000.0);
  const double gamma = std::max(p.gamma_rot, 0.0);
  const double m_dyn = m * (1.0 + gamma);
  const double mu = std::clamp(x.mu_hat, kMuMin, kMuMax);
  d.f_adh_cap_n = m * p.g * mu;
  const double sgn_v = (x.v_mps >= 0.0) ? 1.0 : -1.0;
  const double brake = std::clamp(u.brake, 0.0, 1.0);
  const double frac_rail = std::clamp(p.brake_nonadhesive_frac, 0.0, 1.0);
  const double f_brake =
      (std::fabs(x.v_mps) < p.v_eps) ? 0.0 : brake * m * p.a_svc * sgn_v;
  const double f_brake_adh = (1.0 - frac_rail) * f_brake;
  const double f_brake_rail = frac_rail * f_brake;
  double f_contact = f_trac_cmd - f_brake_adh;
  f_contact = std::clamp(f_contact, -d.f_adh_cap_n, d.f_adh_cap_n);
  d.f_trac_n = f_contact;
  d.f_run_n = davis_resistance_n(x.v_mps, p);
  const double f_grade = m * p.g * p.i_grade;
  const double f_net = f_contact - f_brake_rail - d.f_run_n - x.f_bias_n - f_grade;
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
    double a_cmd = std::clamp(u.notch, -1.0, 1.0) * p.a_trac_max;
    const double av = std::fabs(x.v_mps);
    if (av > p.v_base_mps) {
      a_cmd *= p.v_base_mps / std::max(av, p.v_base_mps);
    }
    f_star = x.k_trac * m * a_cmd;
  } else {
    f_star = x.k_trac * traction_star_n(u.notch, x.v_mps, p);
  }
  const double f_prev = (f_trac_filt != nullptr) ? *f_trac_filt : 0.0;
  double f_cmd = f_star;
  if (p.tau_drv_s > 1e-12) {
    f_cmd = (p.tau_drv_s * f_prev + dt_s * f_star) / (p.tau_drv_s + dt_s);
  }
  if (p.j_max_mps3 > 1e-12) {
    const double m = std::max(x.m_eff_kg, 1000.0);
    const double df_max = m * p.j_max_mps3 * dt_s;
    f_cmd = std::clamp(f_cmd, f_prev - df_max, f_prev + df_max);
  }
  if (f_trac_filt != nullptr) {
    *f_trac_filt = f_cmd;
  }
  const PlantDeriv d = plant_forces(x, u, p, f_cmd);
  x.a_mps2 = d.a_mps2;
  x.s_m += x.v_mps * dt_s + 0.5 * d.a_mps2 * dt_s * dt_s;
  x.v_mps += d.a_mps2 * dt_s;
  if (clip_params) {
    x.m_eff_kg = std::clamp(x.m_eff_kg, p.mass_min_kg, p.mass_max_kg);
    x.k_trac = std::clamp(x.k_trac, kKtracMin, kKtracMax);
    x.mu_hat = std::clamp(x.mu_hat, kMuMin, kMuMax);
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

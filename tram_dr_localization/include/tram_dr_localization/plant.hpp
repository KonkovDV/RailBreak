#pragma once
#include "tram_dr_localization/types.hpp"

namespace tram_dr {

// Combino NF100 defaults (ITSC 2020). Coulomb on net contact, not Wear 2005 (9)–(16).
struct PlantParams {
  double m0_kg{28000.0};
  double a_trac_max{1.3};
  double a_svc{1.2};
  double g{9.81};
  double v_base_mps{6.0};
  double A_d{800.0};
  double B_d{40.0};
  double C_d{6.0};
  double r0_m{0.35};
  double v_eps{0.1};
  // Drive PT1. 0 = algebraic F=F* (matches synth).
  double tau_drv_s{0.0};
  // a = F_net / (m (1+γ)); adhesion cap still μ m g. 0 matches synth.
  double gamma_rot{0.0};
  // Fraction of brake force that does not go through the rail/wheel μ cap
  // (track magnet). 0 until the bag shows emergency brake.
  double brake_nonadhesive_frac{0.0};
  // Notch as closed-loop accel command: F* = m sat(a_cmd(n)) with the same
  // power hyperbola. false = Combino-style F* ∝ F_max(m0).
  bool notch_as_accel{false};
  // Optional map prior: small-angle grade. 0 unless the organiser map has i(s).
  double i_grade{0.0};
  // |dF*/dt| ≤ m j_max. 0 = off (synth twin). Not a PT1: a rate cap, not lag.
  double j_max_mps3{0.0};
  // SI clip on m_eff outside UT (clip_params) and in xi_to_phys. Львёнок: 15–40 t.
  double mass_min_kg{kMassMinKg};
  double mass_max_kg{kMassMaxKg};
};

struct PlantDeriv {
  double a_mps2{0.0};
  double f_trac_n{0.0};  // adhesion-limited net contact (traction minus brake)
  double f_run_n{0.0};
  double f_adh_cap_n{0.0};
};

PlantParams default_plant_params();

double davis_resistance_n(double v_mps, const PlantParams& p);
double traction_star_n(double notch, double v_mps, const PlantParams& p);
PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p);
// Same Coulomb+Davis, but traction command already lagged (UKF diagnostic / step).
PlantDeriv plant_forces(const State& x, const Input& u, const PlantParams& p,
                        double f_trac_cmd);

// Semi-implicit Euler on s, v. Parameters (mass, d_i, …) are unchanged here;
// UKF process noise is applied on the covariance, not as a random draw.
// f_trac_filt: in/out lagged/rate-limited F* when tau_drv_s>0 or j_max>0.
// clip_params: false on UKF sigma points (bounds are the log/logit bijection).
void plant_step(State& x, const Input& u, double dt_s);
void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p);
void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p,
                double* f_trac_filt);
void plant_step(State& x, const Input& u, double dt_s, const PlantParams& p,
                double* f_trac_filt, bool clip_params);

// Packed x[kStateDim] for UKF sigma points.
void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p);
void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p,
                       double* f_trac_filt);
void plant_step_packed(double* x, const Input& u, double dt_s, const PlantParams& p,
                       double* f_trac_filt, bool clip_params);

}  // namespace tram_dr

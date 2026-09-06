#pragma once
#include "tram_dr_localization/modes.hpp"
#include "tram_dr_localization/plant.hpp"
#include "tram_dr_localization/sca.hpp"
#include "tram_dr_localization/types.hpp"

#include <array>

namespace tram_dr {

struct UkfEstimate {
  State x{};
  double p_ss{1.0e6};
  double p_vv{1.0e6};
  Confidence confidence{Confidence::kUninitialized};
  Mode mode{Mode::kNormal};
  ScaResult sca{};
  bool initialized{false};
  bool slip_latched{false};
  double nis{0.0};       // νᵀ S⁻¹ ν after wheel update; 0 if no update
  bool nis_valid{false};  // false on predict-only / chol fail
  double nis_cusum{0.0};  // CUSUM of NIS − (m + 0.5√m); diagnostic only
  int n_frozen{0};        // axles with Var(ω)≈0 while others still move
  double over_m{0.0};     // PL_s = k_over √P_ss + b_s
  double under_m{0.0};    // k_sigma √P_ss
  double b_s_m{0.0};      // θ_min |v| T_d; unbounded after slip latch
  double pl_s_m{0.0};
  double pl_v_mps{0.0};
  double al_s_m{0.0};
  double a_kin_mps2{0.0}; // Δv/Δt of the estimate, not plant_forces
  bool a_unphysical{false};
  Confidence confidence_v{Confidence::kUninitialized};
  Confidence confidence_s{Confidence::kUninitialized};
  int chol_fail{0};
  double s_unobserved_s{0.0};
  int n_omega_used{0};
  int n_slip_axles{0};  // ω̇-pattern motors under traction, not encoder freeze
  double relative_wheel_slide{0.0};  // κ = (v_wh−v)/max(|v|,1); EN 15595 name
  double path_disagree_m{0.0};       // ∫|v_wh − v_A| dt above floor (signed phases)
  bool path_disagree_latched{false};
  double v_chan_a_mps{0.0};          // channel-A shadow speed (no wheel update)
  bool zupt_at_stop{true};           // mass_door allowed (no map, or |s−stop|≤gate)
  double p_mm{0.0};                  // P on log m; door impulse lands here
  bool s_unbounded{false};           // slip latch: b_s / PL not a metre quantity
  bool zupt_forced{false};           // F-10: ZUPT from ω-only while |v̂|≥0.35
  bool sca_current{true};            // false on predict-only: last_sca_ is stale
};

struct UkfParams {
  // Luo–Moroz sufficient PSD: α≥1/√(1+β)=1/√3≈0.577 at β=2.
  // Necessary W_c^(0)≥0 at κ_UT=0, β=2: α≥√(2−√3)≈0.518. Code uses 0.58.
  double alpha{0.58};
  double beta{2.0};
  double kappa_ut{0.0};        // Julier kappa, not creepage
  double kappa_cut{0.25};      // relative wheel slide |v_wh−v|/max(|v|,1)
  double r_common_mode{80.0};  // extra R scale when κ is large
  double p_ss_init{0.25};      // (0.5 m)^2; s=0 is the DR origin, not a map fix
  double k_sigma{2.0};
  double k_over{2.5};
  double freeze_s{0.5};        // stuck-axle window; not a tick count
  bool cubature{false};        // Arasaratnam–Haykin CKF; default scaled UT
  double kappa_hold_s{0.2};    // common-mode must persist before latch
  double k_lost{2.5};          // LOST if √P_ss > k_lost * (5 + 0.05 max(s,0))
  double notch_lost_s{2.0};    // no notch for this long → LOST
  bool allow_reverse{false};   // otherwise clip s≥0, v≥−1
  double mass_door_kg{3000.0}; // P_mm impulse on leaving ZUPT at a stop
  double q_v{0.0025};          // white-accel intensity m²/s³ (Van Loan Q_d)
  double zupt_hold_s{0.3};     // standstill only after this window
  // Stop vertices (route_s_m). Empty = ungated (synth twin). Not i(s).
  int n_stops{0};
  std::array<double, kMaxStops> stop_s_m{};
  double stop_gate_m{40.0};    // |s − stop| to apply mass_door_kg
  int encoder_pulses_per_rev{0};  // 0 = unused; else R(ω) from pulse count
  double huber_c{3.0};         // DCS/Huber cap on measurement info; 0 = off
  double slide_grade_lost_s{0.4};  // braking + latch + a_kin≥0 → LOST
  double a_kin_downhill{0.05};     // m/s² along-track; 0 = not accelerating under brake
  // Weak prior on log m. Caps P along the (δm, δk) kernel. 0 = off.
  double mass_prior_log_sigma{0.3};
  // F_bias random-walk rate (N/√s) when wheels agree. σ_5s ≈ rate√5.
  double q_fb_wheels_n{3000.0};
  double path_disagree_floor_mps{0.45};
  double path_disagree_rel{0.04};
  double path_disagree_tau_s{60.0};
  // True when wheel_radius_m is a twin placeholder, not a sheet/identify value.
  // Forces DEGRADED on s until identify_coast writes r0 (Львёнок default).
  bool r0_uncalibrated{false};
  double age_degraded_s{0.25};  // core ω-outage (all-NaN / n_ok==0), ROS also uses
  double age_lost_s{1.0};
  // Independent standstill witness (F-10). 0 = off (gate stays |v̂|<0.35 only).
  double zupt_omega_only_s{2.0};
  // Configured axle count. Rest/ZUPT and "incomplete packet" use this, not n.
  int n_wheels{4};
};

// Scaled UKF (Julier 2002; Wan & van der Merwe).
class Ukf {
 public:
  Ukf();
  explicit Ukf(UkfParams cfg);
  void reset();
  void set_params(const UkfParams& cfg);
  void set_plant(const PlantParams& p);
  void set_sca(const ScaParams& p);
  const UkfParams& params() const { return cfg_; }

  // n==0: predict only (no fabricated wheel update). Call update when ω is fresh.
  UkfEstimate predict_and_update(const Input& u, const double* omega, std::size_t n,
                                 double dt_s);
  UkfEstimate predict_and_update(const Input& u, const double* omega, std::size_t n);
  bool initialized() const { return initialized_; }

 private:
  void init_from_wheels(const double* omega, std::size_t n);
  void predict(const Input& u, double dt_s);
  void update_wheels(const double* omega, std::size_t n);
  bool wheels_at_rest(const double* omega, std::size_t n) const;
  bool zupt_gate(const double* omega, std::size_t n, const Input& u) const;
  bool mass_door_allowed() const;
  void maybe_zupt(const double* omega, std::size_t n, const Input& u);
  void observe_rest_packet(const double* omega, std::size_t n, const Input& u);
  void classify(const Input& u, std::size_t n);
  void note_omega(const double* omega, int m);
  void detect_freeze(int m);
  void classify_axle_fault_vs_slip(int m);
  void step_channel_a(const Input& u, double dt_s);
  void accumulate_path_disagree(const Input& u, double dt_s);
  void apply_mass_prior();
  double missed_path_m() const;
  UkfEstimate snapshot() const;

  static constexpr int kFreezeWin = 25;

  UkfParams cfg_{};
  double x_[kStateDim]{};
  double P_[kStateDim * kStateDim]{};
  PlantParams plant_{};
  ScaParams sca_p_{};
  ScaResult last_sca_{};
  Input last_u_{};
  bool slip_latched_{false};
  bool initialized_{false};
  int ticks_no_notch_{0};
  Confidence confidence_{Confidence::kUninitialized};
  Mode mode_{Mode::kNormal};
  double last_nis_{0.0};
  bool nis_valid_{false};
  double nis_cusum_{0.0};
  int n_frozen_{0};
  double omega_hist_[kNWheels][kFreezeWin]{};
  int hist_i_{0};
  int hist_fill_{0};
  bool frozen_[kNWheels]{};
  double v_prev_{0.0};
  bool have_v_prev_{false};
  double a_kin_{0.0};
  bool a_unphysical_{false};
  int chol_fail_{0};
  double last_dt_s_{0.02};
  double s_unobserved_s_{0.0};
  int n_omega_used_{0};
  Confidence confidence_v_{Confidence::kUninitialized};
  Confidence confidence_s_{Confidence::kUninitialized};
  double f_trac_filt_{0.0};
  bool have_f_trac_filt_{false};
  bool standstill_hold_{false};
  double kappa_hold_acc_{0.0};
  double notch_missing_s_{0.0};
  double zupt_acc_{0.0};
  double slide_grade_acc_{0.0};
  int n_slip_axles_{0};
  double relative_wheel_slide_{0.0};
  double v_chan_a_{0.0};
  bool have_v_chan_a_{false};
  double path_disagree_m_{0.0};
  bool path_disagree_latched_{false};
  double mass_prior_acc_{0.0};
  double wheel_outage_s_{0.0};
  bool encoder_outage_{false};
  double omega_zero_s_{0.0};
  bool zupt_estimate_disagree_{false};
  bool sca_r0_locked_{false};
  double brake_missing_s_{0.0};
};

}  // namespace tram_dr

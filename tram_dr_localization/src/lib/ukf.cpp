#include "tram_dr_localization/ukf.hpp"

#include "tram_dr_localization/lin_alg.hpp"
#include "tram_dr_localization/ut_weights.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace tram_dr {
namespace {

constexpr double kDtDefault = 0.02;
constexpr double kMaxStepS = 0.20;  // larger gaps need explicit replay subdivision

void validate_ukf(const UkfParams& p) {
  for (double v : {p.alpha, p.beta, p.kappa_ut, p.kappa_cut, p.r_common_mode,
       p.p_ss_init, p.k_sigma, p.k_over, p.freeze_s, p.kappa_hold_s, p.k_lost,
       p.notch_lost_s, p.mass_door_kg, p.q_v, p.zupt_hold_s, p.stop_gate_m,
       p.huber_c, p.slide_grade_lost_s, p.a_kin_downhill, p.mass_prior_log_sigma,
       p.q_fb_wheels_n, p.path_disagree_floor_mps, p.path_disagree_rel,
       p.path_disagree_tau_s, p.age_degraded_s, p.age_lost_s,
       p.zupt_omega_only_s, static_cast<double>(p.n_wheels)}) {
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite UKF parameter");
  }
  const double c = p.alpha * p.alpha * (kStateDim + p.kappa_ut);
  if (!p.cubature && (!(c > 0.0) || !std::isfinite(c) ||
      !std::isfinite(1.0 / c) || p.alpha <= 0.0)) {
    throw std::invalid_argument("invalid unscented-transform scaling");
  }
  // F-08: alpha, beta and kappa_ut are jointly constrained. Per-parameter
  // bounds accept (0.58, 0, 0), which drives W_c^(0) negative.
  if (!p.cubature &&
      !ut::weights_psd_ok(p.alpha, p.beta, p.kappa_ut, kStateDim)) {
    throw std::invalid_argument(
        "unscented-transform weights are not positive-semidefinite");
  }
  if (p.kappa_cut <= 0 || p.r_common_mode < 1 || p.p_ss_init < 0 ||
      p.k_sigma <= 0 || p.k_over < p.k_sigma || p.freeze_s <= 0 ||
      p.kappa_hold_s < 0 || p.k_lost <= 0 || p.notch_lost_s < 0 ||
      p.mass_door_kg < 0 || p.q_v < 0 || p.zupt_hold_s < 0 ||
      p.stop_gate_m < 0 || p.huber_c < 0 || p.slide_grade_lost_s < 0 ||
      p.mass_prior_log_sigma < 0 || p.q_fb_wheels_n < 0 ||
      p.path_disagree_floor_mps < 0 || p.path_disagree_rel < 0 ||
      p.path_disagree_tau_s < 0 || p.age_degraded_s < 0 ||
      p.age_lost_s < p.age_degraded_s || p.encoder_pulses_per_rev < 0 ||
      p.zupt_omega_only_s < 0 || p.n_stops < 0 || p.n_stops > kMaxStops ||
      p.n_wheels < 1 || p.n_wheels > kNWheels) {
    throw std::invalid_argument("invalid UKF bounds, noise or timeout");
  }
  for (int i = 0; i < p.n_stops; ++i) {
    if (!std::isfinite(p.stop_s_m[static_cast<std::size_t>(i)]))
      throw std::invalid_argument("nonfinite stop coordinate");
  }
}

void validate_plant(const PlantParams& p) {
  for (double v : {p.m0_kg, p.a_trac_max, p.a_svc, p.g, p.v_base_mps,
       p.A_d, p.B_d, p.C_d, p.r0_m, p.v_eps, p.tau_drv_s, p.gamma_rot,
       p.brake_nonadhesive_frac, p.i_grade, p.j_max_mps3,
       p.mass_min_kg, p.mass_max_kg}) {
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite plant parameter");
  }
  if (p.m0_kg <= 0 || p.r0_m <= 0 || p.g <= 0 || p.v_base_mps <= 0 ||
      p.v_eps <= 0 || p.a_trac_max < 0 || p.a_svc < 0 || p.A_d < 0 ||
      p.B_d < 0 || p.C_d < 0 || p.tau_drv_s < 0 || p.gamma_rot < 0 ||
      p.brake_nonadhesive_frac < 0 || p.brake_nonadhesive_frac > 1 ||
      p.j_max_mps3 < 0 || p.mass_min_kg <= 0 || p.mass_max_kg <= p.mass_min_kg) {
    throw std::invalid_argument("invalid plant bounds or units");
  }
}

void set_diag_p(double* P, int i, double v) { la::at(P, kStateDim, i, i) = v; }

struct UtWeights {
  double lam{0.0};
  double c{0.0};
  double wm[kSigma]{};
  double wc[kSigma]{};
};

UtWeights make_ut(const UkfParams& p) {
  UtWeights u;
  if (p.cubature) {
    const double w = 0.5 / static_cast<double>(kStateDim);
    u.lam = 0.0;
    u.c = static_cast<double>(kStateDim);
    for (int i = 0; i < kSigma; ++i) {
      u.wm[i] = w;
      u.wc[i] = w;
    }
    return u;
  }
  u.lam = p.alpha * p.alpha * (kStateDim + p.kappa_ut) - kStateDim;
  u.c = kStateDim + u.lam;
  u.wm[0] = u.lam / u.c;
  u.wc[0] = u.wm[0] + (1.0 - p.alpha * p.alpha + p.beta);
  const double w = 0.5 / u.c;
  for (int i = 1; i < kSigma; ++i) {
    u.wm[i] = w;
    u.wc[i] = w;
  }
  return u;
}

int n_sigma(const UkfParams& p) { return p.cubature ? (2 * kStateDim) : kSigma; }

void fill_q(double* q, double dt_s, const PlantParams& plant, const Input& u,
            bool standstill, const ScaResult& sca, const UkfParams& cfg) {
  la::zero(q, kStateDim);
  q[kS] = 0.0;  // Van Loan Q_ss / Q_sv applied in add_process_q
  q[kV] = 0.0;
  const bool coast = std::fabs(commanded_notch(u)) < 0.05 && commanded_brake(u) < 0.1;
  const bool wheels_ok = sca.n_inflated == 0 && !sca.common_mode;
  double qfb = 200.0 * 200.0;
  if (standstill) {
    qfb = 0.0;
  } else if (coast && wheels_ok) {
    qfb = 1000.0 * 1000.0;
  } else if (wheels_ok) {
    const double rate = std::max(cfg.q_fb_wheels_n, 0.0);
    qfb = rate * rate;
  }
  q[kFbias] = qfb * dt_s;
  const double m0 = std::max(plant.m0_kg, 1.0);
  q[kMass] = (80.0 / m0) * (80.0 / m0) * dt_s;
  q[kKtrac] = (1e-3) * (1e-3) * dt_s;
  for (int i = 0; i < kNWheels; ++i) {
    q[kD0 + i] = (1e-4) * (1e-4) * dt_s;
  }
  q[kMu] = (2e-2) * (2e-2) * dt_s;
}

void add_process_q(double* P, double dt_s, const UkfParams& cfg, const PlantParams& plant,
                   const Input& u, bool standstill, const ScaResult& sca) {
  double q[kStateDim];
  fill_q(q, dt_s, plant, u, standstill, sca, cfg);
  la::add_diag(P, kStateDim, q);
  const double qv = std::max(cfg.q_v, 0.0);
  const double dt = std::max(dt_s, 0.0);
  const double dt2 = dt * dt;
  // Van Loan discrete white-accel on (s, v). No extra diagonal q_s.
  la::at(P, kStateDim, kS, kS) += qv * dt2 * dt / 3.0;
  const double qsv = qv * dt2 / 2.0;
  la::at(P, kStateDim, kS, kV) += qsv;
  la::at(P, kStateDim, kV, kS) += qsv;
  la::at(P, kStateDim, kV, kV) += qv * dt;
}

void xi_to_phys_p(double* x, const PlantParams& plant) {
  xi_to_phys(x, plant.mass_min_kg, plant.mass_max_kg);
}

void step_sigma_xi(double* x_xi, const Input& u, double dt_s, const PlantParams& plant,
                   double* f_lag) {
  double phys[kStateDim];
  std::memcpy(phys, x_xi, sizeof(phys));
  xi_to_phys_p(phys, plant);
  plant_step_packed(phys, u, dt_s, plant, f_lag, false);
  phys_to_xi(phys);
  std::memcpy(x_xi, phys, sizeof(phys));
}

}  // namespace

Ukf::Ukf() : Ukf(UkfParams{}) {}

Ukf::Ukf(UkfParams cfg) { set_params(cfg); reset(); }

void Ukf::set_params(const UkfParams& cfg) { validate_ukf(cfg); cfg_ = cfg; }

void Ukf::set_plant(const PlantParams& p) {
  validate_plant(p);
  if (sca_r0_locked_ && std::fabs(p.r0_m - sca_p_.r0_m) > 1e-12) {
    throw std::invalid_argument("plant r0 mismatches locked SCA r0");
  }
  plant_ = p;
  sca_p_.r0_m = p.r0_m;
}

void Ukf::set_sca(const ScaParams& p) {
  for (double v : {p.r0_m, p.sigma_v_mps, p.sigma_v_rel, p.z_thresh, p.inflate_max}) {
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite SCA parameter");
  }
  if (p.r0_m <= 0 || p.sigma_v_mps <= 0 || p.sigma_v_rel < 0 ||
      p.z_thresh <= 0 || p.inflate_max < 1 ||
      std::fabs(p.r0_m - plant_.r0_m) > 1e-12) {
    throw std::invalid_argument("invalid SCA parameters or wheel-radius mismatch");
  }
  sca_p_ = p;
  sca_r0_locked_ = true;
}

void Ukf::reset() {
  initialized_ = false;
  ticks_no_notch_ = 0;
  confidence_ = Confidence::kUninitialized;
  mode_ = Mode::kNormal;
  last_sca_ = {};
  last_u_ = {};
  slip_latched_ = false;
  last_nis_ = 0.0;
  nis_valid_ = false;
  nis_cusum_ = 0.0;
  n_frozen_ = 0;
  v_prev_ = 0.0;
  have_v_prev_ = false;
  a_kin_ = 0.0;
  a_unphysical_ = false;
  chol_fail_ = 0;
  last_dt_s_ = 0.02;
  s_unobserved_s_ = 0.0;
  n_omega_used_ = 0;
  confidence_v_ = Confidence::kUninitialized;
  confidence_s_ = Confidence::kUninitialized;
  f_trac_filt_ = 0.0;
  have_f_trac_filt_ = false;
  standstill_hold_ = false;
  kappa_hold_acc_ = 0.0;
  notch_missing_s_ = 0.0;
  zupt_acc_ = 0.0;
  slide_grade_acc_ = 0.0;
  n_slip_axles_ = 0;
  relative_wheel_slide_ = 0.0;
  v_chan_a_ = 0.0;
  have_v_chan_a_ = false;
  path_disagree_m_ = 0.0;
  path_disagree_latched_ = false;
  mass_prior_acc_ = 0.0;
  wheel_outage_s_ = 0.0;
  encoder_outage_ = false;
  omega_zero_s_ = 0.0;
  zupt_estimate_disagree_ = false;
  brake_missing_s_ = 0.0;
  hist_i_ = 0;
  hist_fill_ = 0;
  std::memset(omega_hist_, 0, sizeof(omega_hist_));
  std::memset(frozen_, 0, sizeof(frozen_));
  std::memset(x_, 0, sizeof(x_));
  std::memset(P_, 0, sizeof(P_));
  x_[kMass] = plant_.m0_kg;
  x_[kKtrac] = 1.0;
  x_[kMu] = 0.35;
  for (int i = 0; i < kNWheels; ++i) {
    x_[kD0 + i] = 1.0;
  }
  set_diag_p(P_, kS, 1.0e6);
  set_diag_p(P_, kV, 1.0e6);
  set_diag_p(P_, kFbias, 1.0e6);
  set_diag_p(P_, kMass, 1.0e6);
  set_diag_p(P_, kKtrac, 1.0);
  for (int i = 0; i < kNWheels; ++i) {
    set_diag_p(P_, kD0 + i, 1.0e-2);
  }
  set_diag_p(P_, kMu, 0.04);
  phys_to_xi(x_);
}

void Ukf::init_from_wheels(const double* omega, std::size_t n) {
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  double acc = 0.0;
  int n_ok = 0;
  for (int i = 0; i < m; ++i) {
    if (!std::isfinite(omega[i]) || std::fabs(omega[i]) > kOmegaAbsMax) {
      continue;
    }
    acc += omega[i] * plant_.r0_m;
    ++n_ok;
  }
  const double v0 = (n_ok > 0) ? acc / static_cast<double>(n_ok) : 0.0;
  std::memset(x_, 0, sizeof(x_));
  x_[kV] = v0;
  x_[kMass] = plant_.m0_kg;
  x_[kKtrac] = 1.0;
  x_[kMu] = 0.35;
  for (int i = 0; i < kNWheels; ++i) {
    x_[kD0 + i] = 1.0;
  }
  std::memset(P_, 0, sizeof(P_));
  set_diag_p(P_, kS, cfg_.p_ss_init);
  set_diag_p(P_, kV, 1.0);
  set_diag_p(P_, kFbias, 1.0e6);  // N^2
  const double m0 = std::max(plant_.m0_kg, 1.0);
  set_diag_p(P_, kMass, (2000.0 / m0) * (2000.0 / m0));
  set_diag_p(P_, kKtrac, 0.04);
  for (int i = 0; i < kNWheels; ++i) {
    set_diag_p(P_, kD0 + i, 1.0e-4);
  }
  set_diag_p(P_, kMu, 1.0);  // logit space
  phys_to_xi(x_);
  initialized_ = true;
  confidence_ = Confidence::kDegraded;
  v_chan_a_ = v0;
  have_v_chan_a_ = true;
}

void Ukf::predict(const Input& u, double dt_s) {
  const UtWeights ut = make_ut(cfg_);

  double Lchol[kStateDim * kStateDim];
  double Pwork[kStateDim * kStateDim];
  std::memcpy(Pwork, P_, sizeof(Pwork));
  bool ok = false;
  double jit = 1e-9;
  for (int attempt = 0; attempt < 8; ++attempt) {
    if (la::chol(Pwork, Lchol, kStateDim, jit)) {
      ok = true;
      break;
    }
    jit *= 10.0;
    std::memcpy(Pwork, P_, sizeof(Pwork));
  }
  if (!ok) {
    // Finding B: do not step the mean / inflate P here. predict_and_update
    // rolls the whole object back whenever chol_fail_ moves, so a local
    // fallback was dead code that contradicted the atomic-rollback contract.
    ++chol_fail_;
    return;
  }

  const int ns = n_sigma(cfg_);
  double X[kSigma][kStateDim];
  if (cfg_.cubature) {
    const double scale = std::sqrt(static_cast<double>(kStateDim));
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        const double col = scale * la::at(Lchol, kStateDim, j, i);
        X[i][j] = x_[j] + col;
        X[kStateDim + i][j] = x_[j] - col;
      }
    }
  } else {
    for (int j = 0; j < kStateDim; ++j) {
      X[0][j] = x_[j];
    }
    const double scale = std::sqrt(ut.c);
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        const double col = scale * la::at(Lchol, kStateDim, j, i);
        X[1 + i][j] = x_[j] + col;
        X[1 + kStateDim + i][j] = x_[j] - col;
      }
    }
  }
  const double f0 = have_f_trac_filt_ ? f_trac_filt_ : 0.0;
  double f_acc = 0.0;
  for (int s = 0; s < ns; ++s) {
    double f_lag = f0;
    step_sigma_xi(X[s], u, dt_s, plant_, &f_lag);
    f_acc += ut.wm[s] * f_lag;
  }
  f_trac_filt_ = f_acc;
  have_f_trac_filt_ = true;

  double x_pred[kStateDim];
  la::zero(x_pred, kStateDim);
  for (int s = 0; s < ns; ++s) {
    for (int j = 0; j < kStateDim; ++j) {
      x_pred[j] += ut.wm[s] * X[s][j];
    }
  }

  double Pnew[kStateDim * kStateDim];
  la::zero(Pnew, kStateDim * kStateDim);
  for (int s = 0; s < ns; ++s) {
    double dx[kStateDim];
    for (int j = 0; j < kStateDim; ++j) {
      dx[j] = X[s][j] - x_pred[j];
    }
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        la::at(Pnew, kStateDim, i, j) += ut.wc[s] * dx[i] * dx[j];
      }
    }
  }
  add_process_q(Pnew, dt_s, cfg_, plant_, u, standstill_hold_, last_sca_);
  if (!la::project_pd(Pnew, kStateDim)) {
    ++chol_fail_;
    return;
  }
  std::memcpy(x_, x_pred, sizeof(x_));
  std::memcpy(P_, Pnew, sizeof(P_));
}

void Ukf::note_omega(const double* omega, int m) {
  for (int i = 0; i < kNWheels; ++i) {
    omega_hist_[i][hist_i_] = (i < m) ? omega[i] : 0.0;
  }
  hist_i_ = (hist_i_ + 1) % kFreezeWin;
  if (hist_fill_ < kFreezeWin) {
    ++hist_fill_;
  }
}

void Ukf::detect_freeze(int m) {
  n_frozen_ = 0;
  std::memset(frozen_, 0, sizeof(frozen_));
  const double dt = std::max(last_dt_s_, 1e-3);
  const int need =
      std::clamp(static_cast<int>(std::lround(cfg_.freeze_s / dt)), 5, kFreezeWin);
  if (hist_fill_ < need || m <= 0) {
    return;
  }
  double var[kNWheels]{};
  for (int i = 0; i < m; ++i) {
    double mean = 0.0;
    for (int k = 0; k < need; ++k) {
      const int idx = (hist_i_ - 1 - k + kFreezeWin) % kFreezeWin;
      mean += omega_hist_[i][idx];
    }
    mean /= static_cast<double>(need);
    double acc = 0.0;
    for (int k = 0; k < need; ++k) {
      const int idx = (hist_i_ - 1 - k + kFreezeWin) % kFreezeWin;
      const double d = omega_hist_[i][idx] - mean;
      acc += d * d;
    }
    var[i] = acc / static_cast<double>(need);
  }
  double vmax = 0.0;
  for (int i = 0; i < m; ++i) {
    vmax = std::max(vmax, var[i]);
  }
  // Healthy constant-speed synth has Var=0 on every axle — do not flag
  // unless at least one axle is still moving.
  constexpr double kVarStill = 0.02 * 0.02;
  constexpr double kVarMoving = 0.05 * 0.05;
  if (vmax <= kVarMoving) {
    return;
  }
  for (int i = 0; i < m; ++i) {
    if (var[i] < kVarStill) {
      frozen_[i] = true;
      ++n_frozen_;
    }
  }
}

void Ukf::classify_axle_fault_vs_slip(int m) {
  n_slip_axles_ = 0;
  if (hist_fill_ < 2 || m <= 0) {
    return;
  }
  const double dt = std::max(last_dt_s_, 1e-3);
  const int i0 = (hist_i_ - 1 + kFreezeWin) % kFreezeWin;
  const int i1 = (hist_i_ - 2 + kFreezeWin) % kFreezeWin;
  const bool traction = last_u_.notch > 0.05;
  for (int i = 0; i < m; ++i) {
    if (frozen_[i]) {
      continue;
    }
    if (last_sca_.inflate[static_cast<std::size_t>(i)] <= 1.0 + 1e-9) {
      continue;
    }
    const double wdot =
        (omega_hist_[i][i0] - omega_hist_[i][i1]) / dt;
    const bool motor = sca_p_.axle_role[static_cast<std::size_t>(i)] != kAxleTrailer;
    // Spinning motor under traction: ω̇ ~ T/J. Encoder freeze is Var≈0, not this.
    if (traction && motor && std::fabs(wdot) > 8.0) {
      ++n_slip_axles_;
    }
  }
}

void Ukf::update_wheels(const double* omega, std::size_t n) {
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  if (m <= 0 || omega == nullptr) {
    return;
  }
  nis_valid_ = false;
  last_nis_ = 0.0;
  int n_ok = 0;
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(omega[i]) && std::fabs(omega[i]) <= kOmegaAbsMax) {
      ++n_ok;
    }
  }
  n_omega_used_ = n_ok;
  if (n_ok == 0) {
    // All channels NaN/out of range: do not keep a stale SCA consensus.
    // sca_analyze marks every axle inflated → classify() can LOST (n≥4).
    double phys[kStateDim];
    std::memcpy(phys, x_, sizeof(phys));
    xi_to_phys_p(phys, plant_);
    last_sca_ = sca_analyze(omega, static_cast<std::size_t>(m), &phys[kD0], sca_p_);
    encoder_outage_ = true;
    nis_valid_ = false;
    last_nis_ = 0.0;
    return;
  }
  encoder_outage_ = false;
  wheel_outage_s_ = 0.0;
  double phys[kStateDim];
  std::memcpy(phys, x_, sizeof(phys));
  xi_to_phys_p(phys, plant_);
  sca_p_.traction = last_u_.notch > 0.05;
  last_sca_ = sca_analyze(omega, static_cast<std::size_t>(m), &phys[kD0], sca_p_);
  int live[kNWheels];
  int m_live = 0;
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(omega[i]) && std::fabs(omega[i]) <= kOmegaAbsMax) {
      live[m_live++] = i;
    }
  }
  note_omega(omega, m);
  detect_freeze(m);
  for (int i = 0; i < m; ++i) {
    if (!std::isfinite(omega[i]) || std::fabs(omega[i]) > kOmegaAbsMax) {
      frozen_[i] = true;
    }
  }
  n_frozen_ = 0;
  for (int i = 0; i < m; ++i) {
    if (frozen_[i]) {
      ++n_frozen_;
    }
  }
  classify_axle_fault_vs_slip(m);
  for (int i = 0; i < m; ++i) {
    if (!frozen_[i]) {
      continue;
    }
    last_sca_.r_omega[static_cast<std::size_t>(i)] *=
        std::max(10.0, sca_p_.inflate_max);
    if (last_sca_.inflate[static_cast<std::size_t>(i)] <= 1.0 + 1e-9) {
      last_sca_.inflate[static_cast<std::size_t>(i)] = sca_p_.inflate_max;
      ++last_sca_.n_inflated;
    }
  }
  const double v_wh = last_sca_.v_consensus_mps;
  const double v_body = x_[kV];
  // Denominator floor 1 m/s: relative wheel slide κ=(v_wh−v)/v is degenerate near
  // standstill (a 0.14 m/s stop transient reads as 100% slide). Below the
  // floor, standstill logic (ZUPT) owns the estimate, not the slip detector.
  const double r_par = v_wh - v_body;
  const double kappa = r_par / std::max(std::fabs(v_body), 1.0);
  relative_wheel_slide_ = kappa;
  const bool slide_large = std::fabs(kappa) > cfg_.kappa_cut;
  const bool coast_quiet = std::fabs(last_u_.notch) < 0.05 && last_u_.brake < 0.15;
  const bool traction = last_u_.notch > 0.05;
  const bool braking = last_u_.brake > 0.15;
  // Model-ahead-of-wheels on a traction ramp: r<0 under notch. Real slip: r>0.
  const bool physics_slip = (traction && r_par > 0.0) || (braking && r_par < 0.0);
  if (last_sca_.n_inflated == 0 && slide_large && !coast_quiet && physics_slip) {
    kappa_hold_acc_ += std::max(last_dt_s_, 0.0);
  } else if (!(last_sca_.n_inflated == 0 && slide_large && physics_slip)) {
    kappa_hold_acc_ = 0.0;
  }
  last_sca_.common_mode =
      last_sca_.n_inflated == 0 && slide_large && physics_slip &&
      kappa_hold_acc_ >= cfg_.kappa_hold_s;
  if (slide_large) {
    for (int i = 0; i < m; ++i) {
      last_sca_.r_omega[static_cast<std::size_t>(i)] *= cfg_.r_common_mode;
    }
  }
  if (cfg_.encoder_pulses_per_rev > 0) {
    const double n_p = static_cast<double>(cfg_.encoder_pulses_per_rev);
    const double dt_enc = std::max(last_dt_s_, 1e-6);
    const double two_pi = 2.0 * std::acos(-1.0);
    const double sig_w = (two_pi / (n_p * dt_enc)) / std::sqrt(12.0);
    const double extra = sig_w * sig_w;
    for (int i = 0; i < m; ++i) {
      last_sca_.r_omega[static_cast<std::size_t>(i)] += extra;
    }
  }

  const UtWeights ut = make_ut(cfg_);
  const int ns = n_sigma(cfg_);

  double Lchol[kStateDim * kStateDim];
  if (!la::chol(P_, Lchol, kStateDim, 1e-9)) {
    if (!la::project_pd(P_, kStateDim) ||
        !la::chol(P_, Lchol, kStateDim, 1e-9)) {
      ++chol_fail_;
      return;
    }
  }
  double X[kSigma][kStateDim];
  if (cfg_.cubature) {
    const double scale = std::sqrt(static_cast<double>(kStateDim));
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        const double col = scale * la::at(Lchol, kStateDim, j, i);
        X[i][j] = x_[j] + col;
        X[kStateDim + i][j] = x_[j] - col;
      }
    }
  } else {
    for (int j = 0; j < kStateDim; ++j) {
      X[0][j] = x_[j];
    }
    const double scale = std::sqrt(ut.c);
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < kStateDim; ++j) {
        const double col = scale * la::at(Lchol, kStateDim, j, i);
        X[1 + i][j] = x_[j] + col;
        X[1 + kStateDim + i][j] = x_[j] - col;
      }
    }
  }

  double Z[kSigma][kNWheels];
  for (int s = 0; s < ns; ++s) {
    double sp[kStateDim];
    std::memcpy(sp, X[s], sizeof(sp));
    xi_to_phys_p(sp, plant_);
    const double v = sp[kV];
    for (int j = 0; j < m_live; ++j) {
      const int i = live[j];
      const double d = std::max(sp[kD0 + i], 1e-3);
      Z[s][j] = v / (d * plant_.r0_m);
    }
  }
  double zhat[kNWheels];
  la::zero(zhat, m_live);
  for (int s = 0; s < ns; ++s) {
    for (int j = 0; j < m_live; ++j) {
      zhat[j] += ut.wm[s] * Z[s][j];
    }
  }

  double Pzz[kNWheels * kNWheels];
  la::zero(Pzz, m_live * m_live);
  double Pxz[kStateDim * kNWheels];
  std::memset(Pxz, 0, sizeof(Pxz));
  for (int s = 0; s < ns; ++s) {
    double dz[kNWheels];
    double dx[kStateDim];
    for (int j = 0; j < m_live; ++j) {
      dz[j] = Z[s][j] - zhat[j];
    }
    for (int k = 0; k < kStateDim; ++k) {
      dx[k] = X[s][k] - x_[k];
    }
    for (int i = 0; i < m_live; ++i) {
      for (int j = 0; j < m_live; ++j) {
        la::at(Pzz, m_live, i, j) += ut.wc[s] * dz[i] * dz[j];
      }
    }
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < m_live; ++j) {
        Pxz[i * m_live + j] += ut.wc[s] * dx[i] * dz[j];
      }
    }
  }
  for (int j = 0; j < m_live; ++j) {
    la::at(Pzz, m_live, j, j) +=
        last_sca_.r_omega[static_cast<std::size_t>(live[j])];
  }

  double innov[kNWheels];
  for (int j = 0; j < m_live; ++j) {
    innov[j] = omega[live[j]] - zhat[j];
  }
  // Huber/DCS: scale S_ii (already Pzz+R) by max(1, ν²/(c² S_ii)). Caps
  // the information of a locked wheel at full slide so it cannot drag v̂
  // after finite SCA inflate.
  if (cfg_.huber_c > 0.0) {
    const double c2 = cfg_.huber_c * cfg_.huber_c;
    for (int j = 0; j < m_live; ++j) {
      const double sii = std::max(la::at(Pzz, m_live, j, j), 1e-12);
      const double scale = std::max(1.0, (innov[j] * innov[j]) / (c2 * sii));
      la::at(Pzz, m_live, j, j) *= scale;
    }
  }

  double Sinv[kNWheels * kNWheels];
  if (!la::inv_spd(Pzz, Sinv, m_live)) {
    ++chol_fail_;
    return;
  }
  double K[kStateDim * kNWheels];
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < m_live; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m_live; ++k) {
        acc += Pxz[i * m_live + k] * la::at(Sinv, m_live, k, j);
      }
      K[i * m_live + j] = acc;
    }
  }
  double nis = 0.0;
  for (int i = 0; i < m_live; ++i) {
    for (int j = 0; j < m_live; ++j) {
      nis += innov[i] * la::at(Sinv, m_live, i, j) * innov[j];
    }
  }
  last_nis_ = nis;
  nis_valid_ = std::isfinite(nis);
  if (nis_valid_) {
    const double expected =
        static_cast<double>(m_live) + 0.5 * std::sqrt(static_cast<double>(m_live));
    nis_cusum_ = std::max(0.0, nis_cusum_ + last_nis_ - expected);
  }
  for (int i = 0; i < kStateDim; ++i) {
    double acc = 0.0;
    for (int j = 0; j < m_live; ++j) {
      acc += K[i * m_live + j] * innov[j];
    }
    x_[i] += acc;
  }
  double KP[kStateDim * kNWheels];
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < m_live; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m_live; ++k) {
        acc += K[i * m_live + k] * la::at(Pzz, m_live, k, j);
      }
      KP[i * m_live + j] = acc;
    }
  }
  double Pnew[kStateDim * kStateDim];
  std::memcpy(Pnew, P_, sizeof(Pnew));
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < kStateDim; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m_live; ++k) {
        acc += KP[i * m_live + k] * K[j * m_live + k];
      }
      la::at(Pnew, kStateDim, i, j) -= acc;
    }
  }
  if (!la::project_pd(Pnew, kStateDim)) {
    ++chol_fail_;
    return;
  }
  std::memcpy(P_, Pnew, sizeof(P_));
  if (!std::isfinite(x_[kV]) || !std::isfinite(x_[kS])) {
    x_[kV] = v_body;
    x_[kS] = std::max(0.0, x_[kS]);
  }
  const double vmin = cfg_.allow_reverse ? -22.0 : -1.0;
  const double smin = cfg_.allow_reverse ? -1.0e5 : 0.0;
  x_[kV] = std::clamp(x_[kV], vmin, 22.0);
  x_[kS] = std::clamp(x_[kS], smin, 1.0e5);
}

bool Ukf::wheels_at_rest(const double* omega, std::size_t n) const {
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  // A truncated packet cannot prove the vehicle is stopped: one live zero is
  // not four axles at rest (F-22).
  if (m < cfg_.n_wheels || omega == nullptr) {
    return false;
  }
  for (int i = 0; i < m; ++i) {
    if (!std::isfinite(omega[i]) || std::fabs(omega[i]) > kOmegaAbsMax) {
      return false;
    }
    if (std::fabs(omega[i]) >= 0.08) {
      return false;
    }
  }
  return true;
}

bool Ukf::zupt_gate(const double* omega, std::size_t n, const Input& u) const {
  if (!wheels_at_rest(omega, n)) {
    return false;
  }
  const bool notch_quiet = u.notch_valid && std::fabs(u.notch) < 0.05;
  if (!notch_quiet) {
    return false;
  }
  if (std::fabs(x_[kV]) < 0.35) {
    return true;
  }
  // F-01: a short burst of zeros on a moving estimate is a locked-wheel
  // slide, not a stand. F-10: the same zeros held for zupt_omega_only_s with
  // a quiet notch are an orthogonal rest witness (zupt_forced, not silent OK).
  return cfg_.zupt_omega_only_s > 0.0 && omega_zero_s_ >= cfg_.zupt_omega_only_s;
}

bool Ukf::mass_door_allowed() const {
  // No stop map: synth twin / CSV without --route. Keep legacy door impulse.
  if (cfg_.n_stops <= 0) {
    return true;
  }
  const double s = x_[kS];
  const double gate = std::max(cfg_.stop_gate_m, 0.0);
  for (int i = 0; i < cfg_.n_stops && i < kMaxStops; ++i) {
    if (std::fabs(s - cfg_.stop_s_m[static_cast<std::size_t>(i)]) <= gate) {
      return true;
    }
  }
  return false;
}

void Ukf::maybe_zupt(const double* omega, std::size_t n, const Input& u) {
  const bool was = standstill_hold_;
  const double v_before = std::fabs(x_[kV]);
  if (!zupt_gate(omega, n, u)) {
    zupt_acc_ = 0.0;
    if (was && n > 0 && omega != nullptr && n_omega_used_ > 0 && mass_door_allowed()) {
      const double m0 = std::max(plant_.m0_kg, 1.0);
      const double dm = cfg_.mass_door_kg / m0;
      la::at(P_, kStateDim, kMass, kMass) += dm * dm;
    }
    standstill_hold_ = false;
    zupt_estimate_disagree_ = false;
    mode_ = Mode::kNormal;
    return;
  }
  zupt_acc_ += std::max(last_dt_s_, 0.0);
  if (zupt_acc_ < cfg_.zupt_hold_s && !standstill_hold_) {
    return;
  }
  if (v_before >= 0.35) {
    zupt_estimate_disagree_ = true;
  }
  x_[kV] = 0.0;
  for (int i = 0; i < kStateDim; ++i) {
    if (i == kV) {
      continue;
    }
    la::at(P_, kStateDim, kV, i) = 0.0;
    la::at(P_, kStateDim, i, kV) = 0.0;
  }
  la::at(P_, kStateDim, kV, kV) = std::min(la::at(P_, kStateDim, kV, kV), 0.01);
  standstill_hold_ = true;
  mode_ = Mode::kStandstill;
  v_chan_a_ = 0.0;
  have_v_chan_a_ = true;
}

void Ukf::observe_rest_packet(const double* omega, std::size_t n, const Input& u) {
  // ω≈0 on a full packet is standstill / lock evidence, never a rolling-axle
  // measurement. Updating here would drag v̂ to 0 and make the F-10 ω-only
  // witness unreachable (the estimate would already look stopped).
  nis_valid_ = false;
  last_nis_ = 0.0;
  n_omega_used_ = static_cast<int>(n);
  encoder_outage_ = false;
  double phys[kStateDim];
  std::memcpy(phys, x_, sizeof(phys));
  xi_to_phys_p(phys, plant_);
  last_sca_ = sca_analyze(omega, n, &phys[kD0], sca_p_);
  n_frozen_ = n_slip_axles_ = 0;
  std::memset(frozen_, 0, sizeof(frozen_));
  const double denom = std::max(std::fabs(x_[kV]), 1.0);
  relative_wheel_slide_ = (last_sca_.v_consensus_mps - x_[kV]) / denom;
  kappa_hold_acc_ = 0.0;
  note_omega(omega, static_cast<int>(n));
  maybe_zupt(omega, n, u);
}

double Ukf::missed_path_m() const {
  return cfg_.kappa_cut * std::max(std::fabs(x_[kV]), 0.0) *
         std::max(cfg_.kappa_hold_s, 0.0);
}

void Ukf::step_channel_a(const Input& u, double dt_s) {
  if (!have_v_chan_a_ || dt_s <= 0.0) {
    return;
  }
  double phys[kStateDim];
  std::memcpy(phys, x_, sizeof(phys));
  xi_to_phys_p(phys, plant_);
  State s = unpack_state(phys);
  s.v_mps = v_chan_a_;
  // Integrity channel A: do not let F_bias eat a WSP-held brake (the UKF
  // otherwise explains low μ as an extra force and D never grows).
  if (u.brake > 0.15) {
    s.f_bias_n = 0.0;
  }
  const PlantDeriv d = plant_forces(s, u, plant_);
  v_chan_a_ += d.a_mps2 * dt_s;
  const double vmin = cfg_.allow_reverse ? -22.0 : -1.0;
  v_chan_a_ = std::clamp(v_chan_a_, vmin, 22.0);
}

void Ukf::accumulate_path_disagree(const Input& u, double dt_s) {
  if (standstill_hold_ || dt_s <= 0.0 || !have_v_chan_a_) {
    return;
  }
  const double v_w = last_sca_.v_consensus_mps;
  const double resid = v_w - v_chan_a_;
  const bool traction = u.notch > 0.05;
  const bool braking = u.brake > 0.15;
  // Traction: only wheels faster than channel A (true slip). Brake: both
  // signs — WSP-held low-μ has v_A falling faster than the wheels (plant
  // still believes dry Coulomb), which is the bag-day case κ_cut misses.
  const bool physics = (traction && resid > 0.0) || braking;
  const double mag = std::fabs(resid);
  const double floor = std::max(cfg_.path_disagree_floor_mps,
                                cfg_.path_disagree_rel * std::max(std::fabs(v_chan_a_), 1.0));
  if (physics && mag > floor) {
    path_disagree_m_ += mag * dt_s;
  } else if (cfg_.path_disagree_tau_s > 1e-6) {
    path_disagree_m_ *= std::exp(-dt_s / cfg_.path_disagree_tau_s);
  }
  const double al_s = 5.0 + 0.05 * std::max(x_[kS], 0.0);
  if (path_disagree_m_ >= al_s) {
    path_disagree_latched_ = true;
  }
}

void Ukf::apply_mass_prior() {
  if (cfg_.mass_prior_log_sigma <= 1e-9) {
    return;
  }
  // σ=0.3 is a once-per-second prior, not a 50 Hz measurement. Applying it
  // every tick pins log m to ~1 % of m0 (see F9).
  mass_prior_acc_ += std::max(last_dt_s_, 0.0);
  if (mass_prior_acc_ < 1.0) {
    return;
  }
  mass_prior_acc_ = 0.0;
  const double z = std::log(std::max(plant_.m0_kg, 1.0));
  const double R = cfg_.mass_prior_log_sigma * cfg_.mass_prior_log_sigma;
  const double Pmm = la::at(P_, kStateDim, kMass, kMass);
  const double S = Pmm + R;
  if (!(S > 1e-18) || !std::isfinite(S)) {
    return;
  }
  const double innov = z - x_[kMass];
  double K[kStateDim];
  for (int i = 0; i < kStateDim; ++i) {
    K[i] = la::at(P_, kStateDim, i, kMass) / S;
  }
  for (int i = 0; i < kStateDim; ++i) {
    x_[i] += K[i] * innov;
  }
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < kStateDim; ++j) {
      la::at(P_, kStateDim, i, j) -= K[i] * S * K[j];
    }
  }
  if (!la::project_pd(P_, kStateDim)) {
    ++chol_fail_;
  }
}

void Ukf::classify(const Input& u, std::size_t n) {
  const double pss = la::at(P_, kStateDim, kS, kS);
  const double pvv = la::at(P_, kStateDim, kV, kV);
  if (!u.notch_valid) {
    notch_missing_s_ += std::max(last_dt_s_, 0.0);
    ++ticks_no_notch_;
  } else {
    notch_missing_s_ = 0.0;
    ticks_no_notch_ = 0;
  }
  if (!u.brake_valid) {
    brake_missing_s_ += std::max(last_dt_s_, 0.0);
  } else {
    brake_missing_s_ = 0.0;
  }
  const bool sca_current = n_omega_used_ > 0;
  if (sca_current && last_sca_.common_mode) {
    slip_latched_ = true;
  }
  if (path_disagree_latched_) {
    slip_latched_ = true;
  }
  const double s_abs = std::max(x_[kS], 0.0);
  const double sig_s = std::sqrt(std::max(pss, 0.0));
  const bool s_unbounded = slip_latched_ || path_disagree_latched_;
  const double b_s = missed_path_m();
  const double al_s = 5.0 + 0.05 * s_abs;
  const double pl_s = s_unbounded
      ? std::numeric_limits<double>::infinity()
      : cfg_.k_over * sig_s + b_s;
  const double pss_lim = cfg_.k_lost * al_s;
  const int need = cfg_.n_wheels;
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  // A current all-NaN packet (n≥n_wheels) is LOST immediately. Do not wait for
  // age_lost_s: that timer is for predict-only silence, not a dead frame
  // that already arrived. sca_current is false here (n_ok==0), so the
  // n_inflated branch alone would miss it.
  const bool packet_all_dead = m >= need && n_omega_used_ == 0;
  const bool all_inflated =
      packet_all_dead || (sca_current && m >= need && last_sca_.n_inflated >= m);
  encoder_outage_ = n_omega_used_ == 0;
  if (encoder_outage_) wheel_outage_s_ += last_dt_s_;
  else wheel_outage_s_ = 0.0;
  const bool wheel_age_lost = wheel_outage_s_ > cfg_.age_lost_s;
  const bool wheel_age_deg = wheel_outage_s_ > cfg_.age_degraded_s;
  const bool both_down = slip_latched_ && all_inflated;
  const double along = a_kin_ * ((x_[kV] >= 0.0) ? 1.0 : -1.0);
  if (commanded_brake(u) > 0.15 && slip_latched_ && along > cfg_.a_kin_downhill) {
    slide_grade_acc_ += std::max(last_dt_s_, 0.0);
  } else {
    slide_grade_acc_ = 0.0;
  }
  const bool slide_on_grade =
      slide_grade_acc_ > std::max(cfg_.slide_grade_lost_s, 0.0);
  const bool lost = (sig_s > pss_lim) || (notch_missing_s_ > cfg_.notch_lost_s) ||
                    (brake_missing_s_ > cfg_.notch_lost_s) ||
                    all_inflated || both_down || slide_on_grade || wheel_age_lost;
  const bool few = sca_current && n > 0 && n <= 2;
  const bool incomplete_packet = sca_current && m < need;
  int n_frozen_trailer = 0;
  if (sca_current) {
    for (int i = 0; i < m; ++i) {
      if (frozen_[i] && sca_p_.axle_role[static_cast<std::size_t>(i)] == kAxleTrailer) {
        ++n_frozen_trailer;
      }
    }
  }
  const bool trailer_ref = sca_current &&
      last_sca_.used_trailer_consensus && last_sca_.n_trailer_ok >= 1 &&
      commanded_notch(u) > 0.05;
  const bool wheels_split = !sca_current ? false : (trailer_ref
      ? ((last_sca_.n_inflated_trailer >= 1 && last_sca_.n_trailer_ok < 2) ||
         (n_frozen_trailer >= 2))
      : ((few && (last_sca_.n_inflated >= 1 || n_frozen_ >= 1)) ||
         (last_sca_.n_inflated >= 2) || (n_frozen_ >= 2)));
  constexpr double kWheelBodyMps = 0.4;
  // Unanimous wheels (SCA did not inflate anyone) that still disagree with
  // the body: locked zeros, not a single-axle freeze SCA already isolated.
  const bool wheel_body_disagree =
      sca_current && last_sca_.n_inflated == 0 &&
      std::fabs(last_sca_.v_consensus_mps - x_[kV]) > kWheelBodyMps;
  const bool degraded = slip_latched_ || (sca_current && last_sca_.common_mode) ||
                        wheels_split || path_disagree_latched_ || (pvv > 4.0) ||
                        (pl_s >= al_s) || cfg_.r0_uncalibrated || wheel_age_deg ||
                        !u.notch_valid || !u.brake_valid || zupt_estimate_disagree_ ||
                        incomplete_packet || wheel_body_disagree;
  if (lost) {
    confidence_ = Confidence::kLost;
    confidence_v_ = Confidence::kLost;
    confidence_s_ = Confidence::kLost;
  } else if (degraded) {
    confidence_ = Confidence::kDegraded;
    confidence_v_ = (wheels_split || (pvv > 4.0) ||
                     (sca_current && last_sca_.common_mode) ||
                     cfg_.r0_uncalibrated || wheel_age_deg || !u.notch_valid ||
                     !u.brake_valid || zupt_estimate_disagree_ || incomplete_packet ||
                     wheel_body_disagree)
                        ? Confidence::kDegraded : Confidence::kOk;
    confidence_s_ = (slip_latched_ || (sca_current && last_sca_.common_mode) ||
                     path_disagree_latched_ || wheels_split || (pl_s >= al_s) ||
                     cfg_.r0_uncalibrated || wheel_age_deg || !u.notch_valid ||
                     !u.brake_valid || zupt_estimate_disagree_ || incomplete_packet ||
                     wheel_body_disagree)
                        ? Confidence::kDegraded
                        : Confidence::kOk;
  } else {
    confidence_ = Confidence::kOk;
    confidence_v_ = Confidence::kOk;
    confidence_s_ = Confidence::kOk;
  }
  if (standstill_hold_) {
    mode_ = Mode::kStandstill;
    return;
  }
  if (!sca_current) {
    mode_ = Mode::kSensorFault;
    return;
  }
  const double v_body = x_[kV];
  const double v_wh = last_sca_.v_consensus_mps;
  if (n_frozen_ >= 2) {
    mode_ = Mode::kSensorFault;
  } else if (n_slip_axles_ >= 1 ||
             (last_sca_.used_trailer_consensus && last_sca_.n_inflated_motor >= 1 &&
              last_sca_.n_inflated_trailer == 0)) {
    mode_ = Mode::kSlip;
  } else if (last_sca_.n_inflated >= 2) {
    mode_ = Mode::kSensorFault;
  } else if (v_wh > v_body + kWheelBodyMps) {
    mode_ = Mode::kSlip;
  } else if (v_wh + kWheelBodyMps < v_body) {
    mode_ = Mode::kSlide;
  } else {
    mode_ = Mode::kNormal;
  }
}

UkfEstimate Ukf::snapshot() const {
  UkfEstimate e;
  double phys[kStateDim];
  std::memcpy(phys, x_, sizeof(phys));
  xi_to_phys_p(phys, plant_);
  e.x = unpack_state(phys, 0.0);
  State tmp = e.x;
  const PlantDeriv d = (have_f_trac_filt_ &&
                        (plant_.tau_drv_s > 1e-12 || plant_.j_max_mps3 > 1e-12))
                           ? plant_forces(tmp, last_u_, plant_, f_trac_filt_)
                           : plant_forces(tmp, last_u_, plant_);
  e.x.a_mps2 = d.a_mps2;
  e.p_ss = la::at(P_, kStateDim, kS, kS);
  e.p_vv = la::at(P_, kStateDim, kV, kV);
  e.confidence = initialized_ ? confidence_ : Confidence::kUninitialized;
  e.mode = mode_;
  e.sca = last_sca_;
  e.initialized = initialized_;
  e.slip_latched = slip_latched_;
  e.nis = last_nis_;
  e.nis_valid = nis_valid_;
  e.nis_cusum = nis_cusum_;
  e.n_frozen = n_frozen_;
  const double sig = std::sqrt(std::max(e.p_ss, 0.0));
  const double sig_v = std::sqrt(std::max(e.p_vv, 0.0));
  e.under_m = cfg_.k_sigma * sig;
  e.s_unbounded = slip_latched_ || path_disagree_latched_;
  e.b_s_m = e.s_unbounded ? 0.0 : missed_path_m();
  e.pl_s_m = e.s_unbounded ? std::numeric_limits<double>::infinity()
                           : cfg_.k_over * sig + e.b_s_m;
  e.over_m = e.s_unbounded ? 0.0 : e.pl_s_m;
  e.pl_v_mps = cfg_.k_over * sig_v;
  e.al_s_m = 5.0 + 0.05 * std::max(e.x.s_m, 0.0);
  e.a_kin_mps2 = a_kin_;
  e.a_unphysical = a_unphysical_;
  e.confidence_v = initialized_ ? confidence_v_ : Confidence::kUninitialized;
  e.confidence_s = initialized_ ? confidence_s_ : Confidence::kUninitialized;
  e.chol_fail = chol_fail_;
  e.s_unobserved_s = s_unobserved_s_;
  e.n_omega_used = n_omega_used_;
  e.n_slip_axles = n_slip_axles_;
  e.relative_wheel_slide = relative_wheel_slide_;
  e.path_disagree_m = path_disagree_m_;
  e.path_disagree_latched = path_disagree_latched_;
  e.v_chan_a_mps = v_chan_a_;
  e.zupt_at_stop = mass_door_allowed();
  e.p_mm = la::at(P_, kStateDim, kMass, kMass);
  e.zupt_forced = zupt_estimate_disagree_;
  e.sca_current = n_omega_used_ > 0;
  return e;
}

UkfEstimate Ukf::predict_and_update(const Input& u, const double* omega, std::size_t n,
                                   double dt_s) {
  // Input contract: finite controls and 0 < dt <= 0.20 s (subdivide larger
  // known gaps). A violating frame is refused per frame, not silently
  // absorbed: the last finite estimate is kept, the output is LOST, and the
  // estimator recovers when valid inputs resume. Latching until reset() would
  // turn one bad DDS payload into a permanent loss of the backup channel.
  const auto reject = [this]() {
    confidence_ = confidence_v_ = confidence_s_ = Confidence::kLost;
    mode_ = Mode::kSensorFault;
    n_omega_used_ = 0;
    nis_valid_ = false;
    last_nis_ = 0.0;
    auto e = snapshot();
    e.confidence = e.confidence_v = e.confidence_s = Confidence::kLost;
    return e;
  };
  Input u_use = u;
  if (!u_use.notch_valid) {
    u_use.notch = 0.0;
  }
  if (!u_use.brake_valid) {
    u_use.brake = 0.0;
  }
  // Finite check is on the command that will actually be integrated. An
  // invalid NaN payload (stale topic, empty CSV cell) is coast, not LOST.
  // A *valid* NaN/Inf still rejects: that is a poisoned live command.
  if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > kMaxStepS ||
      !std::isfinite(u_use.notch) || !std::isfinite(u_use.brake)) {
    return reject();
  }
  const Ukf previous = *this;  // atomic step: rollback a numerically invalid result
  n = omega ? std::min(n, static_cast<std::size_t>(kNWheels)) : 0;
  n_omega_used_ = 0;
  nis_valid_ = false;
  last_nis_ = 0.0;
  last_u_ = u_use;
  last_dt_s_ = dt_s;
  if (n > 0 && omega != nullptr && wheels_at_rest(omega, n) && u_use.notch_valid &&
      std::fabs(u_use.notch) < 0.05) {
    omega_zero_s_ += dt_s;
  } else {
    omega_zero_s_ = 0.0;
  }
  if (!initialized_) {
    if (n == 0 || omega == nullptr) {
      return snapshot();
    }
    int n_ok = 0;
    const int m0 = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
    for (int i = 0; i < m0; ++i) {
      if (std::isfinite(omega[i]) && std::fabs(omega[i]) <= kOmegaAbsMax) {
        ++n_ok;
      }
    }
    if (n_ok == 0) {
      return snapshot();
    }
    init_from_wheels(omega, n);
  }
  predict(u_use, dt_s);
  step_channel_a(u_use, dt_s);
  if (n > 0) {
    const bool at_rest = wheels_at_rest(omega, n);
    const bool notch_quiet = u_use.notch_valid && std::fabs(u_use.notch) < 0.05;
    const bool lock_at_speed = at_rest && std::fabs(x_[kV]) >= 0.35;
    if ((at_rest && notch_quiet) || lock_at_speed) {
      observe_rest_packet(omega, n, u_use);
    } else {
      update_wheels(omega, n);
      maybe_zupt(omega, n, u_use);
      apply_mass_prior();
      if (n_omega_used_ > 0) accumulate_path_disagree(u_use, dt_s);
    }
  } else {
    maybe_zupt(nullptr, 0, u_use);  // invalidate a hold; missing data is not departure
  }
  bool healthy = chol_fail_ == previous.chol_fail_;
  for (double value : x_) healthy = healthy && std::isfinite(value);
  for (double value : P_) healthy = healthy && std::isfinite(value);
  healthy = healthy && std::isfinite(f_trac_filt_) && std::isfinite(v_chan_a_) &&
            std::isfinite(path_disagree_m_) && std::isfinite(last_nis_);
  double check[kStateDim * kStateDim];
  if (healthy) healthy = la::chol(P_, check, kStateDim, 1e-12);
  if (!healthy) {
    *this = previous;
    ++chol_fail_;
    return reject();
  }
  if (have_v_prev_ && dt_s > 1e-6) {
    a_kin_ = (x_[kV] - v_prev_) / dt_s;
  } else {
    a_kin_ = 0.0;
  }
  have_v_prev_ = true;
  v_prev_ = x_[kV];
  // Coulomb cap ~ μ g; a constant 3 m/s² false-flags dry emergency braking.
  double phys_a[kStateDim];
  std::memcpy(phys_a, x_, sizeof(phys_a));
  xi_to_phys_p(phys_a, plant_);
  const double a_lim = 1.2 * std::clamp(phys_a[kMu], kMuMin, kMuMax) * plant_.g;
  a_unphysical_ = std::fabs(a_kin_) > a_lim;
  classify(u_use, n);
  if (slip_latched_ || n_omega_used_ == 0) {
    s_unobserved_s_ += dt_s;
  }
  return snapshot();
}

UkfEstimate Ukf::predict_and_update(const Input& u, const double* omega, std::size_t n) {
  return predict_and_update(u, omega, n, kDtDefault);
}

}  // namespace tram_dr

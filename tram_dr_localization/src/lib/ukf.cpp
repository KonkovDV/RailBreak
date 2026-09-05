#include "tram_dr_localization/ukf.hpp"

#include "tram_dr_localization/lin_alg.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace tram_dr {
namespace {

constexpr double kDtDefault = 0.02;

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
  const bool coast = std::fabs(u.notch) < 0.05 && u.brake < 0.1;
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

Ukf::Ukf(UkfParams cfg) : cfg_(cfg) { reset(); }

void Ukf::set_params(const UkfParams& cfg) { cfg_ = cfg; }

void Ukf::set_plant(const PlantParams& p) { plant_ = p; }

void Ukf::set_sca(const ScaParams& p) { sca_p_ = p; }

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
    ++chol_fail_;
    double f_lag = have_f_trac_filt_ ? f_trac_filt_ : 0.0;
    step_sigma_xi(x_, u, dt_s, plant_, &f_lag);
    f_trac_filt_ = f_lag;
    have_f_trac_filt_ = true;
    add_process_q(P_, dt_s, cfg_, plant_, u, standstill_hold_, last_sca_);
    for (int i = 0; i < kStateDim; ++i) {
      la::at(P_, kStateDim, i, i) += 1e-3;
    }
    la::project_pd(P_, kStateDim);
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
  la::project_pd(Pnew, kStateDim);
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
  double w[kNWheels]{};
  int n_ok = 0;
  for (int i = 0; i < m; ++i) {
    w[i] = omega[i];
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
    s_unobserved_s_ += std::max(last_dt_s_, 0.0);
    return;
  }
  encoder_outage_ = false;
  wheel_outage_s_ = 0.0;
  double phys[kStateDim];
  std::memcpy(phys, x_, sizeof(phys));
  xi_to_phys_p(phys, plant_);
  sca_p_.traction = last_u_.notch > 0.05;
  last_sca_ = sca_analyze(omega, static_cast<std::size_t>(m), &phys[kD0], sca_p_);
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(omega[i]) && std::fabs(omega[i]) <= kOmegaAbsMax) {
      continue;
    }
    const double d = std::max(phys[kD0 + i], kDMin);
    w[i] = phys[kV] / (d * plant_.r0_m);
  }
  note_omega(w, m);
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
  // Denominator floor 1 m/s: relative wheel slide κ=(rω−v)/v is degenerate near
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
    la::project_pd(P_, kStateDim);
    if (!la::chol(P_, Lchol, kStateDim, 1e-9)) {
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
    for (int i = 0; i < m; ++i) {
      const double d = std::max(sp[kD0 + i], 1e-3);
      Z[s][i] = v / (d * plant_.r0_m);
    }
  }
  double zhat[kNWheels];
  la::zero(zhat, m);
  for (int s = 0; s < ns; ++s) {
    for (int i = 0; i < m; ++i) {
      zhat[i] += ut.wm[s] * Z[s][i];
    }
  }

  double Pzz[kNWheels * kNWheels];
  la::zero(Pzz, m * m);
  double Pxz[kStateDim * kNWheels];
  std::memset(Pxz, 0, sizeof(Pxz));
  for (int s = 0; s < ns; ++s) {
    double dz[kNWheels];
    double dx[kStateDim];
    for (int i = 0; i < m; ++i) {
      dz[i] = Z[s][i] - zhat[i];
    }
    for (int j = 0; j < kStateDim; ++j) {
      dx[j] = X[s][j] - x_[j];
    }
    for (int i = 0; i < m; ++i) {
      for (int j = 0; j < m; ++j) {
        la::at(Pzz, m, i, j) += ut.wc[s] * dz[i] * dz[j];
      }
    }
    for (int i = 0; i < kStateDim; ++i) {
      for (int j = 0; j < m; ++j) {
        Pxz[i * m + j] += ut.wc[s] * dx[i] * dz[j];
      }
    }
  }
  for (int i = 0; i < m; ++i) {
    la::at(Pzz, m, i, i) += last_sca_.r_omega[static_cast<std::size_t>(i)];
  }

  double innov[kNWheels];
  for (int i = 0; i < m; ++i) {
    innov[i] = w[i] - zhat[i];
  }
  // Huber/DCS: Reff = R max(1, ν²/(c² S)). Caps the information of a locked
  // wheel at full slide so it cannot drag v̂ after finite SCA inflate.
  if (cfg_.huber_c > 0.0) {
    const double c2 = cfg_.huber_c * cfg_.huber_c;
    for (int i = 0; i < m; ++i) {
      const double sii = std::max(la::at(Pzz, m, i, i), 1e-12);
      const double scale = std::max(1.0, (innov[i] * innov[i]) / (c2 * sii));
      la::at(Pzz, m, i, i) *= scale;
    }
  }

  double Sinv[kNWheels * kNWheels];
  if (!la::inv_spd(Pzz, Sinv, m)) {
    return;
  }
  // K = Pxz * Sinv  (L x m)
  double K[kStateDim * kNWheels];
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < m; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m; ++k) {
        acc += Pxz[i * m + k] * la::at(Sinv, m, k, j);
      }
      K[i * m + j] = acc;
    }
  }
  double nis = 0.0;
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      nis += innov[i] * la::at(Sinv, m, i, j) * innov[j];
    }
  }
  last_nis_ = nis;
  nis_valid_ = std::isfinite(nis);
  if (nis_valid_) {
    const double expected =
        static_cast<double>(m) + 0.5 * std::sqrt(static_cast<double>(m));
    nis_cusum_ = std::max(0.0, nis_cusum_ + last_nis_ - expected);
  }
  for (int i = 0; i < kStateDim; ++i) {
    double acc = 0.0;
    for (int j = 0; j < m; ++j) {
      acc += K[i * m + j] * innov[j];
    }
    x_[i] += acc;
  }
  // P = P - K Pzz K^T
  double KP[kStateDim * kNWheels];
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < m; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m; ++k) {
        acc += K[i * m + k] * la::at(Pzz, m, k, j);
      }
      KP[i * m + j] = acc;
    }
  }
  double Pnew[kStateDim * kStateDim];
  std::memcpy(Pnew, P_, sizeof(Pnew));
  for (int i = 0; i < kStateDim; ++i) {
    for (int j = 0; j < kStateDim; ++j) {
      double acc = 0.0;
      for (int k = 0; k < m; ++k) {
        acc += KP[i * m + k] * K[j * m + k];
      }
      la::at(Pnew, kStateDim, i, j) -= acc;
    }
  }
  la::project_pd(Pnew, kStateDim);
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

bool Ukf::zupt_gate(const double* omega, std::size_t n, const Input& u) const {
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  if (m <= 0 || omega == nullptr) {
    return false;
  }
  double wmax = 0.0;
  bool any = false;
  for (int i = 0; i < m; ++i) {
    if (!std::isfinite(omega[i]) || std::fabs(omega[i]) > kOmegaAbsMax) {
      continue;
    }
    any = true;
    wmax = std::max(wmax, std::fabs(omega[i]));
  }
  if (!any) {
    return false;
  }
  const double vabs = std::fabs(x_[kV]);
  // Hold-brake standstill is real; locked sliding is not (v still large).
  return wmax < 0.08 && std::fabs(u.notch) < 0.05 && (u.brake < 0.15 || vabs < 0.35);
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
  if (!zupt_gate(omega, n, u)) {
    zupt_acc_ = 0.0;
    if (was && mass_door_allowed()) {
      // Passenger exchange at a named stop, not a wait at a temporary switch.
      const double m0 = std::max(plant_.m0_kg, 1.0);
      const double dm = cfg_.mass_door_kg / m0;
      la::at(P_, kStateDim, kMass, kMass) += dm * dm;
    }
    standstill_hold_ = false;
    return;
  }
  zupt_acc_ += std::max(last_dt_s_, 0.0);
  if (zupt_acc_ < cfg_.zupt_hold_s && !standstill_hold_) {
    return;
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

double Ukf::missed_path_m() const {
  if (slip_latched_ || path_disagree_latched_) {
    return 1.0e6;
  }
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
  la::project_pd(P_, kStateDim);
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
  if (last_sca_.common_mode) {
    slip_latched_ = true;
  }
  if (path_disagree_latched_) {
    slip_latched_ = true;
  }
  const double s_abs = std::max(x_[kS], 0.0);
  const double sig_s = std::sqrt(std::max(pss, 0.0));
  const double b_s = missed_path_m();
  const double pl_s = cfg_.k_over * sig_s + b_s;
  const double al_s = 5.0 + 0.05 * s_abs;
  const double pss_lim = cfg_.k_lost * al_s;
  const int m = static_cast<int>(std::min(n, static_cast<std::size_t>(kNWheels)));
  const bool all_inflated = m >= 4 && last_sca_.n_inflated >= m;
  if (encoder_outage_ || (n > 0 && n_omega_used_ == 0)) {
    wheel_outage_s_ += std::max(last_dt_s_, 0.0);
  } else if (n > 0) {
    wheel_outage_s_ = 0.0;
  }
  const bool wheel_age_lost = wheel_outage_s_ > cfg_.age_lost_s;
  const bool wheel_age_deg = wheel_outage_s_ > cfg_.age_degraded_s;
  const bool both_down = slip_latched_ && all_inflated;
  const double along = a_kin_ * ((x_[kV] >= 0.0) ? 1.0 : -1.0);
  if (u.brake > 0.15 && slip_latched_ && along > cfg_.a_kin_downhill) {
    slide_grade_acc_ += std::max(last_dt_s_, 0.0);
  } else {
    slide_grade_acc_ = 0.0;
  }
  const bool slide_on_grade =
      slide_grade_acc_ > std::max(cfg_.slide_grade_lost_s, 0.0);
  const bool lost = (sig_s > pss_lim) || (notch_missing_s_ > cfg_.notch_lost_s) ||
                    all_inflated || both_down || slide_on_grade || wheel_age_lost;
  const bool few = n > 0 && n <= 2;
  int n_frozen_trailer = 0;
  for (int i = 0; i < m; ++i) {
    if (frozen_[i] && sca_p_.axle_role[static_cast<std::size_t>(i)] == kAxleTrailer) {
      ++n_frozen_trailer;
    }
  }
  const bool trailer_ref =
      last_sca_.used_trailer_consensus && last_sca_.n_trailer_ok >= 1 &&
      u.notch > 0.05;
  const bool wheels_split = trailer_ref
      ? ((last_sca_.n_inflated_trailer >= 1 && last_sca_.n_trailer_ok < 2) ||
         (n_frozen_trailer >= 2))
      : ((few && (last_sca_.n_inflated >= 1 || n_frozen_ >= 1)) ||
         (last_sca_.n_inflated >= 2) || (n_frozen_ >= 2));
  const bool degraded = slip_latched_ || last_sca_.common_mode || wheels_split ||
                        path_disagree_latched_ || (pvv > 4.0) || (pl_s >= al_s) ||
                        cfg_.r0_uncalibrated || wheel_age_deg;
  if (lost) {
    confidence_ = Confidence::kLost;
    confidence_v_ = Confidence::kLost;
    confidence_s_ = Confidence::kLost;
  } else if (degraded) {
    confidence_ = Confidence::kDegraded;
    confidence_v_ = wheels_split || (pvv > 4.0) ? Confidence::kDegraded : Confidence::kOk;
    confidence_s_ = (slip_latched_ || last_sca_.common_mode || path_disagree_latched_ ||
                     cfg_.r0_uncalibrated || wheel_age_deg)
                        ? Confidence::kDegraded
                        : Confidence::kOk;
  } else {
    confidence_ = Confidence::kOk;
    confidence_v_ = Confidence::kOk;
    confidence_s_ = Confidence::kOk;
  }
  if (mode_ == Mode::kStandstill) {
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
  } else if (v_wh > v_body + 0.4) {
    mode_ = Mode::kSlip;
  } else if (v_wh + 0.4 < v_body) {
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
  e.b_s_m = missed_path_m();
  e.pl_s_m = cfg_.k_over * sig + e.b_s_m;
  e.over_m = e.pl_s_m;
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
  return e;
}

UkfEstimate Ukf::predict_and_update(const Input& u, const double* omega, std::size_t n,
                                   double dt_s) {
  last_u_ = u;
  last_dt_s_ = dt_s;
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
  predict(u, dt_s);
  step_channel_a(u, dt_s);
  if (n > 0) {
    if (zupt_gate(omega, n, u)) {
      // ω=0 is not a rolling-axle measurement. Updating here lets Davis-at-rest
      // leak into F_bias (unobservable vs hold brake / grade).
      nis_valid_ = false;
      last_nis_ = 0.0;
      maybe_zupt(omega, n, u);
    } else {
      update_wheels(omega, n);
      maybe_zupt(omega, n, u);
      apply_mass_prior();
      accumulate_path_disagree(u, dt_s);
    }
  } else {
    nis_valid_ = false;
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
  classify(u, n);
  if (slip_latched_) {
    s_unobserved_s_ += std::max(dt_s, 0.0);
  }
  return snapshot();
}

UkfEstimate Ukf::predict_and_update(const Input& u, const double* omega, std::size_t n) {
  return predict_and_update(u, omega, n, kDtDefault);
}

}  // namespace tram_dr

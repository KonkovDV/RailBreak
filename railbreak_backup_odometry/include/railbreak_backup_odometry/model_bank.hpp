// Shadow model bank: five longitudinal hypotheses for mismatch diagnostics.
// Not an IMM and not GPB2. The mixture does not replace the filter state,
// and the spread is not written into P. fuse_modes compares absolute s.
// Coast and braking stay inside a_tab(notch): the bank does not add a second
// copy of those rows.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace railbreak {

inline constexpr int kModelCount = 5;
inline constexpr int kModelNominal = 0;
inline constexpr int kModelDelay = 1;
inline constexpr int kModelAdhesion = 2;
inline constexpr int kModelWheelScale = 3;
inline constexpr int kModelMismatch = 4;

inline const char* model_mode_name(int mode) {
  switch (mode) {
    case kModelNominal: return "NOMINAL";
    case kModelDelay: return "ACTUATOR_DELAY";
    case kModelAdhesion: return "LOW_ADHESION";
    case kModelWheelScale: return "WHEEL_SCALE";
    case kModelMismatch: return "MODEL_MISMATCH";
    default: return "NOMINAL";
  }
}

struct ModeEstimate {
  double s = 0.0;
  double v = 0.0;
  double p_ss = 1.0;
  double p_sv = 0.0;
  double p_vv = 0.25;
  double innovation = 0.0;
  double likelihood = 0.0;
  double confidence = 0.2;
};

struct ModelConsensus {
  double s = 0.0;
  double v = 0.0;
  double p_ss = 1.0;
  double p_vv = 0.25;
  int leader = kModelNominal;
  int n_used = kModelCount;
  bool outlier_rejected = false;
  ModeEstimate mode[kModelCount];
};

// Probability-weighted information fusion. A mode whose arc is far from the
// current leader is left out, so the result is not the mean of every hypothesis.
inline ModelConsensus fuse_modes(const ModeEstimate* mode, double spread_gate_m) {
  ModelConsensus out;
  int leader = 0;
  for (int j = 1; j < kModelCount; ++j)
    if (mode[j].confidence > mode[leader].confidence) leader = j;
  out.leader = leader;
  double info_s = 0.0, info_v = 0.0, prec_s = 0.0, prec_v = 0.0;
  int used = 0;
  for (int j = 0; j < kModelCount; ++j) {
    out.mode[j] = mode[j];
    if (std::fabs(mode[j].s - mode[leader].s) > spread_gate_m) {
      out.outlier_rejected = true;
      continue;
    }
    const double ws = mode[j].confidence / std::max(mode[j].p_ss, 1.0e-4);
    const double wv = mode[j].confidence / std::max(mode[j].p_vv, 1.0e-6);
    prec_s += ws;
    prec_v += wv;
    info_s += ws * mode[j].s;
    info_v += wv * mode[j].v;
    ++used;
  }
  out.n_used = used;
  out.s = prec_s > 0.0 ? info_s / prec_s : mode[leader].s;
  out.v = prec_v > 0.0 ? info_v / prec_v : mode[leader].v;
  out.p_ss = prec_s > 0.0 ? 1.0 / prec_s : mode[leader].p_ss;
  out.p_vv = prec_v > 0.0 ? 1.0 / prec_v : mode[leader].p_vv;
  return out;
}

class ModelBank {
 public:
  static constexpr double kAdhesionCap = 0.40;  // hypothesis, m/s^2, not an estimated mu
  static constexpr double kWheelScale = 1.05;   // one fixed radius hypothesis
  static constexpr double kMismatchQ = 20.0;    // process-noise multiplier
  static constexpr double kSpreadGateM = 8.0;
  static constexpr double kDelayS = 0.30;

  void init(double s, double v) {
    for (int j = 0; j < kModelCount; ++j) {
      mode_[j] = ModeEstimate{};
      mode_[j].s = s;
      mode_[j].v = std::max(0.0, v);
      mode_[j].confidence = 1.0 / kModelCount;
    }
    consensus_ = fuse_modes(mode_, kSpreadGateM);
    ready_ = true;
  }

  bool ready() const { return ready_; }
  const ModelConsensus& consensus() const { return consensus_; }

  // u_mps is the bogie speed in the same units as v. a_now and a_delayed are
  // longitudinal accelerations. meas_var is the variance of u_mps.
  void step(double dt, double a_now, double a_delayed, double u_mps, double meas_var) {
    if (!ready_ || !(dt > 0.0) || !std::isfinite(u_mps) || !std::isfinite(meas_var) || !(meas_var > 0.0))
      return;
    const double q_s = 1.0e-4;
    const double q_v = 0.05;
    const double pi_stay = 0.90;
    const double pi_switch = (1.0 - pi_stay) / (kModelCount - 1);
    double c[kModelCount] = {};
    for (int j = 0; j < kModelCount; ++j)
      for (int i = 0; i < kModelCount; ++i)
        c[j] += mode_[i].confidence * (i == j ? pi_stay : pi_switch);

    double mix = 0.0;
    double like[kModelCount];
    for (int j = 0; j < kModelCount; ++j) {
      predict(j, dt, a_now, a_delayed, q_s, q_v);
      const double z = j == kModelWheelScale ? u_mps / kWheelScale : u_mps;
      const double nu = z - mode_[j].v;
      const double S = std::max(mode_[j].p_vv + meas_var, 1.0e-9);
      const double Ks = mode_[j].p_sv / S;
      const double Kv = mode_[j].p_vv / S;
      const double p_sv = mode_[j].p_sv;
      const double p_ss = mode_[j].p_ss;
      mode_[j].s += Ks * nu;
      mode_[j].v = std::max(0.0, mode_[j].v + Kv * nu);
      mode_[j].p_vv = std::max((1.0 - Kv) * mode_[j].p_vv, 1.0e-9);
      mode_[j].p_sv = p_sv * meas_var / S;
      mode_[j].p_ss = std::max(p_ss - Ks * p_sv, 1.0e-6);
      mode_[j].innovation = nu;
      const double quad = std::min(nu * nu / S, 40.0);
      like[j] = std::exp(-0.5 * quad) / std::sqrt(2.0 * 3.14159265358979323846 * S);
      like[j] = std::max(like[j], 1.0e-12);
      mode_[j].likelihood = like[j];
      mix += c[j] * like[j];
    }
    if (!(mix > 0.0)) mix = 1.0;
    for (int j = 0; j < kModelCount; ++j)
      mode_[j].confidence = (c[j] * like[j]) / mix;
    consensus_ = fuse_modes(mode_, kSpreadGateM);
  }

 private:
  void predict(int j, double dt, double a_now, double a_delayed, double q_s, double q_v) {
    double a = a_now;
    if (j == kModelDelay) a = a_delayed;
    if (j == kModelAdhesion) a = std::clamp(a_now, -kAdhesionCap, kAdhesionCap);
    const double qv = j == kModelMismatch ? q_v * kMismatchQ : q_v;
    const double v = mode_[j].v;
    const double v_new = std::clamp(v + a * dt, 0.0, 30.0);
    mode_[j].s += 0.5 * (v + v_new) * dt;
    mode_[j].v = v_new;
    const double p_sv = mode_[j].p_sv;
    const double p_vv = mode_[j].p_vv;
    mode_[j].p_ss += 2.0 * dt * p_sv + dt * dt * p_vv + q_s * dt;
    mode_[j].p_sv = p_sv + dt * p_vv;
    mode_[j].p_vv = p_vv + qv * dt;
  }

  ModeEstimate mode_[kModelCount]{};
  ModelConsensus consensus_{};
  bool ready_ = false;
};

}  // namespace railbreak

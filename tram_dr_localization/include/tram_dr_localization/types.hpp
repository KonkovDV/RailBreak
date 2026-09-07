#pragma once
// Units: SI. No ROS types. kNWheels is a compile-time max (6 axles / Витязь-М).
// Runtime n_wheels may be 4 (Combino twin). L = 6+kNWheels. Longitudinal a is diagnostic.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace tram_dr {

constexpr int kNWheels = 6;
constexpr int kMaxStops = 16;  // route_10 stop vertices; not i(s)
constexpr int kStateDim = 6 + kNWheels;  // 12
constexpr int kSigma = 2 * kStateDim + 1;
constexpr double kDMin = 0.85;
constexpr double kDMax = 1.05;
constexpr double kKtracMin = 0.5;
constexpr double kKtracMax = 1.5;
constexpr double kMuMin = 0.05;
constexpr double kMuMax = 0.5;
// Default clip: Combino twin / Витязь. Львёнок YAML сужает до [15, 40] т.
constexpr double kMassMinKg = 20000.0;
constexpr double kMassMaxKg = 70000.0;
constexpr double kOmegaAbsMax = 80.0;
// 0.0.11: numerical/plant contracts and time-driven mass prior.
constexpr const char* kModelVersion = "tramDR-0.0.11";

enum StateIndex {
  kS = 0,
  kV = 1,
  kFbias = 2,
  kMass = 3,
  kKtrac = 4,
  kD0 = 5,  // d_i scale of wheel radius, i=0..kNWheels-1
  kMu = 5 + kNWheels
};

struct State {
  double s_m{0.0};
  double v_mps{0.0};
  double a_mps2{0.0};  // diagnostic F_net / (m (1+γ)), not a UKF column
  double f_bias_n{0.0};
  double m_eff_kg{28000.0};
  double k_trac{1.0};
  std::array<double, kNWheels> d{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
  double mu_hat{0.35};
};

struct Input {
  double notch{0.0};  // [-1, 1] after mapping
  double brake{0.0};  // [0, 1]
  bool notch_valid{true};
  bool brake_valid{true};
};

// Stale / missing commands must not keep driving the plant as if they were live.
inline double commanded_notch(const Input& u) {
  return u.notch_valid ? u.notch : 0.0;
}
inline double commanded_brake(const Input& u) {
  return u.brake_valid ? u.brake : 0.0;
}

// How a raw controller value is mapped onto [-1, 1].
// kAuto: |raw|≤1 pass-through (Combino already-normalized); else raw/N.
// kNormalized: always clamp to [-1, 1]; never divide (1.001 stays ~1, not 0.125).
// kDiscrete: always raw/N (Int8 ±8 → 1 is 8, not 1). RB08-10: auto was
// non-monotonic on {0,1,2,8}.
enum class NotchEncoding { kAuto = 0, kNormalized = 1, kDiscrete = 2 };

inline double map_notch(double raw, double notch_max_abs = 8.0,
                        NotchEncoding enc = NotchEncoding::kAuto) {
  if (!std::isfinite(raw)) {
    return raw;  // propagate: never turn NaN/Inf into a plausible idle
  }
  const double m = std::max(std::fabs(notch_max_abs), 1.0);
  if (enc == NotchEncoding::kDiscrete) {
    return std::clamp(raw / m, -1.0, 1.0);
  }
  if (enc == NotchEncoding::kNormalized) {
    return std::clamp(raw, -1.0, 1.0);
  }
  if (std::fabs(raw) <= 1.0) {
    return std::clamp(raw, -1.0, 1.0);
  }
  return std::clamp(raw / m, -1.0, 1.0);
}

inline void pack_state(const State& x, double* v) {
  v[kS] = x.s_m;
  v[kV] = x.v_mps;
  v[kFbias] = x.f_bias_n;
  v[kMass] = x.m_eff_kg;
  v[kKtrac] = x.k_trac;
  for (int i = 0; i < kNWheels; ++i) {
    v[kD0 + i] = x.d[static_cast<std::size_t>(i)];
  }
  v[kMu] = x.mu_hat;
}

inline State unpack_state(const double* v, double a_diag = 0.0) {
  State x;
  x.s_m = v[kS];
  x.v_mps = v[kV];
  x.f_bias_n = v[kFbias];
  x.m_eff_kg = v[kMass];
  x.k_trac = v[kKtrac];
  for (int i = 0; i < kNWheels; ++i) {
    x.d[static_cast<std::size_t>(i)] = v[kD0 + i];
  }
  x.mu_hat = v[kMu];
  x.a_mps2 = a_diag;
  return x;
}

inline double sigmoid(double z) {
  if (z > 20.0) {
    return 1.0;
  }
  if (z < -20.0) {
    return 0.0;
  }
  return 1.0 / (1.0 + std::exp(-z));
}

inline double logit01(double p) {
  p = std::clamp(p, 1e-6, 1.0 - 1e-6);
  return std::log(p / (1.0 - p));
}

// UKF coordinates: log m, log k, log d_i, logit((μ-0.05)/0.45). s, v, F_bias stay SI.
// μ in x is a clip+RW, not an adhesion observer (no torque, no ω̇).
inline void phys_to_xi(double* x) {
  x[kMass] = std::log(std::max(x[kMass], 1.0));
  x[kKtrac] = std::log(std::max(x[kKtrac], 1e-6));
  for (int i = 0; i < kNWheels; ++i) {
    x[kD0 + i] = std::log(std::max(x[kD0 + i], 1e-6));
  }
  const double u = (std::clamp(x[kMu], kMuMin, kMuMax) - kMuMin) / (kMuMax - kMuMin);
  x[kMu] = logit01(u);
}

inline void xi_to_phys(double* x, double mass_min_kg = kMassMinKg,
                       double mass_max_kg = kMassMaxKg) {
  const double lo = std::log(std::max(mass_min_kg, 1.0));
  const double hi = std::log(std::max(mass_max_kg, mass_min_kg + 1.0));
  x[kMass] = std::exp(std::clamp(x[kMass], lo, hi));
  x[kKtrac] = std::exp(std::clamp(x[kKtrac], std::log(kKtracMin), std::log(kKtracMax)));
  for (int i = 0; i < kNWheels; ++i) {
    x[kD0 + i] = std::clamp(std::exp(x[kD0 + i]), kDMin, kDMax);
  }
  x[kMu] = kMuMin + (kMuMax - kMuMin) * sigmoid(x[kMu]);
}

}  // namespace tram_dr

#pragma once

#include <algorithm>
#include <cmath>

namespace railbreak {

// Continuous per-bogie fault score. One NIS sample does not change the mode.
// F = exp(-dt/tau) * F + (1 - exp(-dt/tau)) * phi(NIS).
struct FaultObs {
  double t = 0.0;
  double nis_front = 0.0;
  double nis_rear = 0.0;
  double nis_gate = 16.0;
  double model_residual_mps = 0.0;
  double front_rear_residual_mps = 0.0;
  double fr_floor = 0.3;
  bool pair_fresh = false;
  bool bogies_agree = true;
};

struct FaultState {
  double fault_score = 0.0;
  double front_score = 0.0;
  double rear_score = 0.0;
  double common_score = 0.0;
  double recovery_score = 0.0;
  double fault_duration_s = 0.0;
  double deep_duration_s = 0.0;
  double recovery_duration_s = 0.0;
  bool front_latched = false;
  bool rear_latched = false;
  bool common_latched = false;
  const char* level = "nominal";
};

class FaultScore {
 public:
  static constexpr double kTauS = 0.25;
  static constexpr double kEnter = 0.8;
  static constexpr double kEnterS = 0.5;
  static constexpr double kDeep = 0.95;
  static constexpr double kDeepS = 2.0;
  static constexpr double kClear = 0.2;
  static constexpr double kClearS = 3.0;
  static constexpr double kDtCapS = 1.0;

  FaultState update(const FaultObs& o) {
    double dt = 0.0;
    if (!have_t_) {
      have_t_ = true;
      t_ = o.t;
    } else if (o.t > t_) {
      dt = std::min(o.t - t_, kDtCapS);
      t_ = o.t;
    }
    const double lam = dt > 0.0 ? std::exp(-dt / kTauS) : 1.0;
    const double phi_f = phi(o.nis_front, o, true);
    const double phi_r = phi(o.nis_rear, o, false);
    front_ = lam * front_ + (1.0 - lam) * phi_f;
    rear_ = lam * rear_ + (1.0 - lam) * phi_r;

    const double score = std::max(front_, rear_);
    const bool agree = o.pair_fresh && o.bogies_agree;
    const double common = agree ? std::min(front_, rear_) : 0.0;
    arm(above_since_, score > kEnter, o.t);
    arm(deep_since_, score > kDeep, o.t);
    arm(low_since_, score < kClear, o.t);
    const double above = elapsed(above_since_, o.t);
    const double deep = elapsed(deep_since_, o.t);
    const double low = elapsed(low_since_, o.t);

    if (!front_latched_ && front_ > kEnter && above >= kEnterS) front_latched_ = true;
    if (!rear_latched_ && rear_ > kEnter && above >= kEnterS) rear_latched_ = true;
    if (front_latched_ && front_ < kClear && low >= kClearS) front_latched_ = false;
    if (rear_latched_ && rear_ < kClear && low >= kClearS) rear_latched_ = false;

    FaultState s;
    s.fault_score = score;
    s.front_score = front_;
    s.rear_score = rear_;
    s.common_score = common;
    s.recovery_score = 1.0 - score;
    s.fault_duration_s = above;
    s.deep_duration_s = deep;
    s.recovery_duration_s = low;
    s.front_latched = front_latched_;
    s.rear_latched = rear_latched_;
    s.common_latched = agree && front_latched_ && rear_latched_ && common > kEnter;
    if ((front_latched_ || rear_latched_) && deep >= kDeepS) s.level = "deep";
    else if ((front_latched_ || rear_latched_) && score < kClear) s.level = "recovering";
    else if (front_latched_ || rear_latched_) s.level = "degraded";
    else s.level = "nominal";
    return s;
  }

 private:
  static double phi(double nis, const FaultObs& o, bool is_front) {
    double p = 0.0;
    if (o.nis_gate > 0.0 && nis > 0.0) p = nis / o.nis_gate;
    if (o.pair_fresh && !o.bogies_agree) {
      const bool worse = is_front ? o.nis_front >= o.nis_rear : o.nis_rear > o.nis_front;
      if (worse) p = std::max(p, 1.0);
    }
    if (o.fr_floor > 0.0 && o.front_rear_residual_mps > o.fr_floor) {
      const bool worse = is_front ? o.nis_front >= o.nis_rear : o.nis_rear > o.nis_front;
      if (worse) p = std::max(p, std::min(1.0, o.front_rear_residual_mps / o.fr_floor));
    }
    if (o.bogies_agree && o.fr_floor > 0.0 && std::fabs(o.model_residual_mps) > 0.0)
      p = std::max(p, std::min(1.0, std::fabs(o.model_residual_mps) / o.fr_floor));
    return std::min(1.0, std::max(0.0, p));
  }

  static void arm(double& since, bool on, double t) {
    if (!on) since = -1.0;
    else if (since < 0.0 || t < since) since = t;
  }

  static double elapsed(double since, double t) {
    return since < 0.0 ? 0.0 : std::max(0.0, t - since);
  }

  double front_ = 0.0;
  double rear_ = 0.0;
  double t_ = 0.0;
  bool have_t_ = false;
  bool front_latched_ = false;
  bool rear_latched_ = false;
  double above_since_ = -1.0;
  double deep_since_ = -1.0;
  double low_since_ = -1.0;
};

}  // namespace railbreak

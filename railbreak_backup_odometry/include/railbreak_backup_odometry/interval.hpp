// Conservative position and speed envelope. Not a percentile and not a
// protection level. Coverage against ground truth is not filled in here:
// an interval that has not been counted on held-out truth is not a 95% interval.
#pragma once

#include <algorithm>
#include <cmath>

namespace railbreak {

struct MotionInterval {
  double s_hat = 0.0;
  double v_hat = 0.0;
  double sigma_s = 0.0;
  double sigma_v = 0.0;
  // Named contributions. They are not written into P_ss.
  double sigma_model = 0.0;
  double sigma_map = 0.0;
  double sigma_scale = 0.0;
  double sigma_common_mode = 0.0;
  double sigma_timestamp = 0.0;
  double e_s = 0.0;
  double e_v = 0.0;
  double s_min = 0.0;
  double s_max = 0.0;
  double v_min = 0.0;
  double v_max = 0.0;
  bool valid = false;
  // Always false. The half-width is not a measured 50/90/95/99% coverage.
  bool claimed_percentile = false;
};

// along_bound is B_s = q_0.99 * sigma_s + B_mode + B_map + B_time.
// That number is an empirical bound, not a certified protection level.
// The five named terms are added beside it. They are not folded into P_ss.
// sigma_timestamp is sigma_v times the unverified duration.
// sigma_common_mode is passed in by the caller and is not also the unverified term.
// LOST publishes no interval.
inline MotionInterval motion_interval(double s, double v, double sigma_s, double sigma_v,
                                     double sigma_k, double along_bound, bool bound_valid,
                                     double distance_since_anchor, double unverified_s,
                                     double model_gap_s, double model_gap_v, bool lost,
                                     double sigma_map_m = 0.0, double sigma_common_mode_m = 0.0) {
  MotionInterval m;
  m.s_hat = s;
  m.v_hat = v;
  m.sigma_s = sigma_s;
  m.sigma_v = sigma_v;
  m.claimed_percentile = false;
  if (lost || !std::isfinite(s) || !std::isfinite(v) || !std::isfinite(sigma_s) ||
      !std::isfinite(sigma_v)) {
    m.s_min = m.s_max = m.v_min = m.v_max = std::nan("");
    return m;
  }
  const double cov_s = bound_valid && std::isfinite(along_bound) ? std::max(along_bound, 0.0) : 0.0;
  m.sigma_scale = std::fabs(distance_since_anchor) * std::max(sigma_k, 0.0);
  m.sigma_model = std::fabs(model_gap_s);
  m.sigma_map = std::isfinite(sigma_map_m) ? std::max(sigma_map_m, 0.0) : 0.0;
  m.sigma_common_mode =
      std::isfinite(sigma_common_mode_m) ? std::max(sigma_common_mode_m, 0.0) : 0.0;
  m.sigma_timestamp = std::max(unverified_s, 0.0) * std::max(sigma_v, 0.0);
  m.e_s = std::max(cov_s + m.sigma_scale + m.sigma_model + m.sigma_map + m.sigma_common_mode +
                       m.sigma_timestamp,
                   std::max(sigma_s, 0.0));
  m.e_v = std::max(sigma_v, 0.0) + std::fabs(v) * std::max(sigma_k, 0.0) + std::fabs(model_gap_v);
  m.s_min = s - m.e_s;
  m.s_max = s + m.e_s;
  m.v_min = v - m.e_v;
  m.v_max = v + m.e_v;
  m.valid = true;
  return m;
}

// Seconds of further silence until both bogie ages pass max_gap. Zero when
// the estimate is already LOST. This is not a measured time-to-alarm on recordings.
inline double time_to_lost(bool lost, double front_age_s, double rear_age_s, double max_gap_s) {
  if (lost) return 0.0;
  const double fresher = std::min(front_age_s, rear_age_s);
  if (!std::isfinite(fresher) || !std::isfinite(max_gap_s)) return 0.0;
  return std::max(0.0, max_gap_s - fresher);
}

}  // namespace railbreak

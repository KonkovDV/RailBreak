// Shadow integrity monitor for the backup odometer.
//
// It does not write the filter state. Status rules are duplicated in
// tools/organizer/integrity.py; change them together.
//
// The along-track number is an empirical bound:
//   q_0.99 * sigma_s + B_mode + B_time + B_map
// It is not a certified protection level.
#pragma once

#include <cmath>
#include <string>

namespace railbreak {

struct BoundCoeff {
  bool calibrated = false;
  double q99 = 0.0;
  double b_nominal = 0.0;
  double b_single = 0.0;
  double b_model = 0.0;
  double b_common = 0.0;
  double b_no_map = 0.0;
  double b_relative = 0.0;
  double b_untrusted = 0.0;
  double b_time_m = 0.0;
  double b_map_m = 0.0;
  const char* coverage = "0.99";
  const char* split = "train";
};

struct IntegrityObs {
  double t = 0.0;
  double sigma_s = 0.0;
  bool slip_front = false;
  bool slip_rear = false;
  double nis_front = 0.0;
  double nis_rear = 0.0;
  double front_age_s = 1.0e9;
  double rear_age_s = 1.0e9;
  bool pair_fresh = false;
  bool bogies_agree = false;
  bool stamp_regressed = false;
  bool absolute_start = false;
  bool map_in_domain = true;
  bool dwell = false;
  int station_candidates = 0;
  int n_anchor = 0;
  double nis_gate = 16.0;
  double wheel_stale_s = 0.35;
  double max_gap_s = 30.0;
  double recover_s = 3.0;
};

struct IntegrityReport {
  const char* status = "NOMINAL";
  std::string reasons;
  double along_bound_m = 0.0;
  bool bound_valid = false;
  bool use_position = true;
  bool certification_claim = false;
  const char* calibrated_coverage = "null";
  const char* calibration_split = "train";
  const char* bound_name = "empirical along-track integrity bound";
};

inline void add_reason(std::string& out, const char* reason) {
  if (!out.empty()) out.push_back(',');
  out += reason;
}

// Reads an observation and remembers only its own latch. coeff is not modified.
class IntegrityMonitor {
 public:
  explicit IntegrityMonitor(BoundCoeff coeff = {}) : coeff_(coeff) {}

  const BoundCoeff& coeff() const { return coeff_; }

  IntegrityReport update(const IntegrityObs& o) {
    const bool front_nis = o.nis_front > o.nis_gate;
    const bool rear_nis = o.nis_rear > o.nis_gate;
    const bool front_stale = o.front_age_s > o.wheel_stale_s;
    const bool rear_stale = o.rear_age_s > o.wheel_stale_s;
    const bool front_bad = o.slip_front || front_nis || front_stale;
    const bool rear_bad = o.slip_rear || rear_nis || rear_stale;
    const bool active = o.pair_fresh && o.bogies_agree && o.slip_front && o.slip_rear;
    if (active) {
      if (common_since_ < 0.0) common_since_ = o.t;
      if (o.t - common_since_ >= o.recover_s) latch_on(o.n_anchor);
    } else {
      if (common_since_ >= 0.0 && o.t - common_since_ >= o.recover_s) latch_on(o.n_anchor);
      common_since_ = -1.0;
    }
    if (latched_ && o.n_anchor > anchors_at_latch_) latch_off();
    if (latched_ && o.pair_fresh && !o.bogies_agree) latch_off();
    const bool common = active || latched_;

    IntegrityReport r;
    r.certification_claim = false;
    r.calibration_split = coeff_.split;
    r.calibrated_coverage = coeff_.calibrated ? coeff_.coverage : "null";
    if (o.stamp_regressed || (front_stale && rear_stale && o.front_age_s > o.max_gap_s &&
                              o.rear_age_s > o.max_gap_s) ||
        (!o.absolute_start && !o.map_in_domain)) {
      r.status = "POSITION_UNTRUSTED";
    } else if (!o.map_in_domain) {
      r.status = "DEGRADED_NO_MAP";
    } else if (!o.absolute_start) {
      r.status = "DEGRADED_RELATIVE_ONLY";
    } else if (common) {
      r.status = "DEGRADED_COMMON_MODE_UNOBSERVABLE";
    } else if (front_bad && rear_bad) {
      r.status = "DEGRADED_MODEL_CARRY";
    } else if (front_bad || rear_bad) {
      r.status = "DEGRADED_SINGLE_BOGIE";
    } else {
      r.status = "NOMINAL";
    }

    if (front_nis) add_reason(r.reasons, "FRONT_NIS_HIGH");
    if (rear_nis) add_reason(r.reasons, "REAR_NIS_HIGH");
    if (o.pair_fresh && !o.bogies_agree) add_reason(r.reasons, "BOGIES_DISAGREE");
    if (common) add_reason(r.reasons, "BOGIES_AGREE_MODEL_DISAGREES");
    if (front_stale) add_reason(r.reasons, "STALE_FRONT");
    if (rear_stale) add_reason(r.reasons, "STALE_REAR");
    if (o.stamp_regressed) add_reason(r.reasons, "STAMP_REGRESSION");
    if (!o.absolute_start) add_reason(r.reasons, "NO_ABSOLUTE_START");
    if (o.dwell && o.station_candidates > 1) add_reason(r.reasons, "AMBIGUOUS_STATION_ANCHOR");
    if (!o.map_in_domain) add_reason(r.reasons, "MAP_OUT_OF_DOMAIN");

    const double gap = std::min(o.front_age_s, o.rear_age_s);
    double b_mode = coeff_.b_nominal;
    if (r.status == std::string("DEGRADED_SINGLE_BOGIE")) b_mode = coeff_.b_single;
    else if (r.status == std::string("DEGRADED_MODEL_CARRY")) b_mode = coeff_.b_model;
    else if (r.status == std::string("DEGRADED_COMMON_MODE_UNOBSERVABLE")) b_mode = coeff_.b_common;
    else if (r.status == std::string("DEGRADED_NO_MAP")) b_mode = coeff_.b_no_map;
    else if (r.status == std::string("DEGRADED_RELATIVE_ONLY")) b_mode = coeff_.b_relative;
    else if (r.status == std::string("POSITION_UNTRUSTED")) b_mode = coeff_.b_untrusted;
    const double b_time = gap > o.wheel_stale_s ? coeff_.b_time_m : 0.0;
    const double b_map = o.map_in_domain ? 0.0 : coeff_.b_map_m;
    if (coeff_.calibrated && std::isfinite(o.sigma_s) && o.sigma_s >= 0.0) {
      r.along_bound_m = coeff_.q99 * o.sigma_s + b_mode + b_time + b_map;
      r.bound_valid = std::isfinite(r.along_bound_m);
    }
    // The coordinate is for use only while the status is not a refusal.
    // An uncalibrated bound does not by itself forbid the coordinate.
    r.use_position = r.status != std::string("POSITION_UNTRUSTED");
    return r;
  }

 private:
  void latch_on(int n_anchor) {
    if (!latched_) anchors_at_latch_ = n_anchor;
    latched_ = true;
  }
  void latch_off() { latched_ = false; }

  BoundCoeff coeff_{};
  double common_since_ = -1.0;
  bool latched_ = false;
  int anchors_at_latch_ = 0;
};

}  // namespace railbreak

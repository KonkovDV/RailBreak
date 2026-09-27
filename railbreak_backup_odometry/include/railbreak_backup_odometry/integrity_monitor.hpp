// Shadow integrity monitor for the backup odometer.
//
// It does not write the filter state. Status rules are duplicated in
// tools/organizer/integrity.py; change them together.
//
// The along-track number is an empirical bound:
//   q_0.99 * sigma_s + B_mode + B_time + B_map
// It is not a certified protection level.
#pragma once

#include "railbreak_backup_odometry/fault_score.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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
  // Set by the filter after recover_s. Agreement of the bogies is not trust.
  bool common_unobservable = false;
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
  // Operational budget while both bogies and the model disagree.
  // Counted from the last accepted anchor, not from the start of the ride.
  // Not a protection distance.
  double distance_since_anchor = 0.0;
  double max_blind_time_s = 5.0;
  double max_blind_distance_m = 100.0;
  // Slip classes latch when FaultScore says so: 0.5 s to enter, 3 s to clear.
  // There is no second confirm timer on that transition.
  // degrade_recover_s holds a non-slip change, such as the return from
  // DEGRADED_NO_MAP or DEGRADED_RELATIVE_ONLY toward NOMINAL.
  // 0 keeps that change immediate. That is how the fitted bound was checked.
  double degrade_recover_s = 0.0;
  double model_residual_mps = 0.0;
  double front_rear_residual_mps = 0.0;
  double fr_floor = 0.3;
};

struct IntegrityReport {
  const char* status = "NOMINAL";
  std::string reasons;
  double along_bound_m = 0.0;
  bool bound_valid = false;
  bool use_position = true;
  const char* confidence_velocity = "ok";
  const char* confidence_position = "ok";
  const char* velocity_confidence = "HIGH";
  const char* position_confidence = "HIGH";
  const char* front_wheel_confidence = "HIGH";
  const char* rear_wheel_confidence = "HIGH";
  const char* model_confidence = "HIGH";
  const char* integrity_mode = "NOMINAL";
  const char* blind_warning = "";
  double blind_time_s = 0.0;
  double fault_score = 0.0;
  double fault_duration_s = 0.0;
  double recovery_score = 0.0;
  double recovery_duration_s = 0.0;
  double common_score = 0.0;
  const char* fault_level = "nominal";
  // Seconds left on the common-mode blind clock. 0 once that clock has
  // reached LOST. NaN while the clock is not running. Not the 30 s silence clock.
  double time_to_lost = std::numeric_limits<double>::quiet_NaN();
  double distance_since_last_trusted_anchor = 0.0;
  bool certification_claim = false;
  const char* calibrated_coverage = "null";
  const char* calibration_split = "train";
  const char* bound_name = "empirical along-track integrity bound";
  // Exact claim. The number is not a protection level.
  const char* bound_statement = "empirical bound, not certified protection level";
};

inline void add_reason(std::string& out, const char* reason) {
  if (!out.empty()) out.push_back(',');
  out += reason;
}

inline const char* trust_level(bool missing, bool degraded) {
  if (missing) return "NONE";
  if (degraded) return "LOW";
  return "HIGH";
}

// Separate channels. LOST means the estimate misses the claimed trust, not that
// the node stopped. A cleared slip can leave velocity HIGH and position LOW.
inline void write_trust(IntegrityReport& r, bool front_bad, bool rear_bad, double front_age,
                        double rear_age, double max_gap, bool active, bool position_held,
                        bool absolute_start, bool map_in_domain, bool over_budget) {
  const bool front_none = front_age > max_gap;
  const bool rear_none = rear_age > max_gap;
  r.front_wheel_confidence = trust_level(front_none, front_bad);
  r.rear_wheel_confidence = trust_level(rear_none, rear_bad);
  const bool front_high = !front_none && !front_bad;
  const bool rear_high = !rear_none && !rear_bad;
  if (front_none && rear_none) r.velocity_confidence = "NONE";
  else if (active || (!front_high && !rear_high)) r.velocity_confidence = "LOW";
  else r.velocity_confidence = "HIGH";
  const bool refused = r.status == std::string("POSITION_UNTRUSTED") || r.status == std::string("LOST");
  if (refused) r.position_confidence = "NONE";
  else if (position_held || active || !absolute_start || !map_in_domain) r.position_confidence = "LOW";
  else r.position_confidence = "HIGH";
  if (refused || (front_none && rear_none)) r.model_confidence = "NONE";
  else if (active || (!front_high && !rear_high)) r.model_confidence = "LOW";
  else r.model_confidence = "HIGH";
  const bool vel_none = r.velocity_confidence[0] == 'N';
  const bool pos_none = r.position_confidence[0] == 'N';
  if (vel_none || pos_none) r.integrity_mode = "LOST";
  else if (active && over_budget) r.integrity_mode = "SAFE_EXTRAPOLATION";
  else if (active) r.integrity_mode = "VELOCITY_DEGRADED";
  else if (!front_high && !rear_high) r.integrity_mode = "MODEL_ASSISTED";
  else if (r.position_confidence[0] == 'L') r.integrity_mode = "POSITION_DEGRADED";
  else if (!front_high || !rear_high) r.integrity_mode = "WHEEL_DEGRADED";
  else r.integrity_mode = "NOMINAL";
  r.blind_warning = std::strcmp(r.integrity_mode, "SAFE_EXTRAPOLATION") == 0
                        ? "position extrapolated past the last anchor"
                        : "";
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
    FaultObs fin;
    fin.t = o.t;
    fin.nis_front = o.nis_front;
    fin.nis_rear = o.nis_rear;
    fin.nis_gate = o.nis_gate;
    fin.model_residual_mps = o.model_residual_mps;
    fin.front_rear_residual_mps = o.front_rear_residual_mps;
    fin.fr_floor = o.fr_floor;
    fin.pair_fresh = o.pair_fresh;
    fin.bogies_agree = o.bogies_agree;
    const FaultState fs = fault_.update(fin);
    // Stale is an age fact. Slip and NIS enter the mode only through the score.
    const bool front_bad = fs.front_latched || front_stale;
    const bool rear_bad = fs.rear_latched || rear_stale;
    const bool active = fs.common_latched;
    const bool common = active || o.common_unobservable;
    if (o.n_anchor != last_anchor_) {
      common_since_ = -1.0;
      blind_since_ = -1.0;
      last_anchor_ = o.n_anchor;
    }
    if (active) {
      if (common_since_ < 0.0) common_since_ = o.t;
      if (o.t - common_since_ >= o.recover_s) latch_on(o.n_anchor);
    } else {
      if (common_since_ >= 0.0 && o.t - common_since_ >= o.recover_s) latch_on(o.n_anchor);
      common_since_ = -1.0;
    }
    const bool blind_on = o.common_unobservable || fs.common_latched;
    if (blind_on) {
      if (blind_since_ < 0.0) blind_since_ = o.t;
    } else {
      blind_since_ = -1.0;
    }
    if (latched_ && o.n_anchor > anchors_at_latch_) latch_off();
    if (latched_ && o.pair_fresh && !o.bogies_agree && !o.common_unobservable) latch_off();
    // `active` is the live slip. The latch is only the open position error.
    const bool position_held = latched_;
    const double live_time =
        (active && common_since_ >= 0.0) ? std::max(0.0, o.t - common_since_) : 0.0;
    const double blind_time = blind_on && blind_since_ >= 0.0 ? std::max(0.0, o.t - blind_since_)
                                                              : live_time;
    const bool over_budget =
        common && ((std::isfinite(o.max_blind_time_s) && blind_time > o.max_blind_time_s) ||
                   (std::isfinite(o.max_blind_distance_m) &&
                    o.distance_since_anchor > o.max_blind_distance_m));

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
    } else if (common && over_budget) {
      // Common mode spent the blind budget. This is not a bound class.
      r.status = "LOST";
    } else if (common) {
      r.status = "DEGRADED_COMMON_MODE_UNOBSERVABLE";
    } else if (front_bad && rear_bad) {
      r.status = "DEGRADED_MODEL_CARRY";
    } else if (front_bad || rear_bad) {
      r.status = "DEGRADED_SINGLE_BOGIE";
    } else {
      r.status = "NOMINAL";
    }
    r.status = latch_status(r.status, o);

    if (front_nis) add_reason(r.reasons, "FRONT_NIS_HIGH");
    if (rear_nis) add_reason(r.reasons, "REAR_NIS_HIGH");
    if (o.pair_fresh && !o.bogies_agree) add_reason(r.reasons, "BOGIES_DISAGREE");
    if (common) add_reason(r.reasons, "BOGIES_AGREE_MODEL_DISAGREES");
    if (over_budget) add_reason(r.reasons, "BLIND_BUDGET");
    if (position_held && !common) add_reason(r.reasons, "POSITION_OPEN");
    if (front_stale) add_reason(r.reasons, "STALE_FRONT");
    if (rear_stale) add_reason(r.reasons, "STALE_REAR");
    if (o.stamp_regressed) add_reason(r.reasons, "STAMP_REGRESSION");
    if (!o.absolute_start) add_reason(r.reasons, "NO_ABSOLUTE_START");
    if (o.dwell && o.station_candidates > 1) add_reason(r.reasons, "AMBIGUOUS_STATION_ANCHOR");
    if (!o.map_in_domain) add_reason(r.reasons, "MAP_OUT_OF_DOMAIN");
    if (std::strcmp(fs.level, "deep") == 0) add_reason(r.reasons, "DEEP_FAULT");

    const double gap = std::min(o.front_age_s, o.rear_age_s);
    double b_mode = coeff_.b_nominal;
    if (r.status == std::string("DEGRADED_SINGLE_BOGIE")) b_mode = coeff_.b_single;
    else if (r.status == std::string("DEGRADED_MODEL_CARRY")) b_mode = coeff_.b_model;
    else if (r.status == std::string("DEGRADED_COMMON_MODE_UNOBSERVABLE")) b_mode = coeff_.b_common;
    else if (r.status == std::string("DEGRADED_NO_MAP")) b_mode = coeff_.b_no_map;
    else if (r.status == std::string("DEGRADED_RELATIVE_ONLY")) b_mode = coeff_.b_relative;
    else if (r.status == std::string("POSITION_UNTRUSTED")) b_mode = coeff_.b_untrusted;
    if (position_held) b_mode = std::max(b_mode, coeff_.b_common);
    const double b_time = gap > o.wheel_stale_s ? coeff_.b_time_m : 0.0;
    const double b_map = o.map_in_domain ? 0.0 : coeff_.b_map_m;
    const bool lost = r.status == std::string("LOST");
    if (!lost && coeff_.calibrated && std::isfinite(o.sigma_s) && o.sigma_s >= 0.0) {
      r.along_bound_m = coeff_.q99 * o.sigma_s + b_mode + b_time + b_map;
      r.bound_valid = std::isfinite(r.along_bound_m);
    }
    // The coordinate is for use only while the status is not a refusal.
    // An uncalibrated bound does not by itself forbid the coordinate.
    // LOST is the common-mode refusal. POSITION_UNTRUSTED is stamp, gap, or no start.
    const bool refused = lost || r.status == std::string("POSITION_UNTRUSTED");
    r.use_position = !refused;
    r.confidence_velocity = (front_bad || rear_bad || common) ? "degraded" : "ok";
    if (refused) r.confidence_position = "untrusted";
    else if (position_held || common || !o.absolute_start || !o.map_in_domain)
      r.confidence_position = "degraded";
    else r.confidence_position = "ok";
    r.blind_time_s = blind_time;
    r.fault_score = fs.fault_score;
    r.fault_duration_s = fs.fault_duration_s;
    r.recovery_score = fs.recovery_score;
    r.recovery_duration_s = fs.recovery_duration_s;
    r.common_score = fs.common_score;
    r.fault_level = (common && over_budget) ? "lost" : fs.level;
    r.distance_since_last_trusted_anchor = o.distance_since_anchor;
    if (common && over_budget) r.time_to_lost = 0.0;
    else if (common && std::isfinite(o.max_blind_time_s))
      r.time_to_lost = std::max(0.0, o.max_blind_time_s - blind_time);
    write_trust(r, front_bad, rear_bad, o.front_age_s, o.rear_age_s, o.max_gap_s, common,
                position_held, o.absolute_start, o.map_in_domain, over_budget && !o.common_unobservable);
    return r;
  }

 private:
  const char* latch_status(const char* raw, const IntegrityObs& o) {
    const std::string next = raw;
    const bool safety = next == "POSITION_UNTRUSTED" || next == "LOST" ||
                        next == "DEGRADED_NO_MAP" || next == "DEGRADED_RELATIVE_ONLY" ||
                        latched_status_ == "POSITION_UNTRUSTED" || latched_status_ == "LOST";
    if (safety || next == latched_status_) {
      latched_status_ = next;
      pending_status_ = next;
      pending_since_ = o.t;
      return status_literal(latched_status_);
    }
    if (next != pending_status_) {
      pending_status_ = next;
      pending_since_ = o.t;
    }
    const bool slip_class = next.rfind("DEGRADED_SINGLE", 0) == 0 ||
                            next == "DEGRADED_MODEL_CARRY" ||
                            next == "DEGRADED_COMMON_MODE_UNOBSERVABLE" ||
                            latched_status_.rfind("DEGRADED_SINGLE", 0) == 0 ||
                            latched_status_ == "DEGRADED_MODEL_CARRY" ||
                            latched_status_ == "DEGRADED_COMMON_MODE_UNOBSERVABLE";
    // Slip classes follow FaultScore: 0.5 s to enter, 3 s to clear.
    // degrade_recover_s is not that timer. It holds only a non-slip change.
    const double need = slip_class ? 0.0 : o.degrade_recover_s;
    if (!(need > 0.0) || (std::isfinite(o.t) && pending_since_ >= 0.0 &&
                          o.t - pending_since_ + 1e-12 >= need))
      latched_status_ = next;
    return status_literal(latched_status_);
  }

  static const char* status_literal(const std::string& s) {
    if (s == "LOST") return "LOST";
    if (s == "POSITION_UNTRUSTED") return "POSITION_UNTRUSTED";
    if (s == "DEGRADED_NO_MAP") return "DEGRADED_NO_MAP";
    if (s == "DEGRADED_RELATIVE_ONLY") return "DEGRADED_RELATIVE_ONLY";
    if (s == "DEGRADED_COMMON_MODE_UNOBSERVABLE") return "DEGRADED_COMMON_MODE_UNOBSERVABLE";
    if (s == "DEGRADED_MODEL_CARRY") return "DEGRADED_MODEL_CARRY";
    if (s == "DEGRADED_SINGLE_BOGIE") return "DEGRADED_SINGLE_BOGIE";
    return "NOMINAL";
  }

  void latch_on(int n_anchor) {
    if (!latched_) anchors_at_latch_ = n_anchor;
    latched_ = true;
  }
  void latch_off() { latched_ = false; }

  BoundCoeff coeff_{};
  double common_since_ = -1.0;
  double blind_since_ = -1.0;
  bool latched_ = false;
  int anchors_at_latch_ = 0;
  int last_anchor_ = -1;
  std::string latched_status_ = "NOMINAL";
  std::string pending_status_ = "NOMINAL";
  double pending_since_ = -1.0;
  FaultScore fault_{};
};

// Diagnostic latch only. It does not write the filter and integrity.py does not
// mirror it. Wheel agreement never sets the exit: only a new accepted anchor does.
struct CommonModeExit {
  bool latched = false;
  int anchors_at_entry = 0;
  const char* exit = "none";
};

inline void note_common_mode_exit(CommonModeExit& state, bool common, int n_anchor) {
  if (common && !state.latched) {
    state.latched = true;
    state.anchors_at_entry = n_anchor;
    state.exit = "none";
  } else if (!common && state.latched) {
    state.latched = false;
    if (n_anchor > state.anchors_at_entry) state.exit = "anchor";
  }
}

}  // namespace railbreak

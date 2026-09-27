// Adhesion anomaly proxy. It does not write the filter and does not estimate mu.
//
// Motor torque, motor current, an IMU, an independent body speed and the
// contact forces are not inputs. Coefficient of adhesion is not observable
// from two wheel speeds and driver command alone. mu_estimate stays null.
//
// The label is a residual pattern: both bogies can agree with each other and
// disagree with the model because of low adhesion, or because the table,
// grade or bias is wrong. Those causes are not separated here.
#pragma once

#include <cmath>
#include <sstream>
#include <string>

namespace railbreak {

struct AdhesionObs {
  double t = 0.0;
  bool pair_fresh = false;
  bool bogies_agree = false;
  bool slip_front = false;
  bool slip_rear = false;
  bool have_consensus = false;
  double wheel_consensus_residual = 0.0;
  bool have_model = false;
  double model_consistency_residual = 0.0;
  double front_nis = 0.0;
  double rear_nis = 0.0;
  int notch = 0;
  double speed = 0.0;
};

struct AdhesionReport {
  const char* classification = "NO_FRESH_PAIR";
  const char* mu_estimate = "null";
  const char* reason = "no fresh bogie pair";
  double duration_s = 0.0;
  double common_mode_duration_s = 0.0;
  bool have_consensus = false;
  double wheel_consensus_residual = 0.0;
  bool have_model = false;
  double model_consistency_residual = 0.0;
  double front_nis = 0.0;
  double rear_nis = 0.0;
  int notch = 0;
  double speed = 0.0;
  bool mu_observable = false;

  std::string json() const {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(4);
    out << "{\"classification\":\"" << classification << "\","
        << "\"mu_estimate\":null,"
        << "\"mu_observable\":false,"
        << "\"reason\":\"" << reason << "\","
        << "\"duration_s\":" << duration_s << ",";
    out << "\"wheel_consensus_residual\":";
    write_optional(out, have_consensus, wheel_consensus_residual);
    out << ",\"model_consistency_residual\":";
    write_optional(out, have_model, model_consistency_residual);
    out << ",\"front_nis\":" << front_nis
        << ",\"rear_nis\":" << rear_nis
        << ",\"common_mode_duration_s\":" << common_mode_duration_s
        << ",\"notch\":" << notch
        << ",\"speed\":" << speed << "}";
    return out.str();
  }

 private:
  static void write_optional(std::ostringstream& out, bool have, double value) {
    if (!have || !std::isfinite(value)) out << "null";
    else out << value;
  }
};

// Remembers only the start of the current common-mode episode.
class AdhesionProxy {
 public:
  AdhesionReport update(const AdhesionObs& o) {
    const bool common = o.pair_fresh && o.bogies_agree && o.slip_front && o.slip_rear;
    double duration = 0.0;
    if (common) {
      if (since_ < 0.0) since_ = o.t;
      duration = std::max(0.0, o.t - since_);
    } else {
      since_ = -1.0;
    }

    AdhesionReport r;
    r.mu_estimate = "null";
    r.mu_observable = false;
    r.duration_s = duration;
    r.common_mode_duration_s = duration;
    r.have_consensus = o.have_consensus;
    r.wheel_consensus_residual = o.wheel_consensus_residual;
    r.have_model = o.have_model;
    r.model_consistency_residual = o.model_consistency_residual;
    r.front_nis = o.front_nis;
    r.rear_nis = o.rear_nis;
    r.notch = o.notch;
    r.speed = o.speed;
    if (!o.pair_fresh) {
      r.classification = "NO_FRESH_PAIR";
      r.reason = "no fresh bogie pair";
    } else if (!o.bogies_agree) {
      r.classification = "BOGIES_DISAGREE";
      r.reason = "bogies differ from each other";
    } else if (common) {
      r.classification = "COMMON_MODE_SUSPECTED";
      r.reason = "both bogies agree but differ from model";
    } else if (o.slip_front || o.slip_rear) {
      r.classification = "SINGLE_BOGIE_ANOMALY";
      r.reason = "one bogie differs from the model";
    } else {
      r.classification = "CONSISTENT";
      r.reason = "bogies agree with each other and with the model";
    }
    return r;
  }

 private:
  double since_ = -1.0;
};

}  // namespace railbreak

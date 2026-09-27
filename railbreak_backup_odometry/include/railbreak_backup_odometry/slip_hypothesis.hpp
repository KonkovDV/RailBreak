// Slip versus model mismatch. One residual is not a slip score.
// The filter still down-weights a high NIS immediately. This diagnosis is the
// separate, hysteretic label: it does not write the filter state.
#pragma once

#include <cstring>

namespace railbreak {

struct SlipEvidence {
  double t = 0.0;
  double front_innov = 0.0;  // wheel minus model, m/s. Positive: the wheel is faster.
  double rear_innov = 0.0;
  bool pair_fresh = false;
  bool bogies_agree = false;
  double nis_front = 0.0;
  double nis_rear = 0.0;
  double nis_gate = 16.0;
  double nis_exit = 4.0;
  int notch = 0;
  bool delay_likely = false;
};

inline const char* residual_pattern(const SlipEvidence& e, double gate) {
  if (!e.pair_fresh) return "no_pair";
  const bool front_out = e.nis_front > gate;
  const bool rear_out = e.nis_rear > gate;
  if (!e.bogies_agree) {
    if (front_out && rear_out) return "chaotic";
    if (front_out || rear_out) return "single_bogie";
    return "chaotic";
  }
  if (front_out || rear_out) return "common";
  return "nominal";
}

// Sign is used only when the bogies agree, the notch is traction or braking,
// and the delay hypothesis is not the better explanation.
inline const char* instant_hypothesis(const SlipEvidence& e, const char* pattern) {
  if (std::strcmp(pattern, "nominal") == 0 || std::strcmp(pattern, "no_pair") == 0) return "nominal";
  if (std::strcmp(pattern, "chaotic") == 0) return "sensor_fault";
  if (std::strcmp(pattern, "single_bogie") == 0) return "single_bogie";
  if (e.delay_likely) return "delay";
  const double innov = 0.5 * (e.front_innov + e.rear_innov);
  if (e.notch > 0 && innov > 0.0) return "spin";
  if (e.notch < 0 && innov < 0.0) return "slide";
  return "model_mismatch";
}

struct SlipReport {
  const char* pattern = "no_pair";
  const char* hypothesis = "nominal";
  const char* phase = "idle";
};

class SlipDiagnosis {
 public:
  static constexpr double kCandidateS = 0.15;
  static constexpr double kConfirmS = 0.40;
  static constexpr double kRecoverS = 1.0;

  SlipReport update(const SlipEvidence& e) {
    const bool held = std::strcmp(confirmed_, "nominal") != 0;
    const double gate = held ? e.nis_exit : e.nis_gate;
    const char* pattern = residual_pattern(e, gate);
    const char* raw = instant_hypothesis(e, pattern);
    if (std::strcmp(raw, candidate_) != 0 || candidate_since_ < 0.0 || e.t < candidate_since_) {
      candidate_ = raw;
      candidate_since_ = e.t;
    }
    const double age = std::max(0.0, e.t - candidate_since_);
    const bool nominal = std::strcmp(raw, "nominal") == 0;
    const bool below_exit = e.nis_front <= e.nis_exit && e.nis_rear <= e.nis_exit;

    if (held && nominal && below_exit && e.bogies_agree) {
      if (recover_since_ < 0.0 || e.t < recover_since_) recover_since_ = e.t;
      if (e.t - recover_since_ >= kRecoverS) {
        confirmed_ = "nominal";
        recover_since_ = -1.0;
        phase_ = "idle";
      } else {
        phase_ = "recovery";
      }
    } else {
      recover_since_ = -1.0;
      if (!nominal && age >= kConfirmS) {
        confirmed_ = raw;
        phase_ = "confirmed";
      } else if (!nominal && age >= kCandidateS && !held) {
        phase_ = "candidate";
      } else if (held) {
        phase_ = "confirmed";
      } else {
        phase_ = "idle";
      }
    }

    SlipReport r;
    r.pattern = pattern;
    if (std::strcmp(phase_, "confirmed") == 0 || std::strcmp(phase_, "recovery") == 0)
      r.hypothesis = confirmed_;
    else if (std::strcmp(phase_, "candidate") == 0)
      r.hypothesis = candidate_;
    else
      r.hypothesis = "nominal";
    r.phase = phase_;
    return r;
  }

 private:
  const char* candidate_ = "nominal";
  const char* confirmed_ = "nominal";
  const char* phase_ = "idle";
  double candidate_since_ = -1.0;
  double recover_since_ = -1.0;
};

}  // namespace railbreak

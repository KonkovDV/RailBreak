#pragma once
// ROS-free measurement timebase (RB08-04). The node must not invent dt from
// duplicate stamps, out-of-order stamps, or a leftover smaller than dt_min.

#include <cmath>

namespace tram_dr {

struct MeasDt {
  enum class Kind { kAnchor, kSkip, kReject, kApply };
  Kind kind{Kind::kSkip};
  double dt{0.0};
  bool advance_stamp{false};
};

// have_last / last_stamp_s: previous accepted measurement stamp.
// consumed_s: filter time already advanced by the timer path since that stamp.
// Duplicate or leftover < dt_min → skip (do not floor to dt_min).
// Regression beyond regress_tol → reject and keep the last good stamp.
inline MeasDt stamped_interval(bool have_last, double last_stamp_s, double stamp_s,
                               double consumed_s, double dt_min, double regress_tol) {
  MeasDt out;
  if (!std::isfinite(stamp_s)) {
    out.kind = MeasDt::Kind::kReject;
    out.advance_stamp = false;
    return out;
  }
  if (!have_last) {
    out.kind = MeasDt::Kind::kAnchor;
    out.advance_stamp = true;
    return out;
  }
  if (!std::isfinite(last_stamp_s) || !std::isfinite(consumed_s)) {
    out.kind = MeasDt::Kind::kReject;
    out.advance_stamp = false;
    return out;
  }
  const double raw = stamp_s - last_stamp_s;
  if (!std::isfinite(raw) || raw < -regress_tol) {
    out.kind = MeasDt::Kind::kReject;
    out.advance_stamp = false;
    return out;
  }
  const double dt = raw - consumed_s;
  if (!std::isfinite(dt) || dt < dt_min) {
    out.kind = MeasDt::Kind::kSkip;
    out.advance_stamp = (raw >= 0.0);
    return out;
  }
  out.kind = MeasDt::Kind::kApply;
  out.dt = dt;
  out.advance_stamp = true;
  return out;
}

}  // namespace tram_dr

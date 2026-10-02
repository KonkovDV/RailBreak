#pragma once

// Stamp of an output published between real inputs. The filter does not see it,
// and it must not move the node's output watermark: a later real input behind
// that watermark is treated as a stamp regression.

#include <algorithm>
#include <cmath>
#include <optional>

namespace railbreak {

// The jury twist at stamp t matches wheel speed from about t − 0.10 s.
// Publishing v(t − delay) at stamp t is the same pairing as publishing
// v(t − delay + offset) at stamp t + offset. Position keeps the input stamp:
// the reference pose does not share that lag. note_out() must stay on the
// input stamp, or the next sample looks late.
inline double velocity_output_stamp(double t_input, double stamp_offset_s) {
  if (!std::isfinite(t_input)) return t_input;
  if (!std::isfinite(stamp_offset_s)) return t_input;
  return t_input + stamp_offset_s;
}

inline double velocity_state_time(double t_input, double delay_s, double stamp_offset_s) {
  const double stamp = velocity_output_stamp(t_input, stamp_offset_s);
  const double delay = (std::isfinite(delay_s) && delay_s > 0.0) ? delay_s : 0.0;
  return stamp - delay;
}

// t_real: stamp of the last applied input, seconds.
// dt: time since that input was applied, seconds.
// t_pub_last: stamp of the last published output, or NaN if nothing was published.
// Returns nullopt when dt is shorter than min_dt, or the candidate is not
// strictly after t_pub_last. The stamp is at most cap seconds ahead of t_real.
inline std::optional<double> next_extrap_stamp(double t_real, double dt, double t_pub_last,
                                               double min_dt = 0.045, double cap = 0.1) {
  if (!std::isfinite(t_real) || !std::isfinite(dt) || !(dt >= min_dt) || !(cap > 0.0)) {
    return std::nullopt;
  }
  const double stamp = t_real + std::min(dt, cap);
  if (!std::isfinite(stamp)) return std::nullopt;
  if (std::isfinite(t_pub_last) && !(stamp > t_pub_last)) return std::nullopt;
  return stamp;
}

}  // namespace railbreak

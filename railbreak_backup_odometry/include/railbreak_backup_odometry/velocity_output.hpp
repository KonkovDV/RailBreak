#pragma once

// Published speed. The traction table explains about half of the acceleration,
// so a Kalman speed that trusts that table is smoother and later than the
// bogies. When the two bogies agree, the output is their raw mean. The filter
// remains the speed only while they disagree, are stale, or are missing.
// The blend is a crossfade, not a step.

#include <algorithm>
#include <cmath>
#include <string>

namespace railbreak {

inline bool velocity_source_is_wheels(const std::string& source) {
  return source != "filter";
}

// fade is 0 at the filter and 1 at the wheel mean. dt_s is the stamp gap
// since the previous output. fade_s is the crossfade length; 0 snaps.
inline double blend_velocity(double wheels_mps, double filter_mps, bool wheels_ok, double dt_s,
                             double fade_s, double& fade) {
  if (!std::isfinite(fade)) fade = 0.0;
  fade = std::clamp(fade, 0.0, 1.0);
  const double target = wheels_ok ? 1.0 : 0.0;
  if (!(fade_s > 0.0) || !std::isfinite(dt_s) || !(dt_s > 0.0)) {
    if (!(fade_s > 0.0)) fade = target;
  } else if (fade < target) {
    fade = std::min(target, fade + dt_s / fade_s);
  } else if (fade > target) {
    fade = std::max(target, fade - dt_s / fade_s);
  }
  const double wheels = std::isfinite(wheels_mps) ? wheels_mps : filter_mps;
  const double filter = std::isfinite(filter_mps) ? filter_mps : wheels;
  return fade * wheels + (1.0 - fade) * filter;
}

}  // namespace railbreak

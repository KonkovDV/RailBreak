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

// Samples on (t0, t1] at a fixed step. The value is the linear blend of the
// wheel means at the two pair stamps. A missing or non-positive step, or a
// backwards pair, publishes the current sample alone. The caller drops a gap
// wider than the stale gate before asking for a grid.
inline int velocity_resample(double t0, double v0, double t1, double v1, double step, double* ts,
                             double* vs, int cap) {
  if (!ts || !vs || cap <= 0 || !std::isfinite(t1) || !std::isfinite(v1)) return 0;
  const bool grid = std::isfinite(t0) && std::isfinite(v0) && std::isfinite(step) && step > 0.0 &&
                    t1 > t0;
  if (!grid) {
    ts[0] = t1;
    vs[0] = v1;
    return 1;
  }
  int n = 0;
  const double span = t1 - t0;
  for (double t = t0 + step; n < cap && t <= t1 + 1e-9; t += step) {
    const double a = std::min(1.0, std::max(0.0, (t - t0) / span));
    ts[n] = t > t1 ? t1 : t;
    vs[n] = v0 + a * (v1 - v0);
    if (n > 0 && !(ts[n] > ts[n - 1])) break;
    ++n;
    if (ts[n - 1] >= t1 - 1e-12) break;
  }
  if (n == 0 || ts[n - 1] < t1 - 1e-6) {
    if (n < cap) {
      ts[n] = t1;
      vs[n] = v1;
      ++n;
    }
  }
  return n;
}

}  // namespace railbreak

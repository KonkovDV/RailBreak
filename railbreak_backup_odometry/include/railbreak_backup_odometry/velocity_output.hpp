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

// Stamps strictly inside (t0, t1). The caller publishes t1. A non-positive
// step or a span longer than 1 s adds nothing: the hole stays visible to the
// 0.25 s acceptance gap. The arc at these stamps is the stored arc, not the
// arc at t1.
inline int position_stamp_grid(double t0, double t1, double step, double* ts, int cap) {
  if (!ts || cap <= 0 || !std::isfinite(t0) || !std::isfinite(t1) || !(t1 > t0)) return 0;
  if (!std::isfinite(step) || !(step > 0.0) || (t1 - t0) > 1.0) return 0;
  int n = 0;
  for (double t = t0 + step; n < cap && t < t1 - 1e-9; t += step) ts[n++] = t;
  return n;
}

inline double wrap_arc(double s, double ring_len) {
  if (!(ring_len > 0.0) || !std::isfinite(s)) return s;
  s = std::fmod(s, ring_len);
  if (s < 0.0) s += ring_len;
  return s;
}

// Arc published at tq, between two stored samples. net_lag_s is
// output_pos_lag_s − output_delay_pos_s. A jump above 8 m is a snap, not
// motion, and is not spread across the gap.
inline bool published_arc_between(double tq, double t0, double s0, double v0, double t1, double s1,
                                  double v1, double net_lag_s, double ring_len, double& s_out,
                                  double& v_out) {
  if (!std::isfinite(tq) || !std::isfinite(t0) || !std::isfinite(t1) || !(t1 > t0)) return false;
  if (tq < t0 - 1e-12 || tq > t1 + 1e-12) return false;
  const double a = std::min(1.0, std::max(0.0, (tq - t0) / (t1 - t0)));
  double ds = s1 - s0;
  if (ring_len > 0.0) {
    ds = std::fmod(ds + 0.5 * ring_len, ring_len);
    if (ds < 0.0) ds += ring_len;
    ds -= 0.5 * ring_len;
  }
  if (!std::isfinite(ds) || std::fabs(ds) > 8.0) return false;
  const double v = (std::isfinite(v0) && std::isfinite(v1)) ? v0 + a * (v1 - v0) : v1;
  double s = s0 + a * ds;
  if (std::isfinite(net_lag_s) && std::isfinite(v)) s -= v * net_lag_s;
  s_out = wrap_arc(s, ring_len);
  v_out = v;
  return std::isfinite(s_out) && std::isfinite(v_out);
}

}  // namespace railbreak

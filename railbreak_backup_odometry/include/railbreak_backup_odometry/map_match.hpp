// Continuity map match on one ring. The filter arc is not replaced.
// A second loop is not stored: "junction" means the nearest geometry
// lies outside the continuous step.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "railbreak_backup_odometry/track_odometer.hpp"

namespace railbreak {

struct MapMatch {
  const char* candidate_path = "ring";
  double candidate_s = 0.0;
  double along_track_error = 0.0;
  double cross_track_error = 0.0;
  double branch_probability = 1.0;
};

inline double arc_delta(const TrackMap& m, double a, double b) {
  double d = a - b;
  if (m.ring_len > 0.0) {
    d = std::fmod(d + 0.5 * m.ring_len, m.ring_len);
    if (d < 0.0) d += m.ring_len;
    d -= 0.5 * m.ring_len;
  }
  return d;
}

inline double match_weight(double cross_m, double along_m, double window_m) {
  const double sigma = 2.0;
  const double along_scale = std::max(window_m, 1.0);
  const double qc = cross_m / sigma;
  const double qa = along_m / along_scale;
  return std::exp(-0.5 * (qc * qc + qa * qa));
}

// s_now is the integrated arc. x, y is the point being matched, in map metres.
// The search stays inside a window ahead of s_prev. A stop does not move s.
inline MapMatch match_ring(const TrackMap& m, double s_prev, double s_now, double x, double y,
                           double v, double dt, const Stop* stops, int n_stops) {
  MapMatch out;
  out.candidate_s = m.wrap(s_prev);
  if (m.s.size() < 2 || m.x.size() != m.s.size() || m.y.size() != m.s.size()) {
    out.candidate_path = "none";
    out.branch_probability = 0.0;
    return out;
  }
  const double window = std::max(8.0, std::max(v, 0.0) * std::max(dt, 0.0) + 5.0);
  const double step = arc_delta(m, s_now, s_prev);
  if (step < -1.0 && v > 0.5) {
    out.candidate_path = "ring";
    out.candidate_s = m.wrap(s_prev);
    out.along_track_error = step;
    out.cross_track_error = 0.0;
    out.branch_probability = 0.25;
    return out;
  }

  struct Hit {
    bool ok = false;
    double dist = 1.0e300;
    double cross = 0.0;
    double along = 0.0;
    double s = 0.0;
    double w = 0.0;
  };
  Hit local;
  Hit global;
  const auto consider = [&](double x0, double y0, double x1, double y1, double s0, double s1) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    const double len = std::sqrt(std::max(len2, 0.0));
    double t = 0.0;
    if (len2 > 1.0e-12) t = std::clamp(((x - x0) * dx + (y - y0) * dy) / len2, 0.0, 1.0);
    const double fx = x0 + t * dx;
    const double fy = y0 + t * dy;
    const double cross = len > 1.0e-9 ? std::fabs((x - x0) * (-dy) + (y - y0) * dx) / len
                                      : std::hypot(x - x0, y - y0);
    const double s_foot = s0 + t * (s1 - s0);
    const double along = arc_delta(m, s_foot, s_prev);
    const double dist = std::hypot(x - fx, y - fy);
    const double w = match_weight(cross, along, window);
    if (!global.ok || dist < global.dist) global = Hit{true, dist, cross, along, s_foot, w};
    if (std::fabs(along) <= window && (!local.ok || dist < local.dist))
      local = Hit{true, dist, cross, along, s_foot, w};
  };

  for (std::size_t j = 1; j < m.s.size(); ++j)
    consider(m.x[j - 1], m.y[j - 1], m.x[j], m.y[j], m.s[j - 1], m.s[j]);
  // The closing chord is the lap seam only when the leftover arc is that same
  // short segment. A chord that cuts across a much longer arc is another path.
  const double seam_arc = m.ring_len - m.s.back() + m.s.front();
  const double chord = std::hypot(m.x.front() - m.x.back(), m.y.front() - m.y.back());
  if (seam_arc > 1.0e-6 && seam_arc < 8.0 && chord < 5.0 && seam_arc - chord < 1.5) {
    const double s1 = m.s.back() + seam_arc;
    consider(m.x.back(), m.y.back(), m.x.front(), m.y.front(), m.s.back(), s1);
  }

  if (!local.ok) {
    out.candidate_path = "ring";
    out.branch_probability = 0.0;
    return out;
  }
  out.candidate_s = m.wrap(local.s);
  out.cross_track_error = local.cross;
  out.along_track_error = arc_delta(m, s_now, out.candidate_s);
  const bool other = global.ok && std::fabs(arc_delta(m, global.s, local.s)) > 5.0 &&
                     global.cross + 1.0 < local.cross;
  if (other) {
    out.candidate_path = "junction";
    const double sum = local.w + global.w;
    out.branch_probability = sum > 0.0 ? local.w / sum : 0.0;
  } else {
    out.candidate_path = "ring";
    out.branch_probability = local.w;
  }
  int near_stops = 0;
  if (stops != nullptr) {
    for (int i = 0; i < n_stops; ++i) {
      if (stops[i].sd_m > 3.0) continue;
      if (std::fabs(arc_delta(m, stops[i].s_m, out.candidate_s)) <= 12.0) ++near_stops;
    }
  }
  // A stop does not replace candidate_s. Two stops in the gate make the branch ambiguous.
  if (near_stops >= 2) out.branch_probability = std::min(out.branch_probability, 0.5);
  out.branch_probability = std::clamp(out.branch_probability, 0.0, 1.0);
  return out;
}

}  // namespace railbreak

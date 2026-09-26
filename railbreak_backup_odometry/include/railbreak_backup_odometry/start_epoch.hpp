#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace railbreak {

// Upper median. The start fix has always selected this element, not the mean
// of the two central samples.
inline double upper_median(std::vector<double> v) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(m), v.end());
  return v[m];
}

// Odometer arc at a wheel or command stamp. Time is non-decreasing.
struct ArcMark {
  double t = 0.0;
  double s = 0.0;
};

// Arc at a GNSS stamp. Each span keeps the speed between the two marks.
inline double arc_at(const std::vector<ArcMark>& h, double t) {
  if (h.empty()) return 0.0;
  if (h.size() == 1) return h.front().s;
  if (t <= h.front().t) {
    const double dt = h[1].t - h[0].t;
    const double v = dt > 1e-9 ? (h[1].s - h[0].s) / dt : 0.0;
    return h.front().s + v * (t - h.front().t);
  }
  if (t >= h.back().t) {
    const ArcMark& a = h[h.size() - 2];
    const ArcMark& b = h.back();
    const double dt = b.t - a.t;
    const double v = dt > 1e-9 ? (b.s - a.s) / dt : 0.0;
    return b.s + v * (t - b.t);
  }
  const auto it = std::upper_bound(
      h.begin(), h.end(), t, [](double q, const ArcMark& m) { return q < m.t; });
  const ArcMark& b = *it;
  const ArcMark& a = *(it - 1);
  const double dt = b.t - a.t;
  const double u = dt > 1e-9 ? (t - a.t) / dt : 0.0;
  return a.s + u * (b.s - a.s);
}

// s0 is the ring arc of one sample. The carried path is the motion since that
// sample's stamp.
inline double align_s(double s0, double s_now, double s_epoch) {
  return s0 + (s_now - s_epoch);
}

}  // namespace railbreak

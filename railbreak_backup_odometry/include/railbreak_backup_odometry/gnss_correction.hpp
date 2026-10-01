// Rare mid-route GNSS. Only master RTK (status 2) corrects s, and only as one
// median per continuous window. Rover is the start-window heading, not a
// mid-route measurement: its lever arm was not calibrated on this route.
// A fix on a siding the map does not contain stays off-axis and is refused.
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace railbreak {

enum class GnssCorr : int {
  kApply = 0,
  kTooSoon,
  kOffAxis,
  kBaseline,
  kAlong,
  kNoPair,
};

struct GnssCorrLimits {
  double min_s = 30.0;
  double min_m = 150.0;
  double gate_m = 25.0;
  double cross_m = 3.0;
  double baseline_m = 12.436;
  double baseline_tol_m = 2.0;
};

inline GnssCorr gnss_correction_decision(double dt_s, double path_m, double cross_m, double along_m,
                                         double baseline_m, bool have_pair, const GnssCorrLimits& lim) {
  if (!have_pair) return GnssCorr::kNoPair;
  if (!(dt_s >= lim.min_s) || !(path_m >= lim.min_m)) return GnssCorr::kTooSoon;
  if (!(baseline_m > 0.0) || std::fabs(baseline_m - lim.baseline_m) > lim.baseline_tol_m)
    return GnssCorr::kBaseline;
  if (!(cross_m >= 0.0) || cross_m > lim.cross_m) return GnssCorr::kOffAxis;
  if (!(std::fabs(along_m) <= lim.gate_m)) return GnssCorr::kAlong;
  return GnssCorr::kApply;
}

// Same gates as a pair, without a second antenna and without a baseline.
inline GnssCorr gnss_single_decision(double dt_s, double path_m, double cross_m, double along_m,
                                     const GnssCorrLimits& lim) {
  if (!(dt_s >= lim.min_s) || !(path_m >= lim.min_m)) return GnssCorr::kTooSoon;
  if (!(cross_m >= 0.0) || cross_m > lim.cross_m) return GnssCorr::kOffAxis;
  if (!(std::fabs(along_m) <= lim.gate_m)) return GnssCorr::kAlong;
  return GnssCorr::kApply;
}

// Filter s is the master-antenna arc. Rover sits rover_baseline_m ahead of master.
inline double antenna_to_master_arc(double snap_s, bool master, double rover_baseline_m) {
  return master ? snap_s : snap_s - rover_baseline_m;
}

// Along gate grows with the filter's own s uncertainty, and never below gate_m.
inline double gnss_along_gate(double gate_m, double sigma_s) {
  return std::max(gate_m, 4.0 * std::max(0.0, sigma_s));
}

// One master burst: fixes closer than kGapS belong together. The correction is
// the median, applied once the next sample (or the wheel clock) is kGapS later.
struct GnssMasterBurst {
  struct Sample {
    double t = 0.0;
    double lat = 0.0;
    double lon = 0.0;
  };
  std::vector<Sample> samples;
  static constexpr double kGapS = 2.0;

  void clear() { samples.clear(); }
  bool empty() const { return samples.empty(); }
  double last_t() const { return samples.empty() ? 0.0 : samples.back().t; }

  bool opens_new(double t) const { return !samples.empty() && t > last_t() + kGapS; }

  void push(double t, double lat, double lon) { samples.push_back(Sample{t, lat, lon}); }

  bool due(double t_now) const { return !samples.empty() && t_now >= last_t() + kGapS; }

  bool median(double& t, double& lat, double& lon) const {
    if (samples.empty()) return false;
    std::vector<double> ts, lats, lons;
    ts.reserve(samples.size());
    lats.reserve(samples.size());
    lons.reserve(samples.size());
    for (const auto& s : samples) {
      ts.push_back(s.t);
      lats.push_back(s.lat);
      lons.push_back(s.lon);
    }
    const auto mid = [](std::vector<double>& v) {
      const std::size_t n = v.size() / 2;
      std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n), v.end());
      return v[n];
    };
    t = mid(ts);
    lat = mid(lats);
    lon = mid(lons);
    return true;
  }
};

}  // namespace railbreak

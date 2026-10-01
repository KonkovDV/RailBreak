// Rare mid-route GNSS. A pair is an s anchor only when it is on the ring.
// A fix on a siding the map does not contain stays off-axis and is refused.
#pragma once

#include <cmath>

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

}  // namespace railbreak

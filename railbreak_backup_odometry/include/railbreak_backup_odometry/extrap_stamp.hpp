#pragma once

// Stamp of an output published between real inputs. The filter does not see it,
// and it must not move the node's output watermark: a later real input behind
// that watermark is treated as a stamp regression.

#include <algorithm>
#include <cmath>
#include <optional>

namespace railbreak {

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

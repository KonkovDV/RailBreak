#pragma once

#include "railbreak_backup_odometry/integrity_monitor.hpp"

namespace railbreak {

// Written by tools/organizer/calibrate_integrity.py from the train split.
// Empirical along-track integrity bound. Not a certified protection level.
inline BoundCoeff empirical_bound() {
  BoundCoeff c;
  c.calibrated = true;
  c.q99 = 8.264474;
  c.b_nominal = 0.000000;
  c.b_single = 10.018627;
  c.b_model = 0.000000;
  c.b_common = 32.605727;
  c.b_no_map = 0.000000;
  c.b_relative = 0.000000;
  c.b_untrusted = 0.000000;
  c.b_time_m = 0.000000;
  c.b_map_m = 0.000000;
  c.coverage = "0.99";
  c.split = "train";
  return c;
}

}  // namespace railbreak

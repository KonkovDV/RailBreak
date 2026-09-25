// Core contracts on a synthetic straight ring. No ROS, no organiser data.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "railbreak_backup_odometry/track_odometer.hpp"

namespace {

int g_fail = 0;

void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) ++g_fail;
}

railbreak::Assets flat_ring(double length_m) {
  railbreak::Assets a;
  for (int i = 0; i <= static_cast<int>(length_m); ++i) {
    a.map.s.push_back(i);
    a.map.x.push_back(i);
    a.map.y.push_back(0.0);
    a.map.h.push_back(0.0);
    a.map.grade.push_back(0.0);
  }
  a.map.ring_len = length_m + 1.0;
  a.map.lat0 = 55.8;
  a.map.lon0 = 37.4;
  for (int j = 0; j < 18; ++j) a.table.v_centre.push_back(j + 0.5);
  for (auto& row : a.table.a) row.assign(18, 0.0);  // coasting holds speed
  a.stops.push_back({450.0, 0.5, 10});
  a.k0 = 1.0;
  return a;
}

// Drive both bogies at v_kmh from t0 to t1 at 10 Hz each, offset 50 ms.
void drive(railbreak::TrackOdometer& od, double t0, double t1, double v_kmh,
           double front_scale = 1.0, bool front_on = true, bool rear_on = true) {
  for (double t = t0; t < t1 - 1e-9; t += 0.1) {
    if (front_on) od.on_bogie(t, true, v_kmh * front_scale);
    if (rear_on) od.on_bogie(t + 0.05, false, v_kmh);
    od.on_cmd(t + 0.025, 0);
  }
}

}  // namespace

int main() {
  const auto assets = flat_ring(2000.0);
  railbreak::Params p;
  p.unit = 1.0 / 3.6;

  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 60.0, 36.0);  // 10 m/s for 60 s
    check(std::fabs(od.v() - 10.0) < 0.05, "constant speed is tracked");
    std::printf("     s=%.2f v=%.3f k=%.5f\n", od.s(), od.v(), od.k());
    check(std::fabs(od.s() - 600.0) < 2.0, "path integrates speed");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 10.0, 36.0);
    od.on_bogie(10.0, true, std::nan(""));
    od.on_bogie(9.0, false, 36.0);  // stamp regressed: no time passes
    check(std::isfinite(od.s()) && std::isfinite(od.v()), "NaN and a regressed stamp keep the state finite");
    check(od.n_rejected() >= 1, "NaN is counted as rejected");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    const double s_before = od.s();
    drive(od, 30.0, 40.0, 36.0, 1.0, false, false);  // both silent 10 s, notch only
    const double gained = od.s() - s_before;
    std::printf("     gained=%.2f v=%.3f\n", gained, od.v());
    check(std::fabs(gained - 100.0) < 5.0, "model carries the path through a 10 s double dropout");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    bool flagged = false;
    for (double t = 30.0; t < 45.0; t += 0.1) {
      od.on_bogie(t, true, 36.0 * 1.2);  // front spins +20 %
      flagged = flagged || od.slip();
      od.on_bogie(t + 0.05, false, 36.0);
    }
    check(flagged, "a spinning bogie is flagged");
    check(std::fabs(od.v() - 10.0) < 0.3, "speed follows the healthy bogie");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    // Wheels read 0.5 % low (the spread seen in the recordings); 40 s at 10 m/s,
    // then a 1 m/s^2 brake to the station at s = 450.
    drive(od, 0.0, 40.0, 36.0 * 0.995);
    for (double t = 40.0; t < 60.0; t += 0.1) {
      const double v_kmh = std::max(0.0, 36.0 * 0.995 * (1.0 - (t - 40.0) / 10.0));
      od.on_bogie(t, true, v_kmh);
      od.on_bogie(t + 0.05, false, v_kmh);
      od.on_cmd(t + 0.025, t < 50.0 ? -5 : -8);
    }
    std::printf("     s=%.2f anchors=%d sigma_s=%.2f\n", od.s(), od.n_anchor(), od.sigma_s());
    check(od.n_anchor() == 1, "one anchor per dwell");
    check(std::fabs(od.v()) < 1e-9, "ZUPT holds zero speed");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 10.0, 36.0);
    od.on_bogie(100.0, true, 36.0);  // 90 s gap: time base reset, no integration
    check(od.n_gap_reset() == 1 && od.s() < 200.0, "a long gap resets the time base");
  }
  {
    // Reference values: pyproj EPSG:4326 -> EPSG:32637 at the two route termini.
    double e = 0, n = 0;
    railbreak::utm_forward(55.810417, 37.462308, 37, e, n);
    const bool a = std::hypot(e - 403632.8321, n - 6186049.6084) < 1e-3;
    railbreak::utm_forward(55.799532, 37.388974, 37, e, n);
    const bool b = std::hypot(e - 399009.2478, n - 6184942.8281) < 1e-3;
    check(a && b, "UTM 37N matches pyproj within 1 mm");
    railbreak::mkrs_forward(55.810417, 37.462308, e, n);
    check(std::hypot(e - (-2363.207), n - 16005.384) < 1e-3,
          "Moscow grid on WGS84 matches pyproj within 1 mm");
    railbreak::OutputFrame f;
    f.mode = railbreak::FrameMode::kEnu;
    f.set_start(55.81, 37.46, 150.0);
    double x, y, z;
    f.to_out(55.81, 37.46 + 0.01, 150.0, x, y, z);
    // 0.01 deg east at 55.81 N: 627.00 m on WGS84 (626.99 via pyproj ECEF);
    // the sphere of radius a gives 624.7 m.
    check(std::fabs(x - 627.0) < 0.01 && std::fabs(y - 0.045) < 0.005,
          "ENU east uses the WGS84 prime-vertical radius");
  }
  std::printf("%s\n", g_fail ? "FAILED" : "all passed");
  return g_fail ? 1 : 0;
}

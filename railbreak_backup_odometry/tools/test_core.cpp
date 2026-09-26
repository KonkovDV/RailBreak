// Core contracts on a synthetic straight ring. No ROS, no organiser data.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

#include "railbreak_backup_odometry/adhesion_proxy.hpp"
#include "railbreak_backup_odometry/gnss_window.hpp"
#include "railbreak_backup_odometry/integrity_bound.hpp"
#include "railbreak_backup_odometry/integrity_monitor.hpp"
#include "railbreak_backup_odometry/start_epoch.hpp"
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
    // b_a has no hard bound. A short agreed spike barely trains it. A sustained
    // regime does, and silence keeps that value. Wheels matching the table again
    // pull it back, but not to zero in one minute.
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    double t = 30.0;
    for (int i = 0; i < 5; ++i) {
      od.on_bogie(t, true, 36.0 + 36.0);
      od.on_bogie(t + 0.05, false, 36.0 + 36.0);
      t += 0.1;
    }
    const double ba_spike = od.model_bias();
    for (int i = 0; i < 50; ++i) {
      od.on_cmd(t, 0);
      t += 0.1;
    }
    std::printf("     spike ba=%.4f v=%.3f\n", ba_spike, od.v());
    check(std::fabs(ba_spike) < 0.01, "a 0.5 s agreed spike barely moves b_a");
    check(std::fabs(od.model_bias() - ba_spike) < 1e-12, "silence does not change b_a");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    double t = 30.0;
    for (int i = 0; i < 80; ++i) {
      const double v_kmh = 36.0 + 0.4 * (i * 0.1) * 3.6;
      od.on_bogie(t, true, v_kmh);
      od.on_bogie(t + 0.05, false, v_kmh);
      od.on_cmd(t + 0.025, 0);
      t += 0.1;
    }
    const double ba = od.model_bias();
    const double v1 = od.v();
    std::printf("     trained ba=%.4f v=%.3f\n", ba, v1);
    check(ba > 0.04, "eight seconds of shared acceleration the table lacks moves b_a");
    for (int i = 0; i < 50; ++i) {
      od.on_cmd(t, 0);
      t += 0.1;
    }
    const double ba_silent = od.model_bias();
    std::printf("     silent ba=%.4f v=%.3f dv=%.3f\n", ba_silent, od.v(), od.v() - v1);
    check(std::fabs(ba_silent - ba) < 1e-12, "b_a stays at the trained value while both bogies are silent");
    check(std::fabs((od.v() - v1) - ba * 5.0) < 0.05, "the silent interval integrates the contaminated b_a");
    const double v_hold = od.v();
    for (int i = 0; i < 600; ++i) {
      const double kmh = v_hold * 3.6;
      od.on_bogie(t, true, kmh);
      od.on_bogie(t + 0.05, false, kmh);
      od.on_cmd(t + 0.025, 0);
      t += 0.1;
    }
    std::printf("     returned ba=%.4f\n", od.model_bias());
    check(od.model_bias() < ba_silent * 0.6, "b_a moves back once the wheels match the table");
    check(od.model_bias() > 0.005, "one minute does not clear b_a");
  }
  {
    auto accel = flat_ring(2000.0);
    for (double& a : accel.table.a[static_cast<std::size_t>(7 - railbreak::kNotchMin)]) a = 0.8;
    railbreak::TrackOdometer od(&accel, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    od.on_cmd(30.0, 7);
    const double s0 = od.s();
    const double v0 = od.v();
    const double a_new = 0.8;  // steady row, the value the fit would keep after 1 s
    for (double t = 30.1; t <= 35.0 + 1e-9; t += 0.1) od.on_cmd(t, 7);
    const double T = 5.0;
    const double lag = 1.0;  // the second excluded from the table, not a state in the filter
    const double dv = od.v() - v0;
    const double ds = od.s() - s0;
    const double dv_lag = a_new * (T - lag);
    const double ds_lag = v0 * T + 0.5 * a_new * (T - lag) * (T - lag);
    std::printf("     notch ba=%.4f dv=%.3f ds=%.2f excess_v=%.3f excess_s=%.2f\n",
                od.model_bias(), dv, ds, dv - dv_lag, ds - ds_lag);
    check(std::fabs(od.model_bias()) < 0.01, "a notch change with no wheels does not train b_a");
    check(std::fabs(dv - a_new * T) < 0.05, "five silent seconds use the new notch acceleration at once");
    check(std::fabs(ds - (v0 * T + 0.5 * a_new * T * T)) < 0.5, "the path is that steady acceleration");
    check(std::fabs((dv - dv_lag) - a_new * lag) < 0.05, "a one-second actuator lag leaves one second of acceleration as speed error");
    check(std::fabs((ds - ds_lag) - (0.5 * a_new * lag * lag + a_new * lag * (T - lag))) < 0.5,
          "the speed error stays and the path error grows through the rest of the silence");
  }
  {
    auto hostile = flat_ring(4000.0);
    hostile.stops.clear();
    hostile.stops.push_back({20.0, 0.2, 10});
    hostile.k0 = 1.0;
    railbreak::Params hp;
    hp.unit = 1.0 / 3.6;
    hp.sigma_k0 = 50.0;
    hp.q_k = 1.0;
    hp.stop_gate = 20.0;
    railbreak::TrackOdometer od(&hostile, hp);
    od.init(0.0, 50.0);
    drive(od, 0.0, 60.0, 36.0);
    for (double t = 60.0; t < 64.0 - 1e-9; t += 0.1) {
      od.on_bogie(t, true, 0.0);
      od.on_bogie(t + 0.05, false, 0.0);
    }
    std::printf("     guarded k=%.3f guards=%d\n", od.k(), od.n_guard());
    check(std::isfinite(od.k()) && od.k() >= 0.5 && od.k() <= 1.5,
          "a damaged anchor cannot drive k through zero");
    check(od.n_guard() >= 1, "the update that would leave the scale interval is dropped");
    check(std::isfinite(od.s()) && std::isfinite(od.v()) && std::isfinite(od.model_bias()),
          "the state stays finite");
    od.on_bogie(64.0, true, 36.0);
    od.on_bogie(64.05, false, 36.0);
    check(std::isfinite(od.v()) && od.k() > 0.0, "the next wheel step still divides by k");
  }
  {
    auto bad = flat_ring(100.0);
    bad.k0 = 0.0;
    railbreak::TrackOdometer od(&bad, p);
    check(od.k() == 1.0, "a zero scale prior is replaced before any division");
    check(od.n_guard() >= 1, "the replacement is counted");
    od.init(0.0, 0.5);
    od.set_time(0.0);
    od.on_bogie(0.0, true, 36.0);
    check(std::isfinite(od.v()), "the first wheel step stays finite");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 10.0, 36.0);
    check(od.n_guard() == 0, "a normal run does not trip the numerical stop");
  }
  {
    double c[36];
    railbreak::TrackOdometer::fill_pose_covariance(4.0, c);
    check(c[0] == 4.0 && c[7] == 4.0 && c[14] == 4.0, "Pss is on x, y and z");
    check(c[1] == 0.0 && c[2] == 0.0 && c[6] == 0.0 && c[8] == 0.0 && c[12] == 0.0 && c[13] == 0.0,
          "position off-diagonals stay zero");
    check(c[7] != railbreak::TrackOdometer::kCrossTrackSigmaM * railbreak::TrackOdometer::kCrossTrackSigmaM,
          "the cross-track floor is not the y variance");
    check(c[14] != railbreak::TrackOdometer::kMapHeightSigmaM * railbreak::TrackOdometer::kMapHeightSigmaM,
          "the height floor is not the z variance");
    check(c[21] == 1e6 && c[28] == 1e6 && c[35] == 1e6, "orientation stays uninformative");
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
    drive(od, 0.0, 30.0, 36.0);
    od.on_bogie(30.0, true, 36.0 * 1.2);
    od.on_bogie(30.1, true, 36.0 * 1.2);
    od.on_bogie(30.15, false, 36.0);
    std::printf("     slip_last=%d front=%d rear=%d run=%d age=%.3f nis=%.1f\n",
                od.slip() ? 1 : 0, od.slip_front() ? 1 : 0, od.slip_rear() ? 1 : 0,
                od.slip_front_run(), od.slip_age_s(), od.slip_front_nis());
    check(!od.slip(), "the published slip flag is the last callback");
    check(od.slip_front() && !od.slip_rear(), "a healthy rear does not clear the front channel");
    check(od.slip_front_run() >= 2, "consecutive front anomalies are counted");
    check(od.slip_age_s() > 0.1, "the front channel keeps its age across the rear callback");
    check(od.slip_front_nis() > 16.0, "the channel keeps the gate statistic, not a probability");
    od.on_bogie(30.2, true, 36.0);
    check(!od.slip_front() && od.slip_front_run() == 0, "a healthy front callback clears only the front channel");
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
    // The anchor is ZUPT plus a unique station inside the gate. No stop id,
    // heading, schedule or previous-stop sequence. A wrong unique station locks.
    auto line = flat_ring(2000.0);
    line.stops.clear();
    line.stops.push_back({100.0, 0.5, 10});
    railbreak::TrackOdometer wrong(&line, p);
    wrong.init(98.0, 5.0);
    wrong.set_time(0.0);
    for (double t = 0.0; t < 2.0 - 1e-9; t += 0.1) {
      wrong.on_bogie(t, true, 0.0);
      wrong.on_bogie(t + 0.05, false, 0.0);
    }
    std::printf("     wrong-station s=%.2f anchors=%d\n", wrong.s(), wrong.n_anchor());
    check(wrong.n_anchor() == 1, "one unique station in the gate is taken");
    check(wrong.s() > 99.5, "that station pulls s off the place the filter was standing");

    line.stops.push_back({104.0, 0.5, 10});
    railbreak::TrackOdometer two(&line, p);
    two.init(100.0, 5.0);
    two.set_time(0.0);
    for (double t = 0.0; t < 2.0 - 1e-9; t += 0.1) {
      two.on_bogie(t, true, 0.0);
      two.on_bogie(t + 0.05, false, 0.0);
    }
    check(two.n_anchor() == 0, "two stations in the gate are not a guess");
    check(std::fabs(two.s() - 100.0) < 0.05, "an ambiguous dwell does not move s");

    line.stops.clear();
    line.stops.push_back({200.0, 0.5, 10});
    railbreak::TrackOdometer far(&line, p);
    far.init(100.0, 1.0);
    far.set_time(0.0);
    for (double t = 0.0; t < 2.0 - 1e-9; t += 0.1) {
      far.on_bogie(t, true, 0.0);
      far.on_bogie(t + 0.05, false, 0.0);
    }
    check(far.n_anchor() == 0, "a station outside the gate is not used");

    line.stops.clear();
    line.stops.push_back({100.0, 12.0, 10});
    railbreak::TrackOdometer loose(&line, p);
    loose.init(100.0, 1.0);
    loose.set_time(0.0);
    for (double t = 0.0; t < 2.0 - 1e-9; t += 0.1) {
      loose.on_bogie(t, true, 0.0);
      loose.on_bogie(t + 0.05, false, 0.0);
    }
    check(loose.n_anchor() == 0, "a stop wider than 3 m is not a station anchor");
  }
  {
    // One ring, no switch graph and no direction after start. Metres of a
    // depot spur, a turnout, a short turn or a partial trip still move s
    // along this ring. Negative wheels are not a second direction.
    railbreak::TrackOdometer od(&assets, p);
    od.init(500.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 10.0, 36.0);
    std::printf("     ring-only s=%.2f v=%.3f\n", od.s(), od.v());
    check(std::fabs(od.v() - 10.0) < 0.05, "speed on the only axis matches the wheels");
    check(std::fabs(od.s() - 600.0) < 2.0, "those metres move s along the ring");
    const double s_fwd = od.s();
    drive(od, 10.0, 15.0, -36.0);
    std::printf("     reverse s=%.2f v=%.3f ds=%.2f\n", od.s(), od.v(), od.s() - s_fwd);
    check(od.v() < 0.05, "agreed reverse is not kept as a negative speed");
    check(od.s() + 0.05 >= s_fwd, "reverse does not walk s backward");
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
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 5.0, 36.0);
    const double s = od.s();
    const int notch = od.notch();
    const int rej = od.n_rejected();
    od.on_bogie(1.0, false, 36.0);
    od.on_cmd(1.0, 8);
    check(std::fabs(od.s() - s) < 1e-9, "a stamp behind the filter does not move the state");
    check(od.notch() == notch, "a stamp behind the filter does not change the notch");
    check(od.n_rejected() == rej + 2, "regressed stamps are counted and dropped");
    const int mid = od.n_rejected();
    const double t_now = 5.0;
    od.on_bogie(t_now, true, 36.0);
    od.on_bogie(t_now, false, 36.0);
    od.on_cmd(t_now, notch);
    check(od.n_rejected() == mid, "an equal stamp of the other bogie is applied");
    od.on_bogie(0.0, true, 36.0);
    od.on_bogie(std::numeric_limits<double>::quiet_NaN(), false, 36.0);
    od.on_cmd(std::numeric_limits<double>::infinity(), 4);
    check(od.n_rejected() == mid + 3, "a zero, NaN or infinite stamp is dropped");
    check(od.notch() == notch, "a non-finite command does not change the notch");
    // rear 10.10 then front 10.00, and a command walked backwards: no pair, no notch.
    od.on_bogie(10.10, false, 36.0);
    const double s2 = od.s();
    const double rho = od.rear_front_ratio();
    const int rej2 = od.n_rejected();
    od.on_bogie(10.00, true, 50.0);
    check(std::fabs(od.s() - s2) < 1e-9, "front at 10.00 after rear at 10.10 does not move s");
    check(std::fabs(od.rear_front_ratio() - rho) < 1e-12, "a regressed bogie does not learn rho");
    check(od.n_rejected() == rej2 + 1, "front at 10.00 is counted");
    od.on_cmd(10.2, 4);
    od.on_cmd(9.9, 8);
    check(od.notch() == 4, "a command at 9.9 does not replace the notch");
    check(od.n_rejected() == rej2 + 2, "the command at 9.9 is counted");
    for (double skew : {0.05, 0.10, 0.30}) {
      od.on_bogie(20.0, true, 36.0);
      const double sb = od.s();
      const double rb = od.rear_front_ratio();
      const int nb = od.notch();
      const int kb = od.n_rejected();
      od.on_bogie(20.0 - skew, false, 50.0);
      od.on_cmd(20.0 - skew, 7);
      check(std::fabs(od.s() - sb) < 1e-9 && std::fabs(od.rear_front_ratio() - rb) < 1e-12 &&
                od.notch() == nb && od.n_rejected() == kb + 2,
            "a stamp 50/100/300 ms behind is not a pair and not a notch");
    }
    // 100 ms lag still applies when the later stamp is processed second.
    const int before = od.n_rejected();
    od.on_bogie(30.0, true, 36.0);
    od.on_bogie(30.1, false, 36.0);
    check(od.n_rejected() == before, "a bogie 100 ms later is not a regression");
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
  {
    using Act = railbreak::GnssWindow::Action;
    const int q = railbreak::GnssWindow::kQuietInputs;
    auto silent_closes = [q](railbreak::GnssWindow& w, double t0) {
      Act last = Act::kWait;
      for (int i = 0; i < q + 2; ++i) {
        last = w.on_input(t0 + 0.05 * i);
        if (last == Act::kFinish) return i == q;
      }
      return false;
    };
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      w.on_fix(true, 2.9, true);
      w.on_fix(false, 2.9, true);
      check(w.on_input(3.05) == Act::kWait, "one late wheel does not close the GNSS window");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      w.on_fix(true, 2.9, true);
      w.on_fix(false, 2.9, true);
      check(silent_closes(w, 3.05), "both antennas silent inside the window still initialise");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(true, 2.5, true);
      check(silent_closes(w, 3.10), "master only inside the window initialises");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(false, 0.0, true);
      w.on_fix(false, 2.5, true);
      check(silent_closes(w, 3.10), "rover only inside the window initialises");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      check(w.on_fix(true, 3.2, true) == Act::kWait, "one antenna past the window waits for the other");
      check(w.on_fix(false, 3.2, true) == Act::kFinish, "fixes past the window close from GNSS, not from a wheel");
    }
    {
      railbreak::GnssWindow w;
      for (int i = 0; i < 6; ++i) {
        w.on_fix(true, 0.1 * i, true);
        w.on_fix(false, 0.1 * i, true);
      }
      check(w.on_input(3.2) == Act::kWait, "a wheel ahead of the GNSS queue does not close");
      bool early = false;
      for (int i = 0; i < 20; ++i) {
        w.on_fix(true, 2.0, true);
        w.on_fix(false, 2.0, true);
        if (w.on_input(3.3 + 0.05 * i) == Act::kFinish) early = true;
      }
      check(!early, "queued fixes inside the window keep the window open");
      Act last = Act::kWait;
      int at = -1;
      for (int i = 0; i < q + 1; ++i) {
        last = w.on_input(6.0 + 0.05 * i);
        if (last == Act::kFinish) {
          at = i;
          break;
        }
      }
      check(at == q - 1, "the window closes once the GNSS queue has gone quiet");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      check(w.on_fix(true, 3.2, true) == Act::kWait, "master past, rover silent: not closed yet");
      Act last = Act::kWait;
      for (int i = 0; i < q + 2; ++i) {
        w.on_fix(true, 3.3 + 0.05 * i, true);  // master keeps publishing
        last = w.on_input(3.4 + 0.05 * i);
        if (last == Act::kFinish) break;
      }
      check(last == Act::kFinish, "a live antenna does not hold the silent one open");
    }
    {
      railbreak::GnssWindow w;
      check(w.on_input(0.0) == Act::kWait, "no fix yet: the wait has not expired");
      check(w.on_input(10.01) == Act::kRelative, "no fix within gnss_wait_s goes relative");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      w.on_fix(true, 2.9, true);
      w.on_fix(false, 2.9, true);
      Act last = Act::kWait;
      int at = -1;
      for (int i = 0; i < q + 4; ++i) {
        w.on_fix(true, 3.2, false);
        w.on_fix(false, 3.2, false);
        last = w.on_input(3.05 + 0.05 * i);
        if (last == Act::kFinish) {
          at = i;
          break;
        }
      }
      check(at == q, "a stream of NO_FIX does not hold the window open");
    }
    {
      railbreak::GnssWindow w;
      check(w.on_fix(true, 0.0, true) == Act::kWait, "a valid master opens the start");
      check(w.on_fix(false, 0.0, true) == Act::kWait, "a valid rover stays in that start");
      check(w.on_fix(true, 3.2, true) == Act::kWait, "master past the window waits for rover");
      check(w.on_fix(false, 3.2, true) == Act::kFinish, "both antennas past the window finish the start");
      w.closed = true;
      const int m = w.master_fixes();
      const int r = w.rover_fixes();
      const double t_open = w.t_open;
      bool ignored = true;
      for (int i = 0; i < 30; ++i) {
        ignored = ignored && w.on_fix(i % 2 == 0, 4.0 + 0.1 * i, false) == Act::kWait;
        ignored = ignored && w.on_input(4.0 + 0.1 * i) == Act::kWait;
      }
      ignored = ignored && w.on_fix(true, std::numeric_limits<double>::quiet_NaN(), false) == Act::kWait;
      check(ignored && w.closed && w.master_fixes() == m && w.rover_fixes() == r && w.t_open == t_open,
            "continuous invalid GNSS after a valid start does not reopen the window");
    }
    {
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      Act last = Act::kWait;
      for (int i = 0; i < q + 2 && last != Act::kFinish; ++i) last = w.on_input(3.2 + 0.05 * i);
      check(last == Act::kFinish, "quiet count asks to close");
      const int m = w.master_fixes();
      check(w.on_fix(true, 2.0, true) == Act::kWait, "a fix taken before unsubscribe is still accepted");
      check(w.master_fixes() == m + 1, "that fix is counted");
      check(w.on_input(4.0) == Act::kWait, "a counted fix resets the quiet drain");
    }
    {
      railbreak::GnssWindow w;
      check(w.on_fix(false, 0.0, true) == Act::kWait, "the first rover opens the window");
      check(w.t_open == 0.0, "the origin is that rover stamp");
      check(w.on_fix(true, 2.5, true) == Act::kWait, "a later master stays inside the original window");
      check(w.t_open == 0.0, "a later master does not move the origin");
      check(4.0 > w.t_open + w.window_s, "a rover at 4 s is outside the frozen window");
      check(w.on_fix(false, 4.0, true) == Act::kWait, "that rover is past the window and waits for master");
      check(w.on_fix(true, 3.1, true) == Act::kFinish, "master past the original end closes the window");
      check(w.t_open == 0.0, "the origin is still the first rover");
    }
    {
      // Eight wheel callbacks are not proof a GNSS fix is absent. The first
      // input past the window only resets the drain; the finish is later.
      railbreak::GnssWindow w;
      w.on_fix(true, 0.0, true);
      w.on_fix(false, 0.0, true);
      int finish_at = -1;
      for (int i = 0; i < 12; ++i) {
        if (w.on_input(3.2 + 0.05 * i) == Act::kFinish) {
          finish_at = i;
          break;
        }
      }
      check(finish_at == q, "the quiet finish is the call after the drain reset, not the eighth wheel");
      railbreak::GnssWindow early;
      early.on_fix(true, 0.0, true);
      early.on_fix(false, 0.0, true);
      bool closed = false;
      for (int i = 0; i < 7; ++i)
        closed = closed || early.on_input(3.2 + 0.05 * i) == Act::kFinish;
      check(!closed, "seven wheels before the next GNSS fix do not close the window");
      check(early.on_fix(true, 2.0, true) == Act::kWait, "a queued valid fix is still accepted");
      check(early.on_input(4.0) == Act::kWait, "that fix resets the drain, so the next wheel does not finish");
    }
    {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const double inf = std::numeric_limits<double>::infinity();
      const int fix = 0;
      check(railbreak::gnss_fix_ok(1.0, 55.8, 37.4, 160.0, fix, fix),
            "a finite fix inside the ranges is valid");
      check(railbreak::gnss_fix_ok(1.0, -90.0, -180.0, 0.0, fix, fix),
            "the closed latitude and longitude bounds are valid");
      check(railbreak::gnss_fix_ok(1.0, 90.0, 180.0, 0.0, fix, fix),
            "the other closed bounds are valid");
      check(!railbreak::gnss_fix_ok(nan, 55.8, 37.4, 160.0, fix, fix), "a NaN stamp is not a fix");
      check(!railbreak::gnss_fix_ok(inf, 55.8, 37.4, 160.0, fix, fix), "an infinite stamp is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, 37.4, nan, fix, fix), "a NaN altitude is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, 37.4, inf, fix, fix), "an infinite altitude is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, nan, 37.4, 160.0, fix, fix), "a NaN latitude is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, nan, 160.0, fix, fix), "a NaN longitude is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 90.1, 37.4, 160.0, fix, fix), "latitude above 90 is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, -90.1, 37.4, 160.0, fix, fix), "latitude below -90 is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, 180.1, 160.0, fix, fix), "longitude above 180 is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, -180.1, 160.0, fix, fix), "longitude below -180 is not a fix");
      check(!railbreak::gnss_fix_ok(1.0, 55.8, 37.4, 160.0, -1, fix), "STATUS_NO_FIX stays invalid");
      railbreak::GnssWindow poisoned;
      check(poisoned.on_fix(false, nan, false) == Act::kWait, "a rejected stamp does not finish the window");
      check(poisoned.t_open < 0.0, "a NaN stamp does not open the window");
      check(poisoned.on_fix(false, 0.0, railbreak::gnss_fix_ok(0.0, 55.8, 37.4, 160.0, fix, fix)) == Act::kWait,
            "the next finite fix is still accepted");
      check(poisoned.t_open == 0.0, "that finite fix is the origin");
      std::vector<double> alt;
      if (railbreak::gnss_fix_ok(0.0, 55.8, 37.4, nan, fix, fix)) alt.push_back(nan);
      if (railbreak::gnss_fix_ok(0.1, 55.8, 37.4, 160.0, fix, fix)) alt.push_back(160.0);
      if (railbreak::gnss_fix_ok(0.2, 55.8, 37.4, 162.0, fix, fix)) alt.push_back(162.0);
      check(alt.size() == 2 && std::isfinite(railbreak::upper_median(alt)),
            "a NaN altitude does not enter the alignment median");
    }
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.on_cmd(0.0, 7);
    check(od.notch() == 7, "a command before any bogie sets the notch");
    od.on_bogie(0.1, true, 36.0);
    od.on_bogie(0.15, false, 36.0);
    check(std::isfinite(od.s()) && std::fabs(od.v() - 10.0) < 0.2,
          "wheels after that command stay finite");
    railbreak::TrackOdometer rates(&assets, p);
    rates.init(0.0, 0.5);
    rates.set_time(0.0);
    for (int i = 0; i < 50; ++i) {
      rates.on_bogie(0.1 * i, true, 36.0);
      if (i % 5 == 0) rates.on_bogie(0.1 * i + 0.02, false, 36.0);
    }
    check(std::isfinite(rates.v()) && rates.v() > 5.0, "front at 10 Hz and rear at 2 Hz stay finite");
    railbreak::TrackOdometer moving(&assets, p);
    moving.on_bogie(0.0, true, 36.0);
    check(std::fabs(moving.v() - 10.0) < 0.2, "the first reading accepts a start already at speed");
  }
  {
    auto badk = assets;
    badk.k0 = 1e-6;
    railbreak::TrackOdometer od(&badk, p);
    check(std::fabs(od.k() - 1.0) < 1e-12 && od.n_guard() >= 1,
          "k0 below 0.5 is replaced by 1 and counted");
    railbreak::Params neg = p;
    neg.sigma_k0 = -0.2;
    railbreak::Params pos = p;
    pos.sigma_k0 = 0.2;
    railbreak::TrackOdometer a(&assets, neg);
    railbreak::TrackOdometer b(&assets, pos);
    a.init(0.0, 1.0);
    b.init(0.0, 1.0);
    a.set_time(0.0);
    b.set_time(0.0);
    drive(a, 0.0, 3.0, 36.0);
    drive(b, 0.0, 3.0, 36.0);
    check(std::fabs(a.s() - b.s()) < 1e-6 && std::fabs(a.k() - b.k()) < 1e-9,
          "a negative sigma_k0 is squared and matches the positive value");
    railbreak::Params nq = p;
    nq.q_s = -100.0;
    nq.q_v = -100.0;
    nq.q_k = -1.0;
    nq.q_ba = -1.0;
    railbreak::TrackOdometer guard(&assets, nq);
    guard.init(10.0, 1.0);
    guard.set_time(0.0);
    guard.on_bogie(1.0, true, 36.0);
    check(guard.n_guard() >= 1 && std::isfinite(guard.s()) && std::isfinite(guard.v()),
          "negative process noise trips the PSD guard and stays finite");
    check(std::fabs(guard.s() - 10.0) < 1e-6, "that rejected step does not move s");
    auto hole = assets;
    hole.map.grade[20] = std::numeric_limits<double>::quiet_NaN();
    railbreak::TrackOdometer nanmap(&hole, p);
    nanmap.init(0.0, 0.5);
    nanmap.set_time(0.0);
    drive(nanmap, 0.0, 5.0, 36.0);
    check(std::isfinite(nanmap.s()) && std::isfinite(nanmap.v()), "NaN in the grade column stays finite");
    auto bent = assets;
    bent.map.s[3] = 1.0;
    railbreak::TrackOdometer ring(&bent, p);
    ring.init(0.0, 0.5);
    ring.set_time(0.0);
    drive(ring, 0.0, 2.0, 36.0);
    check(std::isfinite(ring.s()) && ring.s() > 10.0, "a non-increasing ring sample does not stop the filter");
    auto blank = assets;
    blank.table.a[15].clear();
    check(blank.table.lookup(0, 5.0) == 0.0, "an empty notch row looks up as zero acceleration");
    railbreak::TrackOdometer coast(&blank, p);
    coast.init(0.0, 0.5);
    coast.set_time(0.0);
    drive(coast, 0.0, 2.0, 36.0);
    check(std::fabs(coast.v() - 10.0) < 0.2, "that empty row does not throw and does not change coasting");
    railbreak::Params flip = p;
    flip.unit = -1.0 / 3.6;
    railbreak::TrackOdometer negu(&assets, flip);
    negu.init(0.0, 0.5);
    negu.set_time(0.0);
    drive(negu, 0.0, 2.0, 36.0);
    std::printf("     neg unit s=%.3f v=%.3f\n", negu.s(), negu.v());
    check(negu.v() < 0.05 && negu.s() > 1900.0,
          "a negative wheel_unit_scale keeps speed at 0 and wraps a tiny backward step");
    railbreak::TrackOdometer far(&assets, p);
    far.init(1e8, 1.0);
    far.set_time(0.0);
    drive(far, 0.0, 1.0, 36.0);
    std::printf("     huge s=%.3f v=%.3f\n", far.s(), far.v());
    check(std::isfinite(far.s()) && far.s() > 30.0 && far.s() < 40.0 && std::fabs(far.v() - 10.0) < 0.2,
          "a huge initial_s_m is wrapped onto the ring and speed is still tracked");
  }
  {
    railbreak::TrackOdometer both(&assets, p);
    both.init(0.0, 0.5);
    both.set_time(0.0);
    for (double t = 0.0; t < 10.0 - 1e-9; t += 0.1) {
      both.on_bogie(t, true, 36.0 * 1.05);
      both.on_bogie(t + 0.05, false, 36.0 * 1.05);
    }
    std::printf("     both +5%% s=%.2f v=%.3f\n", both.s(), both.v());
    check(both.v() > 10.2 && both.s() > 100.0, "agreed +5% on both bogies is accepted as speed");
    railbreak::TrackOdometer slide(&assets, p);
    slide.init(0.0, 0.5);
    slide.set_time(0.0);
    for (double t = 0.0; t < 8.0 - 1e-9; t += 0.1) {
      slide.on_bogie(t, true, 36.0 * 0.8);
      slide.on_bogie(t + 0.05, false, 36.0 * 0.8);
    }
    std::printf("     both -20%% s=%.2f v=%.3f\n", slide.s(), slide.v());
    check(slide.v() < 9.0 && slide.v() > 7.0, "agreed -20% on both bogies is accepted as speed");
    railbreak::TrackOdometer frozen(&assets, p);
    frozen.init(0.0, 0.5);
    frozen.set_time(0.0);
    drive(frozen, 0.0, 2.0, 36.0);
    for (double t = 2.0; t < 5.0 - 1e-9; t += 0.1) {
      frozen.on_bogie(t, true, 0.0);
      frozen.on_bogie(t + 0.05, false, 36.0);
    }
    check(std::isfinite(frozen.s()) && frozen.v() >= 0.0, "one bogie frozen at the old speed stays finite");
    auto brake = assets;
    brake.table.a[8].assign(18, -1.0);
    railbreak::TrackOdometer held(&brake, p);
    held.init(0.0, 0.5);
    held.set_time(0.0);
    drive(held, 0.0, 2.0, 36.0);
    const double s_hold = held.s();
    for (double t = 2.0; t < 6.0 - 1e-9; t += 0.1) {
      held.on_bogie(t, true, 36.0);
      held.on_bogie(t + 0.05, false, 36.0);
      held.on_cmd(t + 0.02, -7);
    }
    check(std::isfinite(held.s()) && held.s() + 0.05 >= s_hold && held.v() >= 0.0,
          "both bogies frozen at speed while the notch brakes do not walk backward");
    railbreak::TrackOdometer sgn(&assets, p);
    sgn.init(0.0, 0.5);
    sgn.set_time(0.0);
    for (double t = 0.0; t < 3.0 - 1e-9; t += 0.1) {
      sgn.on_bogie(t, true, 36.0);
      sgn.on_bogie(t + 0.05, false, -36.0);
    }
    check(std::isfinite(sgn.s()) && sgn.v() >= 0.0, "a sign flip on one bogie does not make speed negative");
    railbreak::TrackOdometer lag(&assets, p);
    lag.init(0.0, 0.5);
    lag.set_time(0.0);
    for (double t = 0.0; t < 3.0 - 1e-9; t += 0.1) {
      lag.on_bogie(t, true, 36.0);
      lag.on_bogie(t + 0.2, false, 36.0);
    }
    check(std::fabs(lag.v() - 10.0) < 0.3, "a 200 ms lag on one bogie is still applied");
    railbreak::TrackOdometer drift(&assets, p);
    drift.init(0.0, 0.5);
    drift.set_time(0.0);
    for (double t = 0.0; t < 10.0 - 1e-9; t += 0.1) {
      const double scale = 1.0 + 0.002 * t;
      drift.on_bogie(t, true, 36.0 * scale);
      drift.on_bogie(t + 0.05, false, 36.0 * scale);
    }
    check(std::isfinite(drift.s()) && drift.s() > 100.0, "a slow scale drift on both bogies stays finite");
    auto kick = assets;
    kick.table.a[22].assign(18, 0.5);
    railbreak::TrackOdometer drop(&kick, p);
    drop.init(0.0, 0.5);
    drop.set_time(0.0);
    drive(drop, 0.0, 2.0, 36.0);
    const double s_drop = drop.s();
    for (double t = 2.0; t < 6.0 - 1e-9; t += 0.2) drop.on_cmd(t, 7);
    check(drop.s() > s_drop + 30.0, "silence right after a notch change integrates the new acceleration");
    auto line = flat_ring(2000.0);
    line.stops.clear();
    line.stops.push_back({100.0, 0.5, 10});
    railbreak::TrackOdometer miss(&line, p);
    miss.init(0.0, 0.5);
    miss.set_time(0.0);
    drive(miss, 0.0, 20.0, 36.0);
    check(miss.n_anchor() == 0, "driving past the only station does not anchor");
  }
  {
    railbreak::IntegrityObs o;
    o.t = 10.0;
    o.sigma_s = 2.0;
    o.front_age_s = 0.05;
    o.rear_age_s = 0.05;
    o.pair_fresh = true;
    o.bogies_agree = true;
    o.absolute_start = true;
    o.map_in_domain = true;
    railbreak::IntegrityMonitor mon;
    const auto nominal = mon.update(o);
    check(std::string(nominal.status) == "NOMINAL", "fresh agreeing bogies are nominal");
    check(nominal.reasons.empty(), "nominal has no reason");
    check(nominal.use_position, "nominal may be used");
    check(!nominal.certification_claim, "the bound is not a certificate");
    o.nis_front = 20.0;
    o.slip_front = true;
    const auto one = railbreak::IntegrityMonitor{}.update(o);
    check(std::string(one.status) == "DEGRADED_SINGLE_BOGIE", "one high NIS is a single bogie");
    check(one.reasons.find("FRONT_NIS_HIGH") != std::string::npos, "front NIS is named");
    o.front_age_s = 31.0;
    o.rear_age_s = 31.0;
    o.pair_fresh = false;
    o.bogies_agree = false;
    o.slip_front = false;
    o.nis_front = 0.0;
    const auto gap = railbreak::IntegrityMonitor{}.update(o);
    check(std::string(gap.status) == "POSITION_UNTRUSTED", "a gap past the clock reset is refused");
    check(!gap.use_position, "an untrusted position is not for use");
    railbreak::BoundCoeff c;
    c.calibrated = true;
    c.q99 = 2.5;
    c.b_single = 4.0;
    railbreak::IntegrityObs fresh = o;
    fresh.front_age_s = 0.05;
    fresh.rear_age_s = 0.05;
    fresh.pair_fresh = true;
    fresh.bogies_agree = true;
    fresh.slip_front = true;
    fresh.nis_front = 20.0;
    fresh.sigma_s = 2.0;
    const auto bound = railbreak::IntegrityMonitor{c}.update(fresh);
    check(std::fabs(bound.along_bound_m - 9.0) < 1e-9, "bound is q * sigma + B_mode");
    check(std::string(railbreak::empirical_bound().coverage) == "0.99", "coverage target stays 0.99");
    check(!railbreak::empirical_bound().calibrated || railbreak::empirical_bound().q99 > 0.0,
          "a fitted multiplier is positive");
    auto line = flat_ring(500.0);
    railbreak::TrackOdometer od(&line, railbreak::Params{});
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 2.0, 36.0);
    const double s_before = od.s();
    const int anchors = od.n_anchor();
    railbreak::IntegrityObs live;
    live.t = od.time_s();
    live.sigma_s = od.sigma_s();
    live.absolute_start = true;
    live.map_in_domain = true;
    live.front_age_s = 0.05;
    live.rear_age_s = 0.05;
    live.pair_fresh = od.pair_fresh();
    live.bogies_agree = od.bogies_agree();
    live.n_anchor = od.n_anchor();
    railbreak::IntegrityMonitor shadow;
    shadow.update(live);
    check(od.s() == s_before && od.n_anchor() == anchors, "the monitor does not move the filter");
  }
  {
    railbreak::AdhesionObs o;
    o.t = 10.0;
    o.pair_fresh = true;
    o.bogies_agree = true;
    o.slip_front = true;
    o.slip_rear = true;
    o.have_consensus = true;
    o.wheel_consensus_residual = 0.05;
    o.have_model = true;
    o.model_consistency_residual = -2.4;
    o.front_nis = 40.0;
    o.rear_nis = 36.0;
    o.notch = 4;
    o.speed = 8.5;
    railbreak::AdhesionProxy proxy;
    const auto first = proxy.update(o);
    o.t = 11.7;
    const auto second = proxy.update(o);
    check(std::string(second.classification) == "COMMON_MODE_SUSPECTED",
          "agreeing bogies that both leave the model are a suspected common mode");
    check(std::string(second.mu_estimate) == "null", "mu is not estimated");
    check(!second.mu_observable, "mu is not observable from these inputs");
    check(std::string(second.reason) == "both bogies agree but differ from model",
          "the reason names the residual pattern");
    check(std::fabs(first.duration_s) < 1e-12, "the episode starts at zero duration");
    check(std::fabs(second.duration_s - 1.7) < 1e-12, "duration is the length of this episode");
    check(second.json().find("\"mu_estimate\":null") != std::string::npos, "the proxy json keeps mu null");
    o.bogies_agree = false;
    o.t = 12.0;
    const auto split = proxy.update(o);
    check(std::string(split.classification) == "BOGIES_DISAGREE", "a bogie split is not an adhesion estimate");
    check(std::fabs(split.common_mode_duration_s) < 1e-12, "a split clears the common-mode clock");
    auto line = flat_ring(500.0);
    railbreak::TrackOdometer od(&line, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 2.0, 36.0);
    const double v_before = od.v();
    od.on_bogie(2.0, true, 36.0 * 0.7);
    od.on_bogie(2.05, false, 36.0 * 0.7);
    const double s_after = od.s();
    const double v_after = od.v();
    check(od.wheel_consensus_have() && od.wheel_consensus_residual() < 0.5,
          "matched bogies keep a small consensus residual");
    check(od.model_consistency_have() && od.model_consistency_residual() < -1.0,
          "wheels below the model leave a negative consistency residual");
    check(od.slip_front() && od.slip_rear(), "both low bogies are flagged");
    check(od.v() != v_before, "the bogie callbacks themselves still update speed");
    railbreak::AdhesionObs live;
    live.t = od.time_s();
    live.pair_fresh = od.pair_fresh();
    live.bogies_agree = od.bogies_agree();
    live.slip_front = od.slip_front();
    live.slip_rear = od.slip_rear();
    live.have_consensus = od.wheel_consensus_have();
    live.wheel_consensus_residual = od.wheel_consensus_residual();
    live.have_model = od.model_consistency_have();
    live.model_consistency_residual = od.model_consistency_residual();
    live.front_nis = od.slip_front_nis();
    live.rear_nis = od.slip_rear_nis();
    live.notch = od.notch();
    live.speed = od.v();
    const auto live_report = railbreak::AdhesionProxy{}.update(live);
    check(std::string(live_report.classification) == "COMMON_MODE_SUSPECTED",
          "the live pair is the common-mode pattern");
    check(std::string(live_report.mu_estimate) == "null", "the live pair still has no mu");
    check(od.s() == s_after && od.v() == v_after, "the proxy does not move the filter");
  }
  {
    // Rover is valid at t=0 and the tram is already moving. Master is valid
    // only from t=2.5. The window, opened by the rover, ends at t=3.
    // base_link is the master antenna plus 9.873 m along the ring.
    const double s_m0 = 400.0;
    const double v = 10.0;
    const double off_along = 9.873;
    const double t_end = 3.0;
    const auto line = flat_ring(2000.0);
    railbreak::TrackOdometer od(&line, p);
    std::vector<railbreak::ArcMark> hist;
    for (double t = 0.0; t < t_end + 1e-9; t += 0.1) {
      od.on_bogie(t, true, 36.0);
      hist.push_back({t, od.s()});
      od.on_cmd(t + 0.025, 0);
      hist.push_back({t + 0.025, od.s()});
      od.on_bogie(t + 0.05, false, 36.0);
      hist.push_back({t + 0.05, od.s()});
    }
    const double s_now = railbreak::arc_at(hist, t_end);
    auto master_at = [&](double t, double& lat, double& lon) {
      const double s = s_m0 + v * t;
      line.map.latlon(line.map.at(line.map.x, s), line.map.at(line.map.y, s), lat, lon);
    };
    auto snap = [&](double lat, double lon) {
      return railbreak::init_on_ring(line.map, lat, lon, false, 0.0, 0.0);
    };
    auto base_link = [&](double s0, double t_epoch) {
      return railbreak::align_s(s0, s_now, railbreak::arc_at(hist, t_epoch)) + off_along;
    };
    const double physical = s_m0 + v * t_end + off_along;

    double lat = 0.0, lon = 0.0;
    master_at(2.5, lat, lon);
    const auto one = snap(lat, lon);
    check(one.ok && std::fabs(one.s0 - (s_m0 + v * 2.5)) < 1e-6,
          "the single master fix snaps to the antenna arc at t=2.5");
    const double at_master = base_link(one.s0, 2.5);
    const double at_rover = base_link(one.s0, 0.0);
    std::printf("     one master: auth err=%.3f rover-epoch err=%.3f physical=%.3f\n",
                at_master - physical, at_rover - physical, physical);
    check(std::fabs(at_master - physical) < 0.3,
          "the alignment epoch is the first master fix, and base_link matches");
    check(at_rover - physical > 20.0,
          "anchoring that master fix at the rover stamp is ahead by the travelled section");

    std::vector<double> ts, lats, lons;
    for (double t = 2.5; t < t_end + 1e-9; t += 0.1) {
      master_at(t, lat, lon);
      ts.push_back(t);
      lats.push_back(lat);
      lons.push_back(lon);
    }
    const double t_sample = railbreak::upper_median(ts);
    const auto med = snap(railbreak::upper_median(lats), railbreak::upper_median(lons));
    const double at_sample = base_link(med.s0, t_sample);
    const double at_first = base_link(med.s0, 2.5);
    const double sample_at_rover = base_link(med.s0, 0.0);
    std::printf("     master interval: sample t=%.2f sample err=%.3f first-master err=%.3f rover err=%.3f\n",
                t_sample, at_sample - physical, at_first - physical, sample_at_rover - physical);
    check(std::fabs(t_sample - 2.5) > 0.2, "the master sample is later than the first master fix");
    check(std::fabs(at_sample - physical) < 0.3,
          "base_link matches when the path is anchored at the master sample stamp");
    check(at_first - physical > 2.0,
          "anchoring the later master sample at the first master stamp is not base_link");
    check(sample_at_rover - physical > 20.0,
          "anchoring the later master sample at the rover stamp is ahead by the travelled section");
  }
  std::printf("%s\n", g_fail ? "FAILED" : "all passed");
  return g_fail ? 1 : 0;
}

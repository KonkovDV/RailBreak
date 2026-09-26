// Core contracts on a synthetic straight ring. No ROS, no organiser data.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "railbreak_backup_odometry/gnss_window.hpp"
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
    const double plat = railbreak::TrackOdometer::kCrossTrackSigmaM *
                        railbreak::TrackOdometer::kCrossTrackSigmaM;
    const double pz = railbreak::TrackOdometer::kMapHeightSigmaM *
                      railbreak::TrackOdometer::kMapHeightSigmaM;
    railbreak::TrackOdometer::fill_pose_covariance(4.0, 1.0, 0.0, 0.0, c);
    check(std::fabs(c[0] - 4.0) < 1e-9, "eastbound along-track variance is Pss");
    check(std::fabs(c[7] - plat) < 1e-9, "eastbound cross-track variance is the map floor");
    check(std::fabs(c[14] - pz) < 1e-9, "level-track height variance is the map floor, not Pss");
    check(std::fabs(c[1]) < 1e-12, "an axis-aligned track has no Pxy");
    railbreak::TrackOdometer::fill_pose_covariance(4.0, 1.0, 1.0, 0.04, c);
    const double half = 0.5 * (4.0 + plat);
    check(std::fabs(c[0] - half) < 1e-9 && std::fabs(c[7] - half) < 1e-9,
          "a diagonal track splits Pss and the lateral floor");
    check(std::fabs(c[1] - 0.5 * (4.0 - plat)) < 1e-9, "a diagonal track publishes Pxy");
    check(std::fabs(c[14] - (4.0 * 0.04 * 0.04 + pz)) < 1e-9,
          "height is the grade term plus the map floor");
    check(std::fabs(c[2] - 4.0 * std::sqrt(0.5) * 0.04) < 1e-9,
          "along-track error couples into height");
    check(c[21] == 1e6 && c[35] == 1e6, "orientation stays uninformative");
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
  }
  std::printf("%s\n", g_fail ? "FAILED" : "all passed");
  return g_fail ? 1 : 0;
}

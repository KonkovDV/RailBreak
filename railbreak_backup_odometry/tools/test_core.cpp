// Core contracts on a synthetic straight ring. No ROS, no organiser data.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "railbreak_backup_odometry/adhesion_proxy.hpp"
#include "railbreak_backup_odometry/gnss_window.hpp"
#include "railbreak_backup_odometry/input_reorder.hpp"
#include "railbreak_backup_odometry/integrity_bound.hpp"
#include "railbreak_backup_odometry/integrity_monitor.hpp"
#include "railbreak_backup_odometry/interval.hpp"
#include "railbreak_backup_odometry/map_match.hpp"
#include "railbreak_backup_odometry/slip_hypothesis.hpp"
#include "railbreak_backup_odometry/start_epoch.hpp"
#include "railbreak_backup_odometry/track_odometer.hpp"

namespace {

int g_fail = 0;

railbreak::IntegrityReport soak(railbreak::IntegrityMonitor& mon, railbreak::IntegrityObs o, double t1) {
  railbreak::IntegrityReport last{};
  const double t0 = o.t;
  const int n = static_cast<int>(std::floor((t1 - t0) / 0.1 + 1e-9));
  for (int i = 0; i <= n; ++i) {
    o.t = t0 + 0.1 * i;
    last = mon.update(o);
  }
  return last;
}

void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) ++g_fail;
}

template <class F>
bool rejected(F&& fn, const char* fragment) {
  try {
    std::forward<F>(fn)();
  } catch (const std::invalid_argument& e) {
    return std::string(e.what()).find(fragment) != std::string::npos;
  } catch (...) {
    return false;
  }
  return false;
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
    check(rejected([&] { railbreak::TrackOdometer od(&bad, p); }, "initial k"),
          "a zero scale prior is rejected before any division");
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
    check(od.slip_front_age_s() > 0.1 && od.slip_rear_age_s() == 0.0,
          "the front channel keeps its own age across the rear callback");
    check(std::fabs(od.slip_age_s() - od.slip_front_age_s()) < 1e-9,
          "one slipping channel is the whole any-slip age");
    check(od.slip_front_nis() > 16.0, "the channel keeps the gate statistic, not a probability");
    od.on_bogie(30.2, true, 36.0);
    check(!od.slip_front() && od.slip_front_run() == 0 && od.slip_front_age_s() == 0.0,
          "a healthy front callback clears only the front channel");
  }
  {
    railbreak::TrackOdometer od(&assets, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    drive(od, 0.0, 30.0, 36.0);
    od.on_bogie(30.0, true, 36.0 * 1.2);
    od.on_bogie(30.4, false, 36.0 * 1.2);
    od.on_bogie(30.5, true, 36.0);
    std::printf("     handoff any=%.3f front=%.3f rear=%.3f\n",
                od.slip_age_s(), od.slip_front_age_s(), od.slip_rear_age_s());
    check(!od.slip_front() && od.slip_rear(), "the rear is the only channel still slipping");
    check(od.slip_front_age_s() == 0.0, "the recovered front does not keep an age");
    check(od.slip_rear_age_s() > 0.05 && od.slip_rear_age_s() < 0.2,
          "the rear age starts when the rear enters, not at the earlier front");
    check(od.slip_age_s() > 0.4, "slip_age_s stays the continuous any-slip age");
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
    check(od.v() > 1.0 && od.common_unobservable(),
          "agreed reverse is not copied into the speed");
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
    {
      // Master queue is ahead of the rover and then delivers an earlier stamp.
      // Rover's only queued stamp is already past the window. Index pairing
      // closes on that rover before the early master is read.
      using Pair = std::pair<bool, double>;
      const std::vector<Pair> by_index = {{true, 3.2}, {false, 3.3}, {true, 2.4}};
      auto opened = []() {
        railbreak::GnssWindow w;
        w.on_fix(true, 0.0, true);
        w.on_fix(false, 0.0, true);
        return w;
      };
      auto apply = [](railbreak::GnssWindow w, const std::vector<Pair>& seq) {
        bool saw_early = false;
        bool closed_on = false;
        for (const auto& s : seq) {
          const auto action = w.on_fix(s.first, s.second, true);
          if (s.second == 2.4) saw_early = true;
          if (action == Act::kFinish) {
            closed_on = true;
            break;
          }
        }
        return std::pair<bool, bool>{saw_early, closed_on};
      };
      const auto dropped = apply(opened(), by_index);
      check(!dropped.first && dropped.second,
            "index interleaving closes before the earlier master in the other queue");
      std::vector<railbreak::QueuedStamp> taken = {
          {true, 3.2, 0},
          {true, 2.4, 1},
          {false, 3.3, 2},
      };
      const auto ordered = railbreak::order_queued_stamps(taken);
      check(ordered.size() == 3 && ordered[0].index == 1 && ordered[1].index == 0 &&
                ordered[2].index == 2,
            "queued fixes are ordered by stamp, not by subscription index");
      std::vector<Pair> by_stamp;
      for (const auto& s : ordered) by_stamp.push_back({s.master, s.t});
      const auto kept = apply(opened(), by_stamp);
      check(kept.first && kept.second,
            "the earlier master is applied before the window closes on the later rover");
      std::vector<railbreak::QueuedStamp> ties = {
          {true, 1.0, 0},
          {true, 1.0, 1},
          {false, 1.0, 2},
      };
      const auto stable = railbreak::order_queued_stamps(ties);
      check(stable.size() == 3 && stable[0].index == 0 && stable[1].index == 1 && stable[2].index == 2,
            "equal stamps keep take order");
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const auto nonfinite = railbreak::order_queued_stamps({{true, nan, 0}, {false, 1.0, 1}});
      check(nonfinite.size() == 2 && nonfinite[0].index == 1 && nonfinite[1].index == 0,
            "a non-finite stamp sorts after a finite one");
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
    check(rejected([&] { railbreak::TrackOdometer od(&badk, p); }, "initial k"),
          "k0 below 0.5 is rejected before the first step");
    badk.k0 = 0.5;
    check(rejected([&] { railbreak::TrackOdometer od(&badk, p); }, "initial k"),
          "k0 at 0.5 is outside the open initial interval");
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
    check(rejected([&] { railbreak::TrackOdometer guard(&assets, nq); }, "q_s"),
          "negative process noise is rejected before the first step");
    railbreak::Params zr = p;
    zr.r0 = 0.0;
    check(rejected([&] { railbreak::TrackOdometer guard(&assets, zr); }, "r0"),
          "a zero measurement variance is rejected before the first step");
    auto hole = assets;
    hole.map.grade[20] = std::numeric_limits<double>::quiet_NaN();
    check(rejected([&] { railbreak::TrackOdometer nanmap(&hole, p); }, "not finite"),
          "NaN in the grade column is rejected before the first step");
    railbreak::Params rq = p;
    rq.r0 = 0.0;
    rq.r_min = 0.0;
    check(rejected([&] { railbreak::TrackOdometer rec(&assets, rq); }, "r0"),
          "a zero measurement variance is rejected before a singular reacquisition");
    auto bent = assets;
    bent.map.s[3] = 1.0;
    check(rejected([&] { railbreak::TrackOdometer ring(&bent, p); }, "strictly increasing"),
          "a non-increasing ring sample is rejected before the first step");
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
    check(rejected([&] { railbreak::TrackOdometer negu(&assets, flip); }, "wheel_unit_scale"),
          "a negative wheel_unit_scale is rejected before it can wrap the ring");
    railbreak::TrackOdometer far(&assets, p);
    check(rejected([&] { far.init(1e8, 1.0); }, "initial s"),
          "a huge initial_s_m is rejected instead of being wrapped onto the ring");
    const double nan = std::numeric_limits<double>::quiet_NaN();
    check(rejected([&] {
            railbreak::validate_geometry(0.0, 10.0, 9.873, -3.0, 12.436, nan, 1000.0, true);
          }, "gnss_init_window_s"),
          "a GNSS window of 0 is rejected");
    check(rejected([&] {
            railbreak::validate_geometry(3.0, -1.0, 9.873, -3.0, 12.436, nan, 1000.0, true);
          }, "gnss_wait_s"),
          "a negative GNSS wait is rejected");
    check(rejected([&] {
            railbreak::validate_geometry(3.0, 10.0, nan, -3.0, 12.436, nan, 1000.0, true);
          }, "output offsets"),
          "a NaN geometric offset is rejected");
    check(rejected([&] {
            railbreak::validate_geometry(3.0, 10.0, 9.873, -3.0, 0.0, nan, 1000.0, true);
          }, "rover_baseline_m"),
          "a zero rover baseline is rejected");
    check(rejected([&] {
            railbreak::validate_geometry(3.0, 10.0, 9.873, -3.0, 12.436, 1e8, 1000.0, true);
          }, "initial_s_m"),
          "initial_s_m far outside the ring is rejected");
    bool geometry_ok = true;
    try {
      railbreak::validate_geometry(3.0, 10.0, 9.873, -3.0, 12.436, nan, 1000.0, true);
    } catch (...) {
      geometry_ok = false;
    }
    check(geometry_ok, "the default window, wait and offsets are accepted");
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
    railbreak::TrackOdometer rear_scale(&assets, p);
    rear_scale.init(0.0, 0.5);
    rear_scale.set_time(0.0);
    for (double t = 0.0; t < 15.0 - 1e-9; t += 0.1) {
      rear_scale.on_bogie(t, true, 36.0);
      rear_scale.on_bogie(t + 0.05, false, 36.0 * 1.05);
      rear_scale.on_cmd(t + 0.025, 0);
    }
    check(std::fabs(rear_scale.k_rear()) < 1e-9 && std::fabs(rear_scale.k_front()) < 1e-9,
          "a persistent rear scale does not train either bogie scale");
    check(std::fabs(rear_scale.v() - 10.0) < 0.25,
          "speed stays with the healthy bogie, not the mean of the pair");
    railbreak::TrackOdometer calm(&assets, p);
    calm.init(0.0, 0.5);
    calm.set_time(0.0);
    for (double t = 0.0; t < 8.0 - 1e-9; t += 0.1) {
      calm.on_bogie(t, true, 36.0);
      calm.on_bogie(t + 0.05, false, 36.0 * 1.002);
      calm.on_cmd(t + 0.025, 0);
    }
    check(calm.k_rear() > 0.0005 && std::fabs(calm.k_front()) < calm.k_rear(),
          "a small agreed rear offset adapts only the rear scale");
    auto closure = flat_ring(4000.0);
    closure.stops = {{91.0, 0.5, 10}};
    railbreak::TrackOdometer along(&closure, p);
    along.init(0.0, 0.5);
    along.set_time(0.0);
    for (double t = 0.0; t < 8.0 - 1e-9; t += 0.1) {
      along.on_bogie(t, true, 36.0);
      along.on_bogie(t + 0.05, false, 36.0);
      along.on_cmd(t + 0.025, 0);
    }
    const double k_before = along.k();
    for (double t = 8.0; t < 10.0 - 1e-9; t += 0.1) {
      along.on_bogie(t, true, 0.0);
      along.on_bogie(t + 0.05, false, 0.0);
    }
    check(along.n_anchor() >= 1 && along.k() > k_before,
          "a station past a long run corrects the common scale");
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
    check(std::string(nominal.confidence_velocity) == "ok", "fresh bogies trust velocity");
    check(std::string(nominal.confidence_position) == "ok", "fresh bogies trust position");
    check(std::string(nominal.integrity_mode) == "NOMINAL", "fresh bogies are the nominal trust mode");
    check(std::string(nominal.velocity_confidence) == "HIGH", "fresh velocity trust is high");
    check(nominal.reasons.empty(), "nominal has no reason");
    check(nominal.use_position, "nominal may be used");
    check(!nominal.certification_claim, "the bound is not a certificate");
    o.nis_front = 20.0;
    o.slip_front = true;
    railbreak::IntegrityMonitor spike;
    const auto one = spike.update(o);
    check(std::string(one.status) == "NOMINAL" && one.fault_score < 0.2,
          "one high NIS does not enter degraded");
    check(one.reasons.find("FRONT_NIS_HIGH") != std::string::npos, "front NIS is named");
    const auto held_one = soak(spike, o, o.t + 1.0);
    check(std::string(held_one.status) == "DEGRADED_SINGLE_BOGIE",
          "a fault score above 0.8 for 0.5 s is a single bogie");
    check(std::string(held_one.integrity_mode) == "WHEEL_DEGRADED",
          "a held fault degrades that wheel");
    check(std::string(held_one.velocity_confidence) == "HIGH" &&
              std::string(held_one.front_wheel_confidence) == "LOW",
          "the other wheel still supports the speed");
    o.front_age_s = 31.0;
    o.rear_age_s = 31.0;
    o.pair_fresh = false;
    o.bogies_agree = false;
    o.slip_front = false;
    o.nis_front = 0.0;
    const auto gap = railbreak::IntegrityMonitor{}.update(o);
    check(std::string(gap.status) == "POSITION_UNTRUSTED", "a gap past the clock reset is refused");
    check(!gap.use_position, "an untrusted position is not for use");
    check(std::string(gap.integrity_mode) == "LOST",
          "a gap past the trust window is lost trust, not a dead node");
    check(std::string(gap.velocity_confidence) == "NONE" && std::string(gap.position_confidence) == "NONE",
          "lost trust clears both channels");
    railbreak::IntegrityObs carry;
    carry.t = 10.0;
    carry.front_age_s = 1.0;
    carry.rear_age_s = 1.0;
    carry.absolute_start = true;
    carry.map_in_domain = true;
    const auto assisted = railbreak::IntegrityMonitor{}.update(carry);
    check(std::string(assisted.integrity_mode) == "MODEL_ASSISTED" &&
              std::string(assisted.model_confidence) == "LOW" &&
              std::string(assisted.velocity_confidence) == "LOW",
          "a short gap in both wheels is model assist, not a lost estimate");
    {
      railbreak::IntegrityObs blind;
      blind.t = 0.0;
      blind.sigma_s = 1.0;
      blind.front_age_s = 0.05;
      blind.rear_age_s = 0.05;
      blind.pair_fresh = true;
      blind.bogies_agree = true;
      blind.absolute_start = true;
      blind.map_in_domain = true;
      blind.slip_front = true;
      blind.slip_rear = true;
      blind.nis_front = 20.0;
      blind.nis_rear = 20.0;
      blind.distance_since_anchor = 10.0;
      railbreak::IntegrityMonitor budget;
      const auto inside = soak(budget, blind, 1.0);
      check(std::string(inside.status) == "DEGRADED_COMMON_MODE_UNOBSERVABLE" &&
                std::string(inside.integrity_mode) == "VELOCITY_DEGRADED" &&
                inside.use_position && inside.time_to_lost > 0.0 &&
                inside.blind_warning[0] == '\0',
            "both bogies off the model stay inside the anchor budget at first");
      blind.t = 1.1;
      blind.distance_since_anchor = 150.0;
      const auto far = budget.update(blind);
      check(std::string(far.status) == "LOST" && !far.use_position && far.time_to_lost == 0.0 &&
                std::fabs(far.distance_since_last_trusted_anchor - 150.0) < 1e-9 &&
                std::string(far.velocity_confidence) == "LOW" &&
                std::string(far.position_confidence) == "NONE" &&
                std::string(far.fault_level) == "lost" &&
                far.reasons.find("BLIND_BUDGET") != std::string::npos,
            "a high common-mode score past the blind distance is LOST");
      blind.n_anchor = 1;
      blind.distance_since_anchor = 0.0;
      blind.t = 1.2;
      const auto reset = budget.update(blind);
      check(reset.use_position && std::string(reset.integrity_mode) != "LOST",
            "an accepted anchor ends the blind refusal");
    }
    {
      railbreak::IntegrityObs held;
      held.t = 10.0;
      held.sigma_s = 1.0;
      held.front_age_s = 0.05;
      held.rear_age_s = 0.05;
      held.pair_fresh = true;
      held.bogies_agree = true;
      held.absolute_start = true;
      held.map_in_domain = true;
      held.slip_front = true;
      held.slip_rear = true;
      held.common_unobservable = true;
      held.distance_since_anchor = 10.0;
      railbreak::IntegrityMonitor open;
      const auto started = open.update(held);
      check(std::string(started.status) == "DEGRADED_COMMON_MODE_UNOBSERVABLE" &&
                std::string(started.integrity_mode) == "VELOCITY_DEGRADED" &&
                std::string(started.velocity_confidence) == "LOW" &&
                std::string(started.position_confidence) == "LOW" &&
                started.use_position && started.time_to_lost == 5.0 &&
                started.blind_time_s == 0.0,
            "common mode starts a blind clock and does not trust the wheels");
      held.t = 16.0;
      const auto lost = open.update(held);
      check(std::string(lost.status) == "LOST" &&
                std::string(lost.integrity_mode) == "LOST" &&
                std::string(lost.velocity_confidence) == "LOW" &&
                std::string(lost.position_confidence) == "NONE" &&
                !lost.use_position && lost.time_to_lost == 0.0 &&
                !lost.bound_valid,
            "without a new anchor the blind budget ends in LOST");
      held.n_anchor = 1;
      held.common_unobservable = false;
      held.slip_front = false;
      held.slip_rear = false;
      held.t = 16.1;
      held.distance_since_anchor = 0.0;
      const auto back = open.update(held);
      check(std::string(back.integrity_mode) == "NOMINAL" && back.use_position,
            "a new station anchor is the recovery from common mode");
    }
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
    railbreak::IntegrityMonitor bound_mon{c};
    const auto bound = soak(bound_mon, fresh, fresh.t + 1.0);
    check(std::fabs(bound.along_bound_m - 9.0) < 1e-9, "bound is q * sigma + B_mode");
    {
      railbreak::BoundCoeff held_c;
      held_c.calibrated = true;
      held_c.q99 = 2.0;
      held_c.b_common = 32.606;
      railbreak::IntegrityMonitor held_mon{held_c};
      railbreak::IntegrityObs live;
      live.sigma_s = 1.0;
      live.front_age_s = 0.05;
      live.rear_age_s = 0.05;
      live.pair_fresh = true;
      live.bogies_agree = true;
      live.absolute_start = true;
      live.map_in_domain = true;
      live.t = 0.0;
      live.slip_front = true;
      live.slip_rear = true;
      live.nis_front = 20.0;
      live.nis_rear = 20.0;
      const auto during = soak(held_mon, live, 4.5);
      check(std::string(during.status) == "DEGRADED_COMMON_MODE_UNOBSERVABLE",
            "both bogies leaving the model degrade velocity once the score holds");
      check(std::string(during.confidence_velocity) == "degraded",
            "velocity confidence drops while the slip is on");
      check(std::string(during.integrity_mode) == "VELOCITY_DEGRADED" &&
                std::string(during.velocity_confidence) == "LOW" &&
                std::string(during.position_confidence) == "LOW",
            "a live common slip degrades speed and position together");
      live.t = 0.2;
      live.slip_front = true;
      live.slip_rear = false;
      live.nis_rear = 0.0;
      railbreak::IntegrityMonitor micro;
      const auto one = micro.update(live);
      check(std::string(one.confidence_position) == "ok",
            "one bogie slip does not hold position");
      live.t = 0.4;
      live.slip_front = false;
      live.nis_front = 0.0;
      const auto back = micro.update(live);
      check(std::string(back.status) == "NOMINAL" &&
                std::string(back.confidence_velocity) == "ok" &&
                std::string(back.confidence_position) == "ok",
            "a micro-slip clears both confidences");
      live.t = 4.5;
      live.slip_front = false;
      live.slip_rear = false;
      live.nis_front = 0.0;
      live.nis_rear = 0.0;
      const auto after = soak(held_mon, live, 8.5);
      check(std::string(after.status) == "NOMINAL",
            "velocity status recovers after the common-mode slip");
      check(std::string(after.confidence_velocity) == "ok", "velocity confidence recovers");
      check(std::string(after.confidence_position) == "degraded",
            "position confidence stays until an anchor");
      check(std::string(after.velocity_confidence) == "HIGH" &&
                std::string(after.position_confidence) == "LOW" &&
                std::string(after.integrity_mode) == "POSITION_DEGRADED",
            "after the slip, speed is high and the open position stays low");
      check(after.reasons.find("POSITION_OPEN") != std::string::npos,
            "the open position is named");
      check(std::fabs(after.along_bound_m - 34.606) < 1e-6,
            "the position margin stays after velocity recovers");
    }
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
  {
    // Master is absent. The only fix is the rover, 12.436 m ahead of the master.
    // Stepping that snap back, then adding the along offset, is base_link.
    const double s_master = 400.0;
    const double baseline = 12.436;
    const double off_along = 9.873;
    const auto line = flat_ring(2000.0);
    double lat = 0.0, lon = 0.0;
    line.map.latlon(line.map.at(line.map.x, s_master + baseline),
                    line.map.at(line.map.y, s_master + baseline), lat, lon);
    const auto snap = railbreak::init_on_ring(line.map, lat, lon, false, 0.0, 0.0);
    check(snap.ok, "a rover-only fix snaps onto the ring");
    const double published = railbreak::arc_from_rover_only(snap.s0, baseline) + off_along;
    const double base_link = s_master + off_along;
    check(std::fabs(published - base_link) < 1.0,
          "rover-only start steps back to the master arc, then the along offset is base_link");
    check(std::fabs((snap.s0 + off_along) - base_link) > 10.0,
          "treating the rover snap as the master leaves base_link about 12.4 m ahead");
  }
  {
    const double k0 = 1.0027;
    check(std::fabs(railbreak::apply_wheel_radius(k0, 0.0, 0.0) - k0) < 1e-15,
          "unset wheel radii leave the train scale; 0.35 m is not applied");
    const double mismatched = railbreak::apply_wheel_radius(k0, 0.31, 0.35);
    check(std::fabs(mismatched / k0 - 0.31 / 0.35) < 1e-12,
          "wheel_radius_mismatch scales k0 by the radius ratio when both are set");
    check(std::fabs(railbreak::apply_wheel_radius(k0, 0.31, 0.0) - k0) < 1e-15,
          "one unset radius does not scale k0");
    check(std::fabs(railbreak::wheel_scale_sigma(0.004, 0.0, 0.0, 0.0) - 0.004) < 1e-15,
          "an unset wheel radius leaves the scale uncertainty");
    check(std::fabs(railbreak::wheel_scale_sigma(0.004, 0.31, 0.35, 0.0035) -
                    std::hypot(0.004, 0.0035 / 0.35)) < 1e-12,
          "a set radius adds its relative uncertainty to sigma_k0");
  }
  {
    const double uncapped = railbreak::zupt_speed_threshold(0.05, 3.0, 1.0, 1e9);
    check(std::fabs(uncapped - 3.0) < 1e-9,
          "R=1 without a cap is a 3 m/s standstill gate");
    check(std::fabs(railbreak::zupt_speed_threshold(0.05, 3.0, 1.0, 0.5) - 0.5) < 1e-12,
          "zupt gate is capped at 0.5 m/s");
    check(std::fabs(railbreak::zupt_speed_threshold(0.05, 3.0, 0.05 * 0.05, 0.5) - 0.15) < 1e-12,
          "nominal wheel noise still uses 0.15 m/s, under the cap");

    railbreak::TrackOdometer moving(&assets, p);
    moving.init(0.0, 0.5);
    moving.set_time(0.0);
    double t = 0.0;
    int i = 0;
    for (; i < 800 && moving.noise_sd() * 3.0 < 2.5; ++i) {
      t = 0.1 * static_cast<double>(i);
      const double front_kmh = (i % 2 == 0) ? 72.0 : 0.0;
      moving.on_bogie(t, true, front_kmh);
      moving.on_bogie(t + 0.05, false, 36.0);
    }
    t = 0.1 * static_cast<double>(i);
    check(moving.noise_sd() * 3.0 > 2.0,
          "inflated wheel noise would have opened a gate above 2 m/s");
    for (double u = 0.0; u < 1.3; u += 0.1) {
      moving.on_bogie(t + u, true, 7.2);  // 2.0 m/s ground truth
      moving.on_bogie(t + u + 0.05, false, 7.2);
    }
    check(moving.mode() != railbreak::Mode::kZupt, "2 m/s ground truth is not a standstill");
    check(moving.v() > 1.0, "speed stays above 1 m/s while both bogies read 2 m/s");

    railbreak::TrackOdometer parked(&assets, p);
    parked.init(0.0, 0.5);
    parked.set_time(0.0);
    drive(parked, 0.0, 2.5, 0.0);
    check(parked.mode() == railbreak::Mode::kZupt, "true zero for 1 s is still ZUPT");
    check(std::fabs(parked.v()) < 1e-6, "a real stop sets speed to zero");
  }
  {
    auto line = flat_ring(2000.0);
    line.stops = {{100.0, 0.5, 10}};
    railbreak::TrackOdometer near(&line, p);
    near.init(98.0, 1.0);
    near.set_time(0.0);
    drive(near, 0.0, 2.0, 0.0);
    check(!near.anchor_log().empty() && near.n_anchor() == 1, "a dwell writes one anchor row");
    const auto& hit = near.anchor_log().back();
    check(hit.accepted && std::string(hit.reason) == "accepted",
          "a station inside the gate is accepted");
    check(std::fabs(hit.innovation) <= hit.gate + 1e-9, "accepted innovation is inside the gate");

    line.stops = {{100.0, 0.5, 10}, {110.0, 0.5, 10}};
    railbreak::TrackOdometer both(&line, p);
    both.init(105.0, 5.0);
    both.set_time(0.0);
    drive(both, 0.0, 2.0, 0.0);
    check(!both.anchor_log().empty() && both.n_anchor() == 0 &&
              std::string(both.anchor_log().back().reason) == "ambiguous",
          "two stations in the gate are not applied");

    line.stops = {{100.0, 0.5, 10}};
    railbreak::TrackOdometer far(&line, p);
    far.init(0.0, 1.0);
    far.set_time(0.0);
    drive(far, 0.0, 2.0, 0.0);
    const auto& miss = far.anchor_log().back();
    check(!miss.accepted && far.n_anchor() == 0 && std::string(miss.reason) == "outside_gate",
          "a station past the gate is rejected");
    check(std::fabs(miss.innovation) > miss.gate, "rejected innovation is outside the gate");
    check(std::fabs(miss.candidate_s - 100.0) < 1e-6, "the log names the nearest station");
    check(std::fabs(miss.predicted_s) < 1.0, "predicted_s is the filter arc at the dwell");
  }
  {
    // Both bogies follow the same speed. There is no wheel slip. The notch
    // table is 0.7 of that acceleration, the stated model-force error.
    auto line = flat_ring(4000.0);
    line.table.a[15].assign(18, 0.7);
    railbreak::TrackOdometer od(&line, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    for (int i = 0; i < 80; ++i) {
      const double t = 0.1 * i;
      const double v_ms = 5.0 + t;
      const double kmh = v_ms * 3.6;
      od.on_bogie(t, true, kmh);
      od.on_bogie(t + 0.05, false, kmh);
      od.on_cmd(t + 0.025, 0);
    }
    check(!od.slip_front() && !od.slip_rear(),
          "a 0.7 model force does not leave a slip flag");
    check(od.slip_age_s() < 0.2, "that model error does not hold slip");
    check(std::fabs(od.v() - 12.9) < 0.4, "speed stays with the agreeing bogies");
    check(od.mode() == railbreak::Mode::kWheels, "mode returns to the wheels");

    auto harsh = flat_ring(4000.0);
    harsh.table.a[15].assign(18, 5.0);
    railbreak::TrackOdometer bad(&harsh, p);
    bad.init(0.0, 0.5);
    bad.set_time(0.0);
    bool saw_slip = false;
    for (int i = 0; i < 80; ++i) {
      const double t = 0.1 * i;
      bad.on_bogie(t, true, 36.0);
      bad.on_bogie(t + 0.05, false, 36.0);
      bad.on_cmd(t + 0.025, 0);
      if (t > 0.5 && t < 2.5 && (bad.slip_front() || bad.slip_rear())) saw_slip = true;
    }
    check(saw_slip, "a model step past the slip gate is noticed before recover");
    check(bad.common_unobservable() && bad.mode() == railbreak::Mode::kCommon,
          "after recover the agreeing bogies stay unobservable");
    check(bad.slip_front() && bad.slip_rear(),
          "agreement does not clear the slip flags");
    check(std::fabs(bad.v() - 10.0) > 2.0,
          "speed is not copied from the agreeing bogies");
    const double k_hold = bad.k();
    const double ba_hold = bad.model_bias();
    for (int i = 0; i < 20; ++i) {
      const double t = 8.0 + 0.1 * i;
      bad.on_bogie(t, true, 36.0);
      bad.on_bogie(t + 0.05, false, 36.0);
      bad.on_cmd(t + 0.025, 0);
    }
    check(std::fabs(bad.k() - k_hold) < 1e-9 && std::fabs(bad.model_bias() - ba_hold) < 1e-9,
          "common mode does not adapt k or the model bias");
    check(bad.sigma_v() > 1.0, "common mode keeps a growing speed uncertainty");
  }
  {
    railbreak::InputReorder<int> q;
    q.set_hold(0.10);
    q.push(1.20, 20);
    q.push(1.05, 5);
    q.push(1.12, 12);
    const auto first = q.drain();
    check(first.ready.size() == 1 && first.ready[0].payload == 5 && first.ready[0].t == 1.05,
          "only a stamp at least 0.10 s behind the newest is released");
    q.push(1.30, 30);
    const auto next = q.drain();
    check(next.ready.size() == 2 && next.ready[0].payload == 12 && next.ready[1].payload == 20,
          "held bogie and notch stamps are released in stamp order");
    check(q.pending() == 1, "the newest stamp stays until a later one arrives");
    railbreak::InputReorder<int> streams;
    streams.set_hold(0.10);
    streams.set_stall(1.0);
    streams.push(1.00, 1, 1);
    streams.push(1.00, 2, 2);
    streams.push(1.40, 0, 0);
    streams.push(1.10, 3, 0);
    const auto held = streams.drain();
    check(held.ready.empty() && held.reason[0] == '\0',
          "a fast front stream does not release past the slower bogie and notch");
    streams.push(1.30, 4, 1);
    streams.push(1.30, 5, 2);
    const auto ordered = streams.drain();
    check(ordered.reason[0] == '\0' && ordered.ready.size() == 3 &&
              ordered.ready[0].t == 1.00 && ordered.ready[1].t == 1.00 &&
              ordered.ready[2].payload == 3 && ordered.ready[2].t == 1.10,
          "streams are released in stamp order up to the slowest watermark");
    check(streams.pending() == 3, "samples ahead of the watermark stay queued");
    railbreak::InputReorder<int> stalled;
    stalled.set_hold(0.10);
    stalled.set_stall(1.0);
    stalled.push(5.0, 0, 0);
    stalled.push(5.0, 1, 1);
    stalled.push(3.0, 2, 2);
    const auto lost = stalled.drain();
    check(std::string(lost.reason) == "ORDER_NOT_RESTORED" && !lost.ready.empty() &&
              lost.ready[0].payload == 2,
          "a stream more than the stall behind is named and its sample is not discarded");
  }
  {
    const auto missing =
        railbreak::classify_source(false, 0.0, 1.0e9, 0.35, 1.0 / 3.6, 0, 0, 0, "none", false);
    check(std::string(missing.kind) == "missing" && !missing.valid && missing.quality_score == 0.0,
          "no sample is missing, not one degraded label");
    const auto stale =
        railbreak::classify_source(true, 1.0, 1.0, 0.35, 1.0 / 3.6, 1, 0, 0, "none", false);
    check(std::string(stale.kind) == "stale" && stale.dropout_counter == 1, "an old sample is stale");
    const auto outlier =
        railbreak::classify_source(true, 1.0, 0.0, 0.35, 1.0 / 3.6, 0, 2, 0, "outlier", false);
    check(std::string(outlier.kind) == "outlier", "a non-finite sample is an outlier");
    const auto impossible =
        railbreak::classify_source(true, 1.0, 0.0, 0.35, 1.0 / 3.6, 0, 0, 1, "impossible", false);
    check(std::string(impossible.kind) == "impossible", "a speed past v_max is impossible");
    const auto split =
        railbreak::classify_source(true, 1.0, 0.0, 0.35, 1.0 / 3.6, 0, 0, 0, "none", true);
    check(std::string(split.kind) == "disagree" && split.valid,
          "disagreement with the other source stays its own kind");
    const auto okq =
        railbreak::classify_source(true, 1.0, 0.0, 0.35, 1.0 / 3.6, 0, 0, 0, "none", false);
    check(std::string(okq.kind) == "ok" && okq.quality_score == 1.0 && okq.valid,
          "a fresh agreeing sample is ok");

    auto line = flat_ring(4000.0);
    railbreak::TrackOdometer od(&line, p);
    od.init(0.0, 0.5);
    od.set_time(0.0);
    od.on_bogie(0.0, true, 36.0);
    od.on_bogie(0.05, false, 36.0);
    check(std::string(od.bogie_quality(true).kind) == "ok", "a fresh bogie is ok");
    od.on_bogie(0.2, true, std::numeric_limits<double>::quiet_NaN());
    check(std::string(od.bogie_quality(true).kind) == "outlier" &&
              od.bogie_quality(true).outlier_counter == 1,
          "NaN increments the front outlier counter");
    check(std::string(od.bogie_quality(false).kind) == "ok", "the rear kind stays ok");
    od.on_bogie(0.25, true, 200.0);
    check(std::string(od.bogie_quality(true).kind) == "impossible" &&
              od.bogie_quality(true).impossible_counter == 1,
          "200 km/h is physically impossible after the unit scale");
    od.on_bogie(0.3, true, 36.0);
    od.on_bogie(1.0, true, 36.0);
    check(od.bogie_quality(true).dropout_counter == 1,
          "a gap past the stale window counts one dropout");
    check(std::string(od.cmd_quality().kind) == "missing", "no controller message is missing");
    od.on_cmd(1.0, 99);
    check(std::string(od.cmd_quality().kind) == "outlier" && od.cmd_quality().outlier_counter == 1,
          "a notch outside -15..15 is an outlier");
  }
  {
    // Flat traction cruise. The table keeps adding 0.4 m/s^2. Wheels hold
    // 10 m/s. After the notch window and five uniform seconds, ba stops.
    auto line = flat_ring(4000.0);
    line.table.a[22].assign(18, 0.4);
    railbreak::TrackOdometer cruise(&line, p);
    cruise.init(0.0, 0.5);
    cruise.set_time(0.0);
    bool saw_frozen = false;
    double ba_at_freeze = 0.0;
    double sig_at_freeze = 0.0;
    double ba_later = 0.0;
    double sig_later = 0.0;
    for (int i = 0; i < 160; ++i) {
      const double t = 0.1 * i;
      cruise.on_cmd(t, 7);
      cruise.on_bogie(t + 0.02, true, 36.0);
      cruise.on_bogie(t + 0.06, false, 36.0);
      if (!saw_frozen && cruise.params_frozen()) {
        saw_frozen = true;
        ba_at_freeze = cruise.model_bias();
        sig_at_freeze = cruise.sigma_ba();
      }
      if (t > 14.0) {
        ba_later = cruise.model_bias();
        sig_later = cruise.sigma_ba();
      }
    }
    check(saw_frozen && std::string(cruise.drive_segment()) == "uniform",
          "a long flat traction cruise freezes the bias");
    check(std::fabs(ba_later - ba_at_freeze) < 1e-6,
          "the frozen bias mean does not keep training");
    check(sig_later > sig_at_freeze, "frozen bias uncertainty still grows");

    auto coast_line = flat_ring(4000.0);
    coast_line.table.a[15].assign(18, -0.3);
    railbreak::TrackOdometer coast(&coast_line, p);
    coast.init(0.0, 0.5);
    coast.set_time(0.0);
    double ba_mid = 0.0;
    for (int i = 0; i < 120; ++i) {
      const double t = 0.1 * i;
      coast.on_cmd(t, 0);
      coast.on_bogie(t + 0.02, true, 36.0);
      coast.on_bogie(t + 0.06, false, 36.0);
      if (std::fabs(t - 6.0) < 0.05) ba_mid = coast.model_bias();
    }
    check(!coast.params_frozen() && std::string(coast.drive_segment()) == "coast",
          "coast stays an identification segment");
    check(std::fabs(coast.model_bias() - ba_mid) > 1e-4,
          "on coast the bias is still allowed to move");
    check(std::string(coast.drive_segment()) != "uniform", "coast is not labelled uniform");
  }
  {
    railbreak::ModelBank matched;
    matched.init(0.0, 10.0);
    for (int i = 0; i < 30; ++i) matched.step(0.1, 0.0, 0.0, 10.0, 0.05 * 0.05);
    const auto& steady = matched.consensus();
    double sum_c = 0.0;
    for (int j = 0; j < railbreak::kModelCount; ++j) {
      sum_c += steady.mode[j].confidence;
      check(std::isfinite(steady.mode[j].s) && std::isfinite(steady.mode[j].v) &&
                std::isfinite(steady.mode[j].likelihood) && std::isfinite(steady.mode[j].p_vv),
            "each mode keeps state, covariance, innovation and likelihood");
    }
    check(std::fabs(sum_c - 1.0) < 1e-9, "mode confidences sum to one");
    check(steady.mode[railbreak::kModelNominal].confidence >
              steady.mode[railbreak::kModelWheelScale].confidence,
          "a matched cruise trusts the table over a 5 percent wheel-scale hypothesis");
    check(std::fabs(steady.v - 10.0) < 0.3, "matched consensus speed stays with the wheels");

    railbreak::ModelBank delayed;
    delayed.init(0.0, 10.0);
    for (int i = 0; i < 40; ++i) delayed.step(0.1, 1.0, 0.0, 10.0, 0.05 * 0.05);
    const auto& lag = delayed.consensus();
    check(lag.mode[railbreak::kModelDelay].confidence > lag.mode[railbreak::kModelNominal].confidence,
          "wheels that ignore a new notch raise the actuator-delay mode");

    railbreak::ModeEstimate modes[railbreak::kModelCount];
    const double weight[railbreak::kModelCount] = {0.50, 0.15, 0.10, 0.15, 0.10};
    for (int j = 0; j < railbreak::kModelCount; ++j) {
      modes[j].s = 0.0;
      modes[j].v = 10.0;
      modes[j].p_ss = 1.0;
      modes[j].p_vv = 0.25;
      modes[j].confidence = weight[j];
    }
    modes[railbreak::kModelAdhesion].s = 40.0;
    const auto mix = railbreak::fuse_modes(modes, railbreak::ModelBank::kSpreadGateM);
    double mean_s = 0.0;
    for (int j = 0; j < railbreak::kModelCount; ++j) mean_s += modes[j].confidence * modes[j].s;
    check(mix.outlier_rejected && mix.n_used == railbreak::kModelCount - 1,
          "a hypothesis 40 m from the leader is left out of the mixture");
    check(std::fabs(mix.s) < std::fabs(mean_s),
          "consensus stays nearer the leader than the probability-weighted mean");
  }
  {
    railbreak::SlipEvidence both;
    both.pair_fresh = true;
    both.bogies_agree = true;
    both.nis_front = 1.0;
    both.nis_rear = 1.0;
    both.notch = 7;
    check(std::string(railbreak::residual_pattern(both, 16.0)) == "nominal",
          "agreeing bogies and the model are the nominal pattern");

    railbreak::SlipEvidence one = both;
    one.bogies_agree = false;
    one.nis_front = 40.0;
    one.front_innov = 2.0;
    check(std::string(railbreak::instant_hypothesis(one, railbreak::residual_pattern(one, 16.0))) ==
              "single_bogie",
          "one bogie off the model is that bogie, not a spin");

    railbreak::SlipEvidence chaos = one;
    chaos.nis_rear = 40.0;
    chaos.rear_innov = -2.0;
    check(std::string(railbreak::residual_pattern(chaos, 16.0)) == "chaotic",
          "bogies that miss the model in opposite directions are chaotic");
    check(std::string(railbreak::instant_hypothesis(chaos, "chaotic")) == "sensor_fault",
          "a chaotic pair is a sensor or sync fault");

    railbreak::SlipEvidence spin = both;
    spin.nis_front = spin.nis_rear = 40.0;
    spin.front_innov = spin.rear_innov = 1.5;
    spin.notch = 7;
    check(std::string(railbreak::instant_hypothesis(spin, "common")) == "spin",
          "traction with both wheels faster than the model is spin");
    spin.delay_likely = true;
    check(std::string(railbreak::instant_hypothesis(spin, "common")) == "delay",
          "the same residual is a delay when that hypothesis is ahead");
    spin.delay_likely = false;
    spin.notch = 0;
    check(std::string(railbreak::instant_hypothesis(spin, "common")) == "model_mismatch",
          "coast does not turn a common residual into spin");
    spin.notch = -5;
    spin.front_innov = spin.rear_innov = -1.5;
    check(std::string(railbreak::instant_hypothesis(spin, "common")) == "slide",
          "braking with both wheels slower than the model is slide");

    railbreak::SlipDiagnosis diag;
    railbreak::SlipEvidence live = both;
    live.t = 0.0;
    live.nis_front = live.nis_rear = 40.0;
    live.front_innov = live.rear_innov = 1.5;
    live.notch = 7;
    auto first = diag.update(live);
    check(std::string(first.phase) == "idle" && std::string(first.hypothesis) == "nominal",
          "one high residual is not yet a confirmed slip");
    live.t = 0.20;
    auto mid = diag.update(live);
    check(std::string(mid.phase) == "candidate" && std::string(mid.hypothesis) == "spin",
          "the same residual for 150 ms is a candidate");
    live.t = 0.50;
    auto yes = diag.update(live);
    check(std::string(yes.phase) == "confirmed" && std::string(yes.hypothesis) == "spin",
          "the same residual for 400 ms is confirmed");
    live.notch = -5;
    live.front_innov = live.rear_innov = -1.5;
    live.t = 0.70;
    auto held = diag.update(live);
    check(std::string(held.hypothesis) == "spin" && std::string(held.phase) == "confirmed",
          "a new sign does not replace a confirmed label before its own confirm time");
    live = both;
    live.t = 0.80;
    auto rec = diag.update(live);
    check(std::string(rec.phase) == "recovery" && std::string(rec.hypothesis) == "spin",
          "a quiet residual starts recovery and keeps the label");
    live.t = 1.90;
    auto done = diag.update(live);
    check(std::string(done.phase) == "idle" && std::string(done.hypothesis) == "nominal",
          "a quiet agreed residual for 1 s clears the label");
  }
  {
    const auto nom = railbreak::motion_interval(100.0, 10.0, 1.0, 0.2, 0.004, 8.0, true, 0.0, 0.0,
                                                0.0, 0.0, false);
    check(nom.valid && !nom.claimed_percentile, "the envelope is not a claimed percentile");
    check(std::fabs(nom.e_s - 8.0) < 1e-12 && std::fabs(nom.s_min - 92.0) < 1e-12 &&
              std::fabs(nom.s_max - 108.0) < 1e-12,
          "with no extra term the half-width is the along-track bound");
    check(std::fabs(nom.v_min - (10.0 - nom.e_v)) < 1e-12 && nom.e_v > 0.2,
          "speed half-width includes sigma_v and the scale");
    const auto grown = railbreak::motion_interval(100.0, 10.0, 1.0, 0.2, 0.004, 8.0, true, 1000.0,
                                                  2.0, 3.0, 0.5, false);
    check(std::fabs(grown.e_s - (8.0 + 4.0 + 0.4 + 3.0)) < 1e-9,
          "scale since the anchor, unverified time and model gap widen the arc");
    check(std::fabs(grown.sigma_scale - 4.0) < 1e-12 && std::fabs(grown.sigma_model - 3.0) < 1e-12 &&
              std::fabs(grown.sigma_timestamp - 0.4) < 1e-12 && grown.sigma_map == 0.0 &&
              grown.sigma_common_mode == 0.0,
          "the extra width is named and is not written into sigma_s");
    check(std::string(railbreak::IntegrityReport{}.bound_statement) ==
              "empirical bound, not certified protection level",
          "the envelope states that it is an empirical bound");
    const auto lost = railbreak::motion_interval(100.0, 10.0, 1.0, 0.2, 0.004, 8.0, true, 0.0, 0.0,
                                                 0.0, 0.0, true);
    check(!lost.valid && !std::isfinite(lost.s_min), "LOST does not publish an interval");
    check(std::fabs(railbreak::time_to_lost(false, 0.05, 1.0, 30.0) - 29.95) < 1e-9,
          "time to LOST is the silence left until the fresher bogie ages out");
    check(railbreak::time_to_lost(true, 0.05, 0.05, 30.0) == 0.0, "an estimate already LOST has no time left");
  }
  {
    railbreak::IntegrityObs h;
    h.t = 0.0;
    h.sigma_s = 1.0;
    h.front_age_s = 0.05;
    h.rear_age_s = 0.05;
    h.pair_fresh = true;
    h.bogies_agree = true;
    h.absolute_start = true;
    h.map_in_domain = true;
    h.nis_front = 20.0;
    h.slip_front = true;
    h.degrade_recover_s = 1.0;
    railbreak::IntegrityMonitor held;
    check(std::string(held.update(h).status) == "NOMINAL",
          "one fault sample does not enter degraded");
    const auto entered = soak(held, h, 1.0);
    check(std::string(entered.status) == "DEGRADED_SINGLE_BOGIE" &&
              std::string(entered.fault_level) == "degraded",
          "a fault score above 0.8 for 0.5 s is DEGRADED, with no second confirm");
    h.nis_front = 40.0;
    const auto deep = soak(held, h, 3.2);
    check(std::string(deep.fault_level) == "deep" &&
              deep.reasons.find("DEEP_FAULT") != std::string::npos,
          "a fault score above 0.95 for 2 s is deep degraded");
    h.slip_front = false;
    h.nis_front = 0.0;
    h.t = 3.3;
    check(std::string(held.update(h).status) == "DEGRADED_SINGLE_BOGIE",
          "one quiet sample does not clear the fault score");
    const auto cleared = soak(held, h, 7.0);
    check(std::string(cleared.status) == "NOMINAL" && std::string(cleared.fault_level) == "nominal",
          "a fault score below 0.2 for 3 s clears degraded");
    railbreak::IntegrityObs bare = h;
    bare.t = 0.0;
    bare.slip_front = false;
    bare.nis_front = 0.0;
    bare.map_in_domain = false;
    check(std::string(railbreak::IntegrityMonitor{}.update(bare).status) == "DEGRADED_NO_MAP",
          "a missing map is DEGRADED on the first sample");
    check(std::string(railbreak::IntegrityMonitor{}.update(bare).confidence_position) == "degraded",
          "a missing map does not keep position confidence ok");
  }
  {
    railbreak::TrackMap line;
    for (int i = 0; i <= 200; ++i) {
      line.s.push_back(i);
      line.x.push_back(i);
      line.y.push_back(0.0);
      line.h.push_back(0.0);
      line.grade.push_back(0.0);
    }
    line.ring_len = 1000.0;
    const auto step = railbreak::match_ring(line, 10.0, 11.0, 11.0, 0.4, 10.0, 0.1, nullptr, 0);
    check(std::string(step.candidate_path) == "ring" && std::fabs(step.candidate_s - 11.0) < 0.05 &&
              std::fabs(step.cross_track_error - 0.4) < 0.05 && std::fabs(step.along_track_error) < 0.05 &&
              step.branch_probability > 0.8,
          "a short step stays on the ring");
    const auto jump = railbreak::match_ring(line, 10.0, 80.0, 80.0, 0.0, 10.0, 0.1, nullptr, 0);
    check(std::string(jump.candidate_path) == "ring" && std::fabs(jump.candidate_s - 18.0) < 0.2 &&
              std::fabs(jump.along_track_error) > 50.0 && jump.branch_probability < 0.8,
          "a longitudinal jump is limited by the step window");
    const auto back = railbreak::match_ring(line, 10.0, 5.0, 5.0, 0.0, 5.0, 0.1, nullptr, 0);
    check(std::fabs(back.candidate_s - 10.0) < 1e-9 && back.along_track_error < -4.0 &&
              back.branch_probability <= 0.25,
          "reverse motion at speed is rejected");
    railbreak::Stop one[] = {{50.0, 0.5}};
    const auto held = railbreak::match_ring(line, 10.0, 11.0, 11.0, 0.0, 10.0, 0.1, one, 1);
    check(std::fabs(held.candidate_s - 11.0) < 0.05, "a stop does not pull the candidate arc");
    railbreak::Stop two[] = {{10.0, 0.5}, {14.0, 0.5}};
    const auto ambiguous = railbreak::match_ring(line, 10.0, 12.0, 12.0, 0.0, 10.0, 0.1, two, 2);
    check(ambiguous.branch_probability <= 0.5 && std::fabs(ambiguous.candidate_s - 10.0) > 0.5 &&
              std::fabs(ambiguous.candidate_s - 14.0) > 0.5,
          "two stops in the gate lower the branch probability and do not snap");

    railbreak::TrackMap fork;
    for (int i = 0; i <= 4; ++i) {
      fork.s.push_back(i);
      fork.x.push_back(i);
      fork.y.push_back(0.0);
    }
    fork.s.push_back(20.0);
    fork.x.push_back(1.0);
    fork.y.push_back(2.0);
    fork.s.push_back(21.0);
    fork.x.push_back(2.0);
    fork.y.push_back(2.0);
    fork.h.assign(fork.s.size(), 0.0);
    fork.grade.assign(fork.s.size(), 0.0);
    fork.ring_len = 30.0;
    const auto junction = railbreak::match_ring(fork, 1.0, 1.2, 1.05, 1.7, 2.0, 0.1, nullptr, 0);
    check(std::string(junction.candidate_path) == "junction" && junction.candidate_s < 8.0 &&
              junction.branch_probability > 0.5 && junction.cross_track_error > 0.15,
          "a nearer loop outside the window is named and not taken");
  }
  {
    auto step = [](railbreak::FaultScore& score, double t, double nis_front, double nis_rear) {
      railbreak::FaultObs o;
      o.t = t;
      o.nis_front = nis_front;
      o.nis_rear = nis_rear;
      o.pair_fresh = true;
      o.bogies_agree = true;
      return score.update(o);
    };
    railbreak::FaultScore clean;
    railbreak::FaultState clean_end{};
    for (int i = 0; i <= 20; ++i) clean_end = step(clean, 0.1 * i, 0.0, 0.0);
    check(!clean_end.front_latched && !clean_end.rear_latched && !clean_end.common_latched,
          "a clean 2 s replay latches neither bogie");
    railbreak::FaultScore single;
    railbreak::FaultState single_end{};
    for (int i = 0; i <= 20; ++i) single_end = step(single, 0.1 * i, 20.0, 0.0);
    check(single_end.front_latched && !single_end.rear_latched && !single_end.common_latched,
          "a single-bogie replay latches only the faulty bogie");
    railbreak::FaultScore stagger;
    bool rear_early = false;
    bool rear_at_one = true;
    railbreak::FaultState at_one{};
    for (int i = 0; i <= 20; ++i) {
      const double t = 0.1 * i;
      const auto s = step(stagger, t, 20.0, t + 1e-12 >= 0.6 ? 20.0 : 0.0);
      if (t < 1.1 && s.rear_latched) rear_early = true;
      if (std::fabs(t - 1.0) < 1e-9) {
        at_one = s;
        rear_at_one = s.rear_latched;
      }
    }
    check(!rear_early && !rear_at_one && at_one.front_latched,
          "rear fault at 0.6 s does not latch before 1.1 s");
  }
  {
    // One lap on a short ring. The published arc is near the start again.
    // The anchor budget must still see the path that was actually travelled.
    auto loop = flat_ring(100.0);
    loop.stops.clear();
    railbreak::TrackOdometer od(&loop, p);
    od.init(0.0, 1.0);
    od.set_time(0.0);
    drive(od, 0.0, 2.0, 36.0);
    check(od.distance_since_anchor() > 15.0 && od.distance_since_anchor() < 30.0,
          "two seconds of travel report the path, not a doubled one");
    drive(od, 2.0, 11.0, 36.0);
    check(od.s() < 20.0, "after one lap the published arc is near the start");
    check(od.distance_since_anchor() > 100.0,
          "a nearly full lap does not fold the anchor path");
    railbreak::IntegrityObs blind;
    blind.t = 1.0;
    blind.sigma_s = 1.0;
    blind.front_age_s = 0.05;
    blind.rear_age_s = 0.05;
    blind.pair_fresh = true;
    blind.bogies_agree = true;
    blind.absolute_start = true;
    blind.map_in_domain = true;
    blind.common_unobservable = true;
    blind.distance_since_anchor = od.distance_since_anchor();
    const auto lost = railbreak::IntegrityMonitor{}.update(blind);
    check(std::string(lost.status) == "LOST" && !lost.use_position &&
              lost.time_to_lost == 0.0 && lost.blind_time_s < 5.0,
          "the lap path spends the distance budget before the 5 s clock");

    // Wheels drop to zero while the filter is still near 10 m/s, so the dwell
    // lands about 11 m past the five-second mark. The station is that dwell.
    auto line = flat_ring(2000.0);
    line.stops = {{61.0, 0.5, 10}};
    railbreak::TrackOdometer parked(&line, p);
    parked.init(0.0, 1.0);
    parked.set_time(0.0);
    drive(parked, 0.0, 5.0, 36.0);
    const double before = parked.distance_since_anchor();
    drive(parked, 5.0, 8.0, 0.0);
    check(before > 40.0 && parked.n_anchor() == 1 && parked.distance_since_anchor() < 5.0,
          "an accepted station clears the path since the anchor");
  }
  std::printf("%s\n", g_fail ? "FAILED" : "all passed");
  return g_fail ? 1 : 0;
}

// Backup odometry node for the "Резервная одометрия по модели" contract.
//
// Inputs (main loop):  /vehicle/front_bogie_velocity, /vehicle/rear_bogie_velocity
//                      (VelocitySensor), /vehicle/driver_position_cmd (DriverControllerCommand).
// Start only:          /sensing/gnss/{master,rover}/fix for gnss_init_window_s from the
//                      first valid fix of either antenna. The arc is anchored at the
//                      stamp of the authoritative sample, then both subscriptions
//                      are destroyed.
// Outputs:             /result/velocity (VelocitySensor, m/s), /result/position (Odometry,
//                      MGRS metres of base_link, see output_frame),
//                      /result/diagnostics.
//
// Every output is published from the input callback with that input's header.stamp.
// There is no wall timer and no dependence on /clock.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tram_vehicle_msgs/msg/driver_controller_command.hpp"
#include "tram_vehicle_msgs/msg/velocity_sensor.hpp"

#include "railbreak_backup_odometry/adhesion_proxy.hpp"
#include "railbreak_backup_odometry/gnss_window.hpp"
#include "railbreak_backup_odometry/integrity_bound.hpp"
#include "railbreak_backup_odometry/start_epoch.hpp"
#include "railbreak_backup_odometry/track_odometer.hpp"

using tram_vehicle_msgs::msg::DriverControllerCommand;
using tram_vehicle_msgs::msg::VelocitySensor;

namespace {

double stamp_s(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

double median(std::vector<double> v) { return railbreak::upper_median(std::move(v)); }

}  // namespace

struct GnssDrainProbe;

class BackupOdometryNode : public rclcpp::Node {
 public:
  explicit BackupOdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("backup_odometry", options) {
    std::string dir = declare_parameter("assets_dir", std::string(""));
    if (dir.empty()) {
      dir = ament_index_cpp::get_package_share_directory("railbreak_backup_odometry") + "/assets";
    }
    win_.window_s = declare_parameter("gnss_init_window_s", 3.0);
    win_.wait_s = declare_parameter("gnss_wait_s", 10.0);
    diag_every_ = std::max<int64_t>(1, declare_parameter("diagnostics_every_n", 20));
    frame_id_ = declare_parameter("frame_id", std::string("map"));
    child_frame_id_ = declare_parameter("child_frame_id", std::string("base_link"));
    const std::string mode = declare_parameter("output_frame", std::string("mgrs"));
    if (mode == "mkrs_start") {
      frame_.mode = railbreak::FrameMode::kMkrsStart;
    } else if (mode == "mkrs") {
      frame_.mode = railbreak::FrameMode::kMkrs;
    } else if (mode == "mgrs") {
      frame_.mode = railbreak::FrameMode::kMgrs;
    } else if (mode == "grid_start") {
      frame_.mode = railbreak::FrameMode::kGridStart;
    } else if (mode == "enu") {
      frame_.mode = railbreak::FrameMode::kEnu;
    } else {
      frame_.mode = railbreak::FrameMode::kMgrs;
      RCLCPP_ERROR(get_logger(), "output_frame '%s' unknown; using mgrs", mode.c_str());
    }
    frame_.x_is_north = declare_parameter("mkrs_x_is_north", false);
    frame_.zone = static_cast<int>(declare_parameter("utm_zone", 37));
    frame_.e_off = declare_parameter("mgrs_square_easting", 300000.0);
    frame_.n_off = declare_parameter("mgrs_square_northing", 6100000.0);
    off_along_ = declare_parameter("output_offset_along_m", 9.873);
    off_up_ = declare_parameter("output_offset_up_m", -3.0);
    rover_baseline_m_ = declare_parameter("rover_baseline_m", 12.436);
    initial_s_ = declare_parameter("initial_s_m", std::numeric_limits<double>::quiet_NaN());
    initial_lat_ = declare_parameter("initial_lat_deg", std::numeric_limits<double>::quiet_NaN());
    initial_lon_ = declare_parameter("initial_lon_deg", std::numeric_limits<double>::quiet_NaN());
    wheel_r_ = declare_parameter("wheel_radius_m", 0.0);
    wheel_r0_ = declare_parameter("wheel_radius_nominal_m", 0.0);
    railbreak::Params p;
    p.unit = declare_parameter("wheel_unit_scale", 1.0 / 3.6);
    p.load_factor = declare_parameter("load_factor", 1.0);
    p.davis_a = declare_parameter("davis_a", 0.0);
    p.davis_b = declare_parameter("davis_b", 0.0);
    p.davis_c = declare_parameter("davis_c", 0.0);
    p.sigma_k0 = declare_parameter("sigma_k0", p.sigma_k0);
    p.q_v = declare_parameter("q_v", p.q_v);
    p.q_s = declare_parameter("q_s", p.q_s);
    p.q_k = declare_parameter("q_k", p.q_k);
    p.q_ba = declare_parameter("q_ba", p.q_ba);
    p.nis_gate = declare_parameter("nis_gate", p.nis_gate);
    p.fr_sigma_gate = declare_parameter("fr_sigma_gate", p.fr_sigma_gate);
    p.fr_floor = declare_parameter("fr_floor", p.fr_floor);
    p.zupt_v = declare_parameter("zupt_v", p.zupt_v);
    p.zupt_hold_s = declare_parameter("zupt_hold_s", p.zupt_hold_s);
    p.stop_gate = declare_parameter("stop_gate", p.stop_gate);
    try {
      assets_ = railbreak::load_assets(dir);
    } catch (const std::exception& e) {
      // A missing directory is relative odometry. A map that opened and then
      // failed validation is a bad configuration: do not start on it.
      const std::string msg = e.what();
      if (msg.find("cannot open") == std::string::npos) {
        RCLCPP_FATAL(get_logger(), "assets rejected (%s)", e.what());
        throw;
      }
      RCLCPP_ERROR(get_logger(), "assets not loaded (%s): speed and relative odometry only", e.what());
      assets_ = railbreak::Assets{};
      assets_.k0 = 1.0;
      assets_ok_ = false;
    }
    if (wheel_r_ > 0.0 && wheel_r0_ > 0.0) assets_.k0 *= wheel_r_ / wheel_r0_;
    const bool have_ring = assets_ok_ && !assets_.map.empty();
    railbreak::validate_geometry(win_.window_s, win_.wait_s, off_along_, off_up_, rover_baseline_m_,
                                 initial_s_, have_ring ? assets_.map.ring_len : 0.0, have_ring);
    od_ = std::make_unique<railbreak::TrackOdometer>(&assets_, p);

    // Depth 10 dropped the start window at ros2 bag play --rate 10 (~400 msg/s)
    // and left output gaps up to 2 s. Best effort stays: a reliable subscriber
    // does not connect to a best-effort publisher.
    const auto in_qos = rclcpp::SensorDataQoS().keep_last(500);
    const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    pub_v_ = create_publisher<VelocitySensor>("/result/velocity", out_qos);
    pub_p_ = create_publisher<nav_msgs::msg::Odometry>("/result/position", out_qos);
    pub_d_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/result/diagnostics", 10);

    drain_gnss_queue_ = declare_parameter("drain_gnss_queue", true);
    const bool gnss_first = declare_parameter("gnss_subscribe_first", false);
    auto make_wheels = [&]() {
      sub_f_ = create_subscription<VelocitySensor>(
          "/vehicle/front_bogie_velocity", in_qos,
          [this](const VelocitySensor& m) { on_bogie(m, true); });
      sub_r_ = create_subscription<VelocitySensor>(
          "/vehicle/rear_bogie_velocity", in_qos,
          [this](const VelocitySensor& m) { on_bogie(m, false); });
      sub_c_ = create_subscription<DriverControllerCommand>(
          "/vehicle/driver_position_cmd", in_qos,
          [this](const DriverControllerCommand& m) { on_cmd(m); });
    };
    auto make_gnss = [&]() {
      sub_gm_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          "/sensing/gnss/master/fix", in_qos,
          [this](const sensor_msgs::msg::NavSatFix& m) { on_fix(m, true); });
      sub_gr_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          "/sensing/gnss/rover/fix", in_qos,
          [this](const sensor_msgs::msg::NavSatFix& m) { on_fix(m, false); });
    };
    if (gnss_first) {
      make_gnss();
      make_wheels();
    } else {
      make_wheels();
      make_gnss();
    }
    RCLCPP_INFO(get_logger(), "assets %s: ring %.1f m, %zu stops, k0 %.5f; GNSS window %.1f s",
                dir.c_str(), assets_.map.ring_len, assets_.stops.size(), assets_.k0, win_.window_s);
  }

 private:
  // --- start window --------------------------------------------------------
  void on_fix(const sensor_msgs::msg::NavSatFix& m, bool master) {
    if (win_.closed) return;
    const double t = stamp_s(m.header.stamp);
    const bool valid = railbreak::gnss_fix_ok(
        t, m.latitude, m.longitude, m.altitude, m.status.status,
        sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    const auto action = win_.on_fix(master, t, valid);
    if (valid) {
      const bool in_window = win_.t_open >= 0.0 && t <= win_.t_open + win_.window_s;
      if (master && in_window) {
        m_lat_.push_back(m.latitude);
        m_lon_.push_back(m.longitude);
        m_alt_.push_back(m.altitude);
        m_t_.push_back(t);
      } else if (!master && in_window) {
        r_lat_.push_back(m.latitude);
        r_lon_.push_back(m.longitude);
        r_alt_.push_back(m.altitude);
        r_t_.push_back(t);
      }
    }
    if (action == railbreak::GnssWindow::Action::kFinish) finish_init();
  }

  void on_window_input(double t) {
    const auto action = win_.on_input(t);
    if (action == railbreak::GnssWindow::Action::kRelative) {
      if (manual_start()) return;
      relative_ = true;
      close_gnss("no fix within gnss_wait_s; relative odometry from the start point");
    } else if (action == railbreak::GnssWindow::Action::kFinish) {
      // Eight quiet callbacks do not prove the GNSS queue is empty. Take what
      // the middleware already holds; a valid fix there resets the drain.
      if (!(drain_gnss_queue_ && absorb_queued_gnss())) finish_init();
    }
  }

  // True if a queued fix was applied or the window already closed from one.
  // Both subscriptions are drained, then ordered by header.stamp. Index
  // interleaving closes on a later stamp before an earlier fix still sitting
  // in the other queue.
  bool absorb_queued_gnss() {
    const int m0 = win_.master_fixes();
    const int r0 = win_.rover_fixes();
    struct Held {
      sensor_msgs::msg::NavSatFix msg;
      bool master;
      double t;
    };
    std::vector<Held> held;
    sensor_msgs::msg::NavSatFix msg;
    rclcpp::MessageInfo info;
    if (sub_gm_) {
      while (sub_gm_->take(msg, info)) held.push_back({msg, true, stamp_s(msg.header.stamp)});
    }
    if (sub_gr_) {
      while (sub_gr_->take(msg, info)) held.push_back({msg, false, stamp_s(msg.header.stamp)});
    }
    std::vector<railbreak::QueuedStamp> order;
    order.reserve(held.size());
    for (std::size_t i = 0; i < held.size(); ++i)
      order.push_back({held[i].master, held[i].t, i});
    order = railbreak::order_queued_stamps(std::move(order));
    for (const auto& s : order) {
      if (win_.closed) break;
      on_fix(held[s.index].msg, s.master);
    }
    return win_.closed || win_.master_fixes() != m0 || win_.rover_fixes() != r0;
  }

  void finish_init() {
    if (win_.closed) return;
    const bool master = !m_lat_.empty();
    const bool rover_only = !master && !r_lat_.empty();
    const double lat = master ? median(m_lat_) : median(r_lat_);
    const double lon = master ? median(m_lon_) : median(r_lon_);
    // Rover is a heading baseline only when master exists. Alone it is the
    // only absolute tie (one issued bag has rover and no master).
    const bool heading = master && !r_lat_.empty();
    const auto r = railbreak::init_on_ring(assets_.map, lat, lon, heading, median(r_lat_), median(r_lon_));
    if (!r.ok || !std::isfinite(lat)) {
      relative_ = true;
      close_gnss("start fix not on the track map; relative odometry");
      return;
    }
    // Rover is 12.436 m ahead of master. Absolute MGRS does not subtract the
    // antenna, so a rover-only snap is stepped back to the master; the along
    // offset then publishes base_link.
    // The window may have been opened by an earlier rover. s0 is the upper
    // median of the authoritative antenna, so the carried path starts at that
    // sample's stamp, not at the rover and not at the next wheel.
    double s0 = r.s0;
    if (rover_only) s0 -= rover_baseline_m_;
    const double t_epoch = median(master ? m_t_ : r_t_);
    const double s_epoch = railbreak::arc_at(arc_hist_, t_epoch);
    od_->init(railbreak::align_s(s0, od_->s(), s_epoch), std::max(r.d0, 0.5));
    // Start-relative frames subtract this point. Absolute mkrs and mgrs do not,
    // so their first sample is the map point in that grid, not (0, 0, 0).
    anchor_frame_to_output();
    initialised_ = true;
    n_fix_used_ = static_cast<int>(m_lat_.size() + r_lat_.size());
    const char* note = rover_only ? "initialised with rover only; no master"
                       : heading   ? "initialised with master+rover azimuth"
                                   : "initialised with master only";
    close_gnss(note);
    RCLCPP_INFO(get_logger(), "s0 %.3f m (snap %.3f), d0 %.2f m, %d fixes", s0, r.s0, r.d0, n_fix_used_);
  }

  // Origin of mkrs_start / grid_start / enu: the point the node publishes at s0,
  // including the along-track and vertical offsets. Not the GNSS median.
  void anchor_frame_to_output() {
    const auto& mp = assets_.map;
    const double s_pub = od_->s() + off_along_;
    double lat = 0.0, lon = 0.0;
    mp.latlon(mp.at(mp.x, s_pub), mp.at(mp.y, s_pub), lat, lon);
    frame_.set_start(lat, lon, mp.at(mp.h, s_pub) + off_up_);
  }

  // GNSS absent. initial_lat/lon snaps to the ring; initial_s_m is an arc length.
  // Both unset: caller publishes relative path.
  bool manual_start() {
    if (!assets_ok_ || assets_.map.empty()) return false;
    double s0 = 0.0;
    if (std::isfinite(initial_lat_) && std::isfinite(initial_lon_)) {
      const auto r = railbreak::init_on_ring(assets_.map, initial_lat_, initial_lon_, false, 0.0, 0.0);
      if (!r.ok) return false;
      s0 = r.s0;
    } else if (std::isfinite(initial_s_)) {
      s0 = initial_s_;
    } else {
      return false;
    }
    if (s_at_first_fix_ < -1e8) s_at_first_fix_ = od_->s();
    od_->init(s0 + (od_->s() - s_at_first_fix_), 1.0);
    anchor_frame_to_output();
    initialised_ = true;
    n_fix_used_ = 0;
    close_gnss("no fix; initial position from parameters");
    RCLCPP_INFO(get_logger(), "manual s0 %.1f m", s0);
    return true;
  }

  void close_gnss(const char* why) {
    win_.closed = true;
    sub_gm_.reset();
    sub_gr_.reset();
    gnss_note_ = why;
    RCLCPP_INFO(get_logger(), "GNSS unsubscribed: %s", why);
  }

  // --- main loop -----------------------------------------------------------
  void on_bogie(const VelocitySensor& m, bool front) {
    const auto t0 = std::chrono::steady_clock::now();
    const double t = stamp_s(m.header.stamp);
    if (!stamp_forward(t)) {
      if (have_out_ && std::isfinite(t) && t < t_out_) ++n_behind_out_;
      od_->on_bogie(t, front, m.velocity);
      note_integrity(true);
      publish_diag(m.header.stamp);
      return;
    }
    if (have_out_ && t == t_out_) ++n_dup_out_;
    touch(t);
    od_->on_bogie(t, front, m.velocity);
    note_integrity(false);
    publish(m.header.stamp, t0);
    if (front) ++n_pub_front_;
    else ++n_pub_rear_;
    note_out(t);
  }

  void on_cmd(const DriverControllerCommand& m) {
    const auto t0 = std::chrono::steady_clock::now();
    const double t = stamp_s(m.header.stamp);
    if (!stamp_forward(t)) {
      if (have_out_ && std::isfinite(t) && t < t_out_) ++n_behind_out_;
      od_->on_cmd(t, static_cast<int>(m.position));
      note_integrity(true);
      publish_diag(m.header.stamp);
      return;
    }
    if (have_out_ && t == t_out_) ++n_dup_out_;
    touch(t);
    od_->on_cmd(t, static_cast<int>(m.position));
    note_integrity(false);
    publish(m.header.stamp, t0);
    ++n_pub_cmd_;
    note_out(t);
  }

  // A stamp behind the last output is counted and dropped. It does not move the
  // GNSS window and it is not published, so header.stamp does not go backwards.
  // An equal stamp still passes: the two bogies can share one.
  bool stamp_forward(double t) const {
    return std::isfinite(t) && !(have_out_ && t < t_out_);
  }

  void note_out(double t) {
    t_out_ = t;
    have_out_ = true;
  }

  void touch(double t) {
    if (win_.t_first_input < 0.0) {
      win_.t_first_input = t;
      od_->set_time(t);
      s_rel_origin_ = od_->s();
    }
    if (!win_.closed) arc_hist_.push_back({t, od_->s()});
    on_window_input(t);
  }

  void out_point(double s, double& x, double& y, double& z) const {
    const auto& mp = assets_.map;
    double lat = 0.0, lon = 0.0;
    mp.latlon(mp.at(mp.x, s), mp.at(mp.y, s), lat, lon);
    frame_.to_out(lat, lon, mp.at(mp.h, s), x, y, z);
    z += off_up_;
  }

  void publish(const builtin_interfaces::msg::Time& stamp,
               std::chrono::steady_clock::time_point t_in) {
    VelocitySensor vel;
    vel.header.stamp = stamp;
    vel.header.frame_id = child_frame_id_;
    vel.velocity = od_->v();
    pub_v_->publish(vel);

    if (initialised_ || relative_) {
      nav_msgs::msg::Odometry o;
      o.header.stamp = stamp;
      o.header.frame_id = frame_id_;
      o.child_frame_id = child_frame_id_;
      const double s = od_->s() + (initialised_ ? off_along_ : 0.0);
      double yaw = 0.0;
      if (initialised_) {
        double x0, y0, x1, y1, zz;
        out_point(s, o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z);
        out_point(s - 2.0, x0, y0, zz);
        out_point(s + 2.0, x1, y1, zz);
        yaw = std::atan2(y1 - y0, x1 - x0);
      } else {
        double d = s - s_rel_origin_;
        if (assets_.map.ring_len > 0.0 && d < 0.0) d += assets_.map.ring_len;
        o.pose.pose.position.x = d;
      }
      o.pose.pose.orientation.z = std::sin(0.5 * yaw);
      o.pose.pose.orientation.w = std::cos(0.5 * yaw);
      const double vs = od_->sigma_s() * od_->sigma_s();
      railbreak::TrackOdometer::fill_pose_covariance(vs, o.pose.covariance.data());
      o.twist.twist.linear.x = od_->v();
      o.twist.covariance[0] = od_->sigma_v() * od_->sigma_v();
      for (int i = 7; i < 36; i += 7) o.twist.covariance[static_cast<std::size_t>(i)] = 1e6;
      pub_p_->publish(o);
    }

    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t_in).count();
    lat_max_us_ = std::max(lat_max_us_, us);
    if (++n_out_ % diag_every_ == 0) publish_diag(stamp);
  }

  void note_integrity(bool stamp_regressed) {
    last_integrity_ = integrity_.update(integrity_obs(stamp_regressed));
    have_integrity_ = true;
    last_adhesion_ = adhesion_.update(adhesion_obs());
    have_adhesion_ = true;
  }

  railbreak::AdhesionObs adhesion_obs() const {
    railbreak::AdhesionObs o;
    o.t = od_->have_time() ? od_->time_s() : 0.0;
    o.pair_fresh = od_->pair_fresh();
    o.bogies_agree = od_->bogies_agree();
    o.slip_front = od_->slip_front();
    o.slip_rear = od_->slip_rear();
    o.have_consensus = od_->wheel_consensus_have();
    o.wheel_consensus_residual = od_->wheel_consensus_residual();
    o.have_model = od_->model_consistency_have();
    o.model_consistency_residual = od_->model_consistency_residual();
    o.front_nis = od_->slip_front_nis();
    o.rear_nis = od_->slip_rear_nis();
    o.notch = od_->notch();
    o.speed = od_->v();
    return o;
  }

  railbreak::IntegrityObs integrity_obs(bool stamp_regressed) const {
    railbreak::IntegrityObs o;
    o.stamp_regressed = stamp_regressed;
    o.absolute_start = initialised_ && !relative_;
    o.map_in_domain = assets_ok_ &&
                      gnss_note_.find("not on the track map") == std::string::npos;
    o.nis_gate = od_->nis_gate();
    o.wheel_stale_s = od_->wheel_stale_s();
    o.max_gap_s = od_->max_gap_s();
    o.recover_s = od_->recover_s();
    o.n_anchor = od_->n_anchor();
    o.sigma_s = od_->sigma_s();
    o.t = od_->have_time() ? od_->time_s() : 0.0;
    o.dwell = od_->mode() == railbreak::Mode::kZupt;
    o.slip_front = od_->slip_front();
    o.slip_rear = od_->slip_rear();
    o.nis_front = od_->slip_front_nis();
    o.nis_rear = od_->slip_rear_nis();
    auto age = [&](bool have, double ts) {
      if (!have || !od_->have_time()) return 1.0e9;
      return std::max(0.0, od_->time_s() - ts);
    };
    o.front_age_s = age(od_->front_have(), od_->front_t());
    o.rear_age_s = age(od_->rear_have(), od_->rear_t());
    o.pair_fresh = od_->pair_fresh();
    o.bogies_agree = od_->bogies_agree();
    o.station_candidates = od_->station_candidates();
    return o;
  }

  void publish_diag(const builtin_interfaces::msg::Time& stamp) {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "railbreak_backup_odometry";
    st.hardware_id = "tram";
    st.level = (od_->mode() == railbreak::Mode::kFreeze || od_->slip_front() || od_->slip_rear())
                   ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                   : diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = railbreak::mode_name(od_->mode());
    auto kv = [&](const char* k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue x;
      x.key = k;
      x.value = v;
      st.values.push_back(x);
    };
    kv("mode", railbreak::mode_name(od_->mode()));
    kv("slip", od_->slip() ? "true" : "false");
    kv("slip_front", od_->slip_front() ? "true" : "false");
    kv("slip_rear", od_->slip_rear() ? "true" : "false");
    kv("slip_age_s", std::to_string(od_->slip_age_s()));
    kv("slip_front_run", std::to_string(od_->slip_front_run()));
    kv("slip_rear_run", std::to_string(od_->slip_rear_run()));
    kv("slip_front_nis", std::to_string(od_->slip_front_nis()));
    kv("slip_rear_nis", std::to_string(od_->slip_rear_nis()));
    kv("s_m", std::to_string(od_->s()));
    kv("sigma_s_m", std::to_string(od_->sigma_s()));
    kv("wheel_scale_k", std::to_string(od_->k()));
    kv("a_model_mps2", std::to_string(od_->a_model_now()));
    kv("notch", std::to_string(od_->notch()));
    kv("n_anchor", std::to_string(od_->n_anchor()));
    kv("n_guard", std::to_string(od_->n_guard()));
    kv("numerical_guard", od_->numerical_guard());
    kv("n_rejected", std::to_string(od_->n_rejected()));
    kv("n_pub_front", std::to_string(n_pub_front_));
    kv("n_pub_rear", std::to_string(n_pub_rear_));
    kv("n_pub_cmd", std::to_string(n_pub_cmd_));
    kv("n_dup_out", std::to_string(n_dup_out_));
    kv("n_behind_out", std::to_string(n_behind_out_));
    kv("gnss", win_.closed ? "closed" : "open");
    kv("gnss_note", gnss_note_);
    kv("gnss_fixes_used", std::to_string(n_fix_used_));
    kv("relative", relative_ ? "true" : "false");
    kv("output_frame", frame_.mode == railbreak::FrameMode::kMkrsStart   ? "mkrs_start"
                       : frame_.mode == railbreak::FrameMode::kMkrs      ? "mkrs"
                       : frame_.mode == railbreak::FrameMode::kMgrs      ? "mgrs"
                       : frame_.mode == railbreak::FrameMode::kGridStart ? "grid_start"
                                                                         : "enu");
    kv("assets", assets_ok_ ? "loaded" : "missing");
    kv("rear_front_ratio", std::to_string(od_->rear_front_ratio()));
    kv("bogie_noise_sd_mps", std::to_string(od_->noise_sd()));
    kv("model_bias_mps2", std::to_string(od_->model_bias()));
    kv("callback_max_us", std::to_string(lat_max_us_));
    const railbreak::IntegrityReport ir = have_integrity_
                                               ? last_integrity_
                                               : integrity_.update(integrity_obs(false));
    kv("integrity_status", ir.status);
    kv("integrity_reasons", ir.reasons.empty() ? "" : ir.reasons);
    if (ir.bound_valid) {
      std::ostringstream bound;
      bound.setf(std::ios::fixed);
      bound.precision(3);
      bound << ir.along_bound_m;
      kv("integrity_along_bound_m", bound.str());
    } else {
      kv("integrity_along_bound_m", "null");
    }
    kv("integrity_calibrated_coverage", ir.calibrated_coverage);
    kv("integrity_calibration_split", ir.calibration_split);
    kv("integrity_certification_claim", "false");
    kv("integrity_use_position", ir.use_position ? "true" : "false");
    kv("integrity_bound_name", ir.bound_name);
    const railbreak::AdhesionReport ar = have_adhesion_
                                              ? last_adhesion_
                                              : adhesion_.update(adhesion_obs());
    auto num = [](bool have, double value) {
      if (!have || !std::isfinite(value)) return std::string("null");
      std::ostringstream out;
      out.setf(std::ios::fixed);
      out.precision(4);
      out << value;
      return out.str();
    };
    kv("adhesion_proxy", ar.json());
    kv("adhesion_classification", ar.classification);
    kv("adhesion_mu_estimate", "null");
    kv("adhesion_reason", ar.reason);
    kv("adhesion_duration_s", num(true, ar.duration_s));
    kv("adhesion_wheel_consensus_residual", num(ar.have_consensus, ar.wheel_consensus_residual));
    kv("adhesion_model_consistency_residual", num(ar.have_model, ar.model_consistency_residual));
    kv("adhesion_front_nis", num(true, ar.front_nis));
    kv("adhesion_rear_nis", num(true, ar.rear_nis));
    kv("adhesion_common_mode_duration_s", num(true, ar.common_mode_duration_s));
    kv("adhesion_notch", std::to_string(ar.notch));
    kv("adhesion_speed_mps", num(true, ar.speed));
    a.status.push_back(st);
    pub_d_->publish(a);
  }

  railbreak::Assets assets_;
  bool assets_ok_ = true;
  std::unique_ptr<railbreak::TrackOdometer> od_;
  railbreak::IntegrityMonitor integrity_{railbreak::empirical_bound()};
  railbreak::IntegrityReport last_integrity_{};
  bool have_integrity_ = false;
  railbreak::AdhesionProxy adhesion_{};
  railbreak::AdhesionReport last_adhesion_{};
  bool have_adhesion_ = false;
  railbreak::GnssWindow win_;
  int64_t diag_every_ = 20;
  std::string frame_id_, child_frame_id_;

  double s_at_first_fix_ = -1e9, s_rel_origin_ = 0.0;
  std::vector<railbreak::ArcMark> arc_hist_;
  std::vector<double> m_lat_, m_lon_, m_alt_, m_t_, r_lat_, r_lon_, r_alt_, r_t_;
  bool initialised_ = false, relative_ = false;
  bool drain_gnss_queue_ = true;
  bool have_out_ = false;
  double t_out_ = 0.0;
  std::string gnss_note_ = "window open";
  int n_fix_used_ = 0;
  railbreak::OutputFrame frame_;
  double off_along_ = 9.873, off_up_ = -3.0, rover_baseline_m_ = 12.436;
  double initial_s_ = std::numeric_limits<double>::quiet_NaN();
  double initial_lat_ = std::numeric_limits<double>::quiet_NaN();
  double initial_lon_ = std::numeric_limits<double>::quiet_NaN();
  double wheel_r_ = 0.0, wheel_r0_ = 0.0;
  double lat_max_us_ = 0.0;
  int64_t n_out_ = 0;
  int64_t n_pub_front_ = 0, n_pub_rear_ = 0, n_pub_cmd_ = 0;
  int64_t n_dup_out_ = 0, n_behind_out_ = 0;

  rclcpp::Publisher<VelocitySensor>::SharedPtr pub_v_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_p_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_d_;
  rclcpp::Subscription<VelocitySensor>::SharedPtr sub_f_, sub_r_;
  rclcpp::Subscription<DriverControllerCommand>::SharedPtr sub_c_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gm_, sub_gr_;

  friend struct GnssDrainProbe;
};

#ifndef RAILBREAK_ODOMETRY_NO_MAIN
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BackupOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
#endif

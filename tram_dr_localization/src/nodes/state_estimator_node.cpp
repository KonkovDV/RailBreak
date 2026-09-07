#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"

#include "tram_dr_localization/timebase.hpp"
#include "tram_dr_localization/ukf.hpp"

// UNINITIALIZED until notch or wheel_odom. No Imu / NavSatFix / PointCloud2.
// Predict is event-driven (wheel callback). Timer is watchdog + publish.
// Always publish (loss of wheels must not silence the topic).
//
// Timebase contract (F-17b)
// -------------------------
// Two clocks are in play here and they are NEVER subtracted from one another:
//
//   * the measurement timebase - header.stamp on joint_state / twist_stamped.
//     It defines the interval between two sensor samples, i.e. dt.
//   * this node's clock - now(). It defines how long ago something arrived,
//     i.e. every freshness/staleness age.
//
// The previous implementation kept one last_step_ and filled it from
// whichever clock happened to produce the current frame, then differenced
// across the two. On a bag replayed without use_sim_time:=true the offset
// between those clocks is hours or years, so dt alternated between a huge
// positive and a huge negative number and the clamp turned both into
// plausible 200 ms / 5 ms steps: the filter integrated at the wrong rate in
// complete silence. The same mixing disabled the notch and wheel freshness
// watchdogs, which are what the integrity case actually rests on.
//
// Timer-driven predict-only steps carry no header stamp, so they are measured
// with a node-clock delta and the filter time they consume is accumulated in
// consumed_since_meas_s_. The next stamped frame subtracts that amount, so the
// filter advances along a single monotone timeline with no double counting and
// no cross-clock arithmetic.
//
// dt is validated, never clamped into plausibility. Regressions are rejected,
// long gaps are advanced as repeated predict-only steps so Q accumulates over
// the true elapsed time, and every fabricated microsecond is counted and
// published.

class StateEstimatorNode : public rclcpp::Node {
 public:
  StateEstimatorNode() : Node("state_estimator") {
    const double rate_hz = declare_parameter("rate_hz", 50.0);
    tram_dr::UkfParams cfg;
    cfg.alpha = declare_parameter("ukf_alpha", 0.58);
    cfg.beta = declare_parameter("ukf_beta", 2.0);
    cfg.kappa_ut = declare_parameter("ukf_kappa", 0.0);
    cfg.kappa_cut = declare_parameter("kappa_cut", 0.25);
    cfg.r_common_mode = declare_parameter("r_common_mode", 80.0);
    cfg.p_ss_init = declare_parameter("p_ss_init", 0.25);
    cfg.k_sigma = declare_parameter("k_sigma", 2.0);
    cfg.k_over = declare_parameter("k_over", 2.5);
    cfg.freeze_s = declare_parameter("freeze_s", 0.5);
    cfg.cubature = declare_parameter("ukf_cubature", false);
    cfg.kappa_hold_s = declare_parameter("kappa_hold_s", 0.2);
    cfg.k_lost = declare_parameter("k_lost", 2.5);
    cfg.notch_lost_s = declare_parameter("notch_lost_s", 2.0);
    cfg.allow_reverse = declare_parameter("allow_reverse", false);
    cfg.mass_door_kg = declare_parameter("mass_door_kg", 3000.0);
    cfg.q_v = declare_parameter("q_v", 0.0025);
    cfg.zupt_hold_s = declare_parameter("zupt_hold_s", 0.3);
    cfg.encoder_pulses_per_rev = declare_parameter("encoder_pulses_per_rev", 0);
    cfg.huber_c = declare_parameter("huber_c", 3.0);
    cfg.slide_grade_lost_s = declare_parameter("slide_grade_lost_s", 0.4);
    cfg.a_kin_downhill = declare_parameter("a_kin_downhill", 0.05);
    cfg.mass_prior_log_sigma = declare_parameter("mass_prior_log_sigma", 0.3);
    // F-23: the mass prior is a Gauss-Markov process, not a repeatedly fused
    // pseudo-measurement, so its time constant is a real knob. Without this
    // declaration it silently kept the 300 s default and could not be tuned
    // from config/estimator.yaml like everything else here.
    cfg.mass_prior_tau_s = declare_parameter("mass_prior_tau_s", 300.0);
    cfg.q_fb_wheels_n = declare_parameter("q_fb_wheels_n", 3000.0);
    cfg.stop_gate_m = declare_parameter("stop_gate_m", 40.0);
    cfg.r0_uncalibrated = declare_parameter("r0_uncalibrated", false);
    cfg.age_degraded_s = declare_parameter("age_degraded_s", 0.25);
    cfg.age_lost_s = declare_parameter("age_lost_s", 1.0);
    cfg.zupt_omega_only_s = declare_parameter("zupt_omega_only_s", 2.0);
    n_wheels_param_ = static_cast<std::size_t>(std::clamp(
        static_cast<int>(declare_parameter("n_wheels", 4)), 1, tram_dr::kNWheels));
    cfg.n_wheels = static_cast<int>(n_wheels_param_);
    const auto stops = declare_parameter("route_s_m", std::vector<double>{});
    cfg.n_stops = 0;
    for (double s : stops) {
      if (cfg.n_stops >= tram_dr::kMaxStops) {
        break;
      }
      cfg.stop_s_m[static_cast<std::size_t>(cfg.n_stops++)] = s;
    }
    ukf_.set_params(cfg);
    // Kept for diagnostics: a protection level published without the k that
    // produced it has no stated confidence level.
    k_sigma_ = cfg.k_sigma;
    k_over_ = cfg.k_over;
    huber_c_ = cfg.huber_c;
    mass_prior_log_sigma_ = cfg.mass_prior_log_sigma;
    mass_prior_tau_s_ = cfg.mass_prior_tau_s;

    tram_dr::PlantParams plant = tram_dr::default_plant_params();
    plant.r0_m = declare_parameter("wheel_radius_m", 0.35);
    plant.m0_kg = declare_parameter("mass_kg", 28000.0);
    plant.a_trac_max = declare_parameter("a_trac_max", 1.3);
    plant.a_svc = declare_parameter("a_svc", 1.2);
    plant.v_base_mps = declare_parameter("v_base_mps", 6.0);
    plant.A_d = declare_parameter("A_d", 800.0);
    plant.B_d = declare_parameter("B_d", 40.0);
    plant.C_d = declare_parameter("C_d", 6.0);
    plant.tau_drv_s = declare_parameter("tau_drv_s", 0.0);
    plant.gamma_rot = declare_parameter("gamma_rot", 0.0);
    plant.brake_nonadhesive_frac = declare_parameter("brake_nonadhesive_frac", 0.0);
    plant.notch_as_accel = declare_parameter("notch_as_accel", false);
    plant.i_grade = declare_parameter("i_grade", 0.0);
    plant.j_max_mps3 = declare_parameter("j_max_mps3", 0.0);
    plant.mass_min_kg = declare_parameter("mass_min_kg", 20000.0);
    plant.mass_max_kg = declare_parameter("mass_max_kg", 70000.0);
    r0_m_ = plant.r0_m;
    ukf_.set_plant(plant);

    vehicle_profile_ = declare_parameter("vehicle_profile", std::string("combino_nf100"));
    wheel_latency_s_ = declare_parameter("wheel_latency_s", 0.0);

    tram_dr::ScaParams sca;
    sca.r0_m = plant.r0_m;
    sca.z_thresh = declare_parameter("sca_z_thresh", 2.5);
    sca.pair_lr = declare_parameter("sca_pair_lr", true);
    sca.sigma_v_rel = declare_parameter("sca_sigma_v_rel", 0.0);
    sca.sigma_v_mps = declare_parameter("sca_sigma_v_mps", 0.15);
    const auto roles = declare_parameter("axle_role", std::vector<int64_t>{});
    for (int i = 0; i < tram_dr::kNWheels; ++i) {
      sca.axle_role[static_cast<std::size_t>(i)] = 0;
      if (i < static_cast<int>(roles.size()) && roles[static_cast<std::size_t>(i)] != 0) {
        sca.axle_role[static_cast<std::size_t>(i)] = tram_dr::kAxleTrailer;
      }
    }
    ukf_.set_sca(sca);

    notch_max_abs_ = declare_parameter("notch_max_abs", 8.0);
    age_degraded_s_ = cfg.age_degraded_s;
    age_lost_s_ = cfg.age_lost_s;
    twist_is_omega_ = declare_parameter("twist_is_omega", false);
    last_u_.brake_valid = false;
    last_u_.notch_valid = false;

    // --- time hygiene parameters (F-17b) ---------------------------------
    // notch_stale_s stays well below UkfParams::notch_lost_s so the core owns
    // the documented LOST delay; this is only the "stop trusting the value"
    // threshold.
    notch_stale_s_ = declare_parameter("notch_stale_s", 0.25);
    brake_stale_s_ = declare_parameter("brake_stale_s", 0.25);
    default_dt_s_ = declare_parameter("default_dt_s", 0.02);
    // 1 ms, not the old 5 ms: a 5 ms floor makes any source above 200 Hz
    // integrate faster than real time.
    dt_min_s_ = declare_parameter("dt_min_s", 0.001);
    dt_max_s_ = declare_parameter("dt_max_s", 0.20);
    stamp_regression_tol_s_ = declare_parameter("stamp_regression_tol_s", 0.001);
    stamp_skew_warn_s_ = declare_parameter("stamp_skew_warn_s", 5.0);
    // 18000 * dt_max_s = 1 h of catch-up. A 20 s dropout must still accumulate
    // Q over the true gap; beyond an hour the solution is meaningless and we
    // fail closed rather than spin.
    max_catchup_steps_ = std::max(
        1, static_cast<int>(declare_parameter("max_catchup_steps", 18000)));
    if (!(dt_min_s_ > 0.0) || !(dt_max_s_ > dt_min_s_) || !std::isfinite(dt_min_s_) ||
        !std::isfinite(dt_max_s_)) {
      throw std::invalid_argument("require 0 < dt_min_s < dt_max_s");
    }
    if (!std::isfinite(default_dt_s_) || default_dt_s_ < dt_min_s_ ||
        default_dt_s_ > dt_max_s_) {
      throw std::invalid_argument("require dt_min_s <= default_dt_s <= dt_max_s");
    }

    const std::string notch_type = declare_parameter("notch_type", std::string("float32"));
    const std::string wheels_type = declare_parameter("wheels_type", std::string("float64_array"));
    if (notch_type != "float32" && notch_type != "float64" && notch_type != "int8" &&
        notch_type != "int16") {
      throw std::invalid_argument(
          "notch_type must be float32|float64|int8|int16, got " + notch_type);
    }
    const std::string notch_enc = declare_parameter("notch_encoding", std::string("auto"));
    if (notch_enc != "auto" && notch_enc != "normalized" && notch_enc != "discrete") {
      throw std::invalid_argument(
          "notch_encoding must be auto|normalized|discrete, got " + notch_enc);
    }
    if (notch_enc == "discrete" ||
        ((notch_type == "int8" || notch_type == "int16") && notch_enc != "normalized")) {
      notch_enc_ = tram_dr::NotchEncoding::kDiscrete;
    } else if (notch_enc == "normalized") {
      notch_enc_ = tram_dr::NotchEncoding::kNormalized;
    } else {
      notch_enc_ = tram_dr::NotchEncoding::kAuto;
    }
    if (wheels_type != "float64_array" && wheels_type != "float32_array" &&
        wheels_type != "joint_state" && wheels_type != "twist_stamped") {
      throw std::invalid_argument(
          "wheels_type must be float64_array|float32_array|joint_state|twist_stamped, got " +
          wheels_type);
    }

    in_qos_ = rclcpp::QoS(1).best_effort();

    pub_ = create_publisher<nav_msgs::msg::Odometry>("/tram/state_estimate", rclcpp::QoS(1).reliable());
    diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/tram/diagnostics", 10);
    subscribe_notch(notch_type);
    sub_brake_ = create_subscription<std_msgs::msg::Float32>(
        "/tram/brake_cmd", in_qos_,
        [this](const std_msgs::msg::Float32& msg) {
          const double b = static_cast<double>(msg.data);
          // std::clamp(NaN, 0.0, 1.0) returns NaN: both comparisons are
          // false, so the value passes through untouched. A clamp is not an
          // input filter (F-17).
          if (!std::isfinite(b)) {
            ++bad_brake_;
            input_fault_ = true;
            return;
          }
          last_u_.brake = std::clamp(b, 0.0, 1.0);
          last_u_.brake_valid = true;
          have_brake_ = true;
          last_brake_arrival_ = now();
        });
    subscribe_wheels(wheels_type);
    RCLCPP_INFO(get_logger(),
                "state_estimator notch_type=%s wheels_type=%s model=%s vehicle=%s n=%zu "
                "stops=%d (mass_door gated)",
                notch_type.c_str(), wheels_type.c_str(), tram_dr::kModelVersion,
                vehicle_profile_.c_str(), n_wheels_param_, cfg.n_stops);
    const int period_ms =
        std::max(1, static_cast<int>(std::lround(1000.0 / std::max(rate_hz, 1.0))));
    period_s_ = 1.0 / std::max(rate_hz, 1.0);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() { tick(); });
  }

 private:
  void apply_notch(double raw) {
    // F-17: a non-finite command must never be promoted to a valid input.
    // map_notch() propagates NaN, and notch_valid = true then tells the core
    // "the controller position is known" while handing it NaN. That poisons
    // the traction force and therefore the entire predict step, and it does
    // so while the diagnostics still report a fresh, valid notch.
    if (!std::isfinite(raw)) {
      ++bad_notch_;
      input_fault_ = true;
      last_u_.notch_valid = false;
      return;
    }
    last_u_.notch = tram_dr::map_notch(raw, notch_max_abs_, notch_enc_);
    last_u_.notch_valid = true;
    have_notch_ = true;
    // Arrival freshness is a property of THIS node's clock, never of the
    // publisher's header stamp. Mixing the two was the defect (F-17b).
    last_notch_arrival_ = now();
  }

  void apply_wheels(const double* data, std::size_t n, const rclcpp::Time* stamp = nullptr) {
    n_omega_nonfinite_ = 0;
    if (n == 0) {
      n_omega_ = 0;
      have_wheels_ = false;
      return;
    }
    // A short JointState / MultiArray is not an n-axle vehicle. Pad missing
    // channels with NaN so SCA inflates them and ZUPT cannot fire on one zero.
    n_omega_ = n_wheels_param_;
    for (std::size_t i = 0; i < n_wheels_param_; ++i) {
      if (i < n) {
        omega_[i] = data[i];
      } else {
        omega_[i] = std::numeric_limits<double>::quiet_NaN();
      }
      if (!std::isfinite(omega_[i])) {
        ++n_omega_nonfinite_;
      }
    }
    have_wheels_ = true;
    const rclcpp::Time arrival = now();
    last_wheel_arrival_ = arrival;
    if (stamp != nullptr && stamp->nanoseconds() > 0) {
      note_stamp_skew(*stamp, arrival);
    }
    step_filter(true, stamp);
  }

  void apply_twist_vx(double vx, const rclcpp::Time* stamp = nullptr) {
    // A malformed twist is not the same thing as a dead encoder. Synthesising
    // a full NaN wheel frame from it would be indistinguishable from a real
    // all-NaN encoder frame, which the core deliberately treats as data.
    if (!std::isfinite(vx)) {
      ++bad_wheels_;
      input_fault_ = true;
      return;
    }
    const double w = twist_is_omega_ ? vx : vx / std::max(r0_m_, 1e-6);
    if (!std::isfinite(w)) {
      ++bad_wheels_;
      input_fault_ = true;
      return;
    }
    // One body-speed sample is one encoder, not n agreeing axles. Copying w
    // onto every channel invented SCA consensus and let a single zero ZUPT.
    // apply_wheels pads the rest with NaN (incomplete packet → not OK).
    double tmp[tram_dr::kNWheels];
    tmp[0] = w;
    apply_wheels(tmp, 1, stamp);
  }

  void subscribe_notch(const std::string& notch_type) {
    if (notch_type == "int8") {
      sub_notch_i8_ = create_subscription<std_msgs::msg::Int8>(
          "/tram/controller_notch", in_qos_,
          [this](const std_msgs::msg::Int8& msg) { apply_notch(static_cast<double>(msg.data)); });
    } else if (notch_type == "int16") {
      sub_notch_i16_ = create_subscription<std_msgs::msg::Int16>(
          "/tram/controller_notch", in_qos_,
          [this](const std_msgs::msg::Int16& msg) { apply_notch(static_cast<double>(msg.data)); });
    } else if (notch_type == "float64") {
      sub_notch_f64_ = create_subscription<std_msgs::msg::Float64>(
          "/tram/controller_notch", in_qos_,
          [this](const std_msgs::msg::Float64& msg) { apply_notch(msg.data); });
    } else {
      sub_notch_f32_ = create_subscription<std_msgs::msg::Float32>(
          "/tram/controller_notch", in_qos_,
          [this](const std_msgs::msg::Float32& msg) {
            apply_notch(static_cast<double>(msg.data));
          });
    }
  }

  void subscribe_wheels(const std::string& wheels_type) {
    if (wheels_type == "float32_array") {
      sub_wheels_f32_ = create_subscription<std_msgs::msg::Float32MultiArray>(
          "/tram/wheel_odom", in_qos_,
          [this](const std_msgs::msg::Float32MultiArray& msg) {
            const std::size_t n = std::min(msg.data.size(), n_wheels_param_);
            double tmp[tram_dr::kNWheels]{};
            for (std::size_t i = 0; i < n; ++i) {
              tmp[i] = static_cast<double>(msg.data[i]);
            }
            apply_wheels(tmp, n);
          });
    } else if (wheels_type == "joint_state") {
      sub_wheels_js_ = create_subscription<sensor_msgs::msg::JointState>(
          "/tram/wheel_odom", in_qos_,
          [this](const sensor_msgs::msg::JointState& msg) {
            const std::size_t n = std::min(msg.velocity.size(), n_wheels_param_);
            double tmp[tram_dr::kNWheels]{};
            for (std::size_t i = 0; i < n; ++i) {
              tmp[i] = msg.velocity[i];
            }
            const rclcpp::Time stamp(msg.header.stamp.sec, msg.header.stamp.nanosec,
                                     now().get_clock_type());
            apply_wheels(tmp, n, &stamp);
          });
    } else if (wheels_type == "twist_stamped") {
      sub_wheels_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "/tram/wheel_odom", in_qos_,
          [this](const geometry_msgs::msg::TwistStamped& msg) {
            const rclcpp::Time stamp(msg.header.stamp.sec, msg.header.stamp.nanosec,
                                     now().get_clock_type());
            apply_twist_vx(msg.twist.linear.x, &stamp);
          });
    } else {
      sub_wheels_f64_ = create_subscription<std_msgs::msg::Float64MultiArray>(
          "/tram/wheel_odom", in_qos_,
          [this](const std_msgs::msg::Float64MultiArray& msg) {
            apply_wheels(msg.data.data(), msg.data.size());
          });
    }
  }

  diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value) const {
    diagnostic_msgs::msg::KeyValue out;
    out.key = key;
    out.value = value;
    return out;
  }

  // One-shot operator warning when the measurement timebase and the node
  // clock are far apart. The usual cause is replaying a recorded bag without
  // use_sim_time:=true - exactly the configuration in which the old code
  // silently differenced the two clocks against each other.
  void note_stamp_skew(const rclcpp::Time& stamp, const rclcpp::Time& arrival) {
    const double skew = (arrival - stamp).seconds();
    if (!std::isfinite(skew)) {
      return;
    }
    stamp_skew_s_ = skew;
    if (std::fabs(skew) < stamp_skew_warn_s_) {
      return;
    }
    ++stamp_skew_events_;
    if (!warned_skew_) {
      warned_skew_ = true;
      RCLCPP_WARN(get_logger(),
                  "header.stamp is %.3f s away from this node's clock. dt is taken "
                  "from header stamps and freshness from the node clock, so both "
                  "remain correct - but if this is a bag replay, launch with "
                  "use_sim_time:=true so the two agree.",
                  skew);
    }
  }

  // Snapshot the control input and apply the notch freshness watchdog. Both
  // sides of the age comparison are this node's clock.
  tram_dr::Input resolve_input(const rclcpp::Time& wall) {
    tram_dr::Input u = last_u_;
    if (!have_notch_) {
      u.notch_valid = false;
    } else if (last_notch_arrival_.has_value()) {
      const double notch_age = (wall - *last_notch_arrival_).seconds();
      if (!std::isfinite(notch_age) || notch_age < 0.0 || notch_age > notch_stale_s_) {
        u.notch_valid = false;
        last_u_.notch_valid = false;
      }
    }
    // RB08-06: never skip brake age because notch was missing. Unknown brake
    // is not an observed zero.
    if (!have_brake_) {
      u.brake_valid = false;
    } else if (last_brake_arrival_.has_value()) {
      const double brake_age = (wall - *last_brake_arrival_).seconds();
      if (!std::isfinite(brake_age) || brake_age < 0.0 || brake_age > brake_stale_s_) {
        u.brake_valid = false;
        last_u_.brake_valid = false;
      }
    }
    return u;
  }

  void step_filter(bool fresh_wheels, const rclcpp::Time* meas_stamp = nullptr) {
    const rclcpp::Time wall = now();
    const bool use_meas = (meas_stamp != nullptr && meas_stamp->nanoseconds() > 0);
    timebase_is_meas_ = use_meas;

    double dt = default_dt_s_;

    if (use_meas) {
      const double stamp_s = meas_stamp->seconds();
      const bool have = last_meas_stamp_.has_value();
      const double last_s = have ? last_meas_stamp_->seconds() : 0.0;
      const tram_dr::MeasDt dec = tram_dr::stamped_interval(
          have, last_s, stamp_s, consumed_since_meas_s_, dt_min_s_,
          stamp_regression_tol_s_);
      if (dec.kind == tram_dr::MeasDt::Kind::kAnchor) {
        last_meas_stamp_ = *meas_stamp;
        consumed_since_meas_s_ = 0.0;
        last_wall_ = wall;
        return;
      }
      if (dec.kind == tram_dr::MeasDt::Kind::kReject) {
        ++stamp_regressions_;
        input_fault_ = true;
        last_wall_ = wall;
        return;
      }
      if (dec.kind == tram_dr::MeasDt::Kind::kSkip) {
        ++dt_floored_;
        last_wall_ = wall;
        return;
      }
      dt = dec.dt;
      last_meas_stamp_ = *meas_stamp;
      consumed_since_meas_s_ = 0.0;
    } else if (last_wall_.has_value()) {
      dt = (wall - *last_wall_).seconds();
    }

    last_wall_ = wall;

    if (!std::isfinite(dt)) {
      ++stamp_regressions_;
      input_fault_ = true;
      return;
    }

    double consumed = 0.0;
    if (dt > dt_max_s_) {
      const double needed = std::floor(dt / dt_max_s_);
      if (!std::isfinite(needed) || needed > static_cast<double>(max_catchup_steps_)) {
        ++dt_gaps_;
        input_fault_ = true;
        return;
      }
      ++dt_gaps_;
      // Same contract as replay_ukf: hold steps while dt > dt_max so leftover
      // stays in (0, dt_max]. floor() leftover 0 would drop this stamp's wheels.
      while (dt > dt_max_s_) {
        const tram_dr::Input ug = resolve_input(wall);
        last_e_ = ukf_.predict_and_update(ug, nullptr, 0, dt_max_s_);
        have_estimate_ = true;
        ++catchup_steps_;
        consumed += dt_max_s_;
        dt -= dt_max_s_;
      }
    }

    if (dt < dt_min_s_) {
      if (!use_meas) {
        // Timer leftover: bank, do not invent dt_min.
        ++dt_floored_;
        consumed_since_meas_s_ += std::max(dt, 0.0);
        return;
      }
      if (dt <= 0.0) {
        ++dt_floored_;
        return;
      }
      // Measurement leftover after catch-up: apply the true remainder.
    }

    last_dt_s_ = dt;
    consumed += dt;
    last_step_used_wheels_ = fresh_wheels;
    const std::size_t n = (fresh_wheels && have_wheels_) ? n_omega_ : 0;
    const tram_dr::Input u = resolve_input(wall);
    last_e_ = ukf_.predict_and_update(u, n > 0 ? omega_.data() : nullptr, n, dt);
    have_estimate_ = true;
    if (fresh_wheels) {
      input_fault_ = false;
    }

    if (!use_meas) {
      // Remember how much filter time this timer-driven step consumed so the
      // next stamped frame can subtract it.
      consumed_since_meas_s_ += consumed;
    }
  }

  void publish_state(const rclcpp::Time& t) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = t;
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";
    msg.pose.covariance.fill(0.0);
    msg.twist.covariance.fill(0.0);

    diagnostic_msgs::msg::DiagnosticArray d;
    d.header.stamp = msg.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "tram_dr";

    // Arrival latency: node clock on both sides of the subtraction (F-17b).
    const double age_s = last_wheel_arrival_.has_value()
                             ? std::max(0.0, (t - *last_wheel_arrival_).seconds())
                             : 1.0e9;
    const bool fault = input_fault_;

    if (!have_estimate_) {
      msg.pose.covariance[0] = 1.0e6;
      msg.twist.covariance[0] = 1.0e6;
      pub_->publish(msg);
      st.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      st.message = "UNINITIALIZED — waiting for /tram/wheel_odom";
      st.values.push_back(kv("age_s", std::to_string(age_s)));
      st.values.push_back(kv("model_version", tram_dr::kModelVersion));
      st.values.push_back(kv("vehicle_profile", vehicle_profile_));
      st.values.push_back(kv("n_wheels", std::to_string(n_wheels_param_)));
      push_time_hygiene(st);
      d.status.push_back(st);
      diag_->publish(d);
      return;
    }

    tram_dr::UkfEstimate e = last_e_;
    tram_dr::Confidence conf = e.confidence;
    tram_dr::Confidence conf_v = e.confidence_v;
    tram_dr::Confidence conf_s = e.confidence_s;
    if (e.initialized && age_s > age_lost_s_) {
      conf = tram_dr::Confidence::kLost;
    } else if (e.initialized && age_s > age_degraded_s_ && conf == tram_dr::Confidence::kOk) {
      conf = tram_dr::Confidence::kDegraded;
    }
    if (fault) {
      conf = tram_dr::Confidence::kLost;
    }
    if (conf == tram_dr::Confidence::kLost) {
      conf_v = tram_dr::Confidence::kLost;
      conf_s = tram_dr::Confidence::kLost;
    }

    msg.pose.pose.position.x = e.x.s_m;
    msg.twist.twist.linear.x = e.x.v_mps;
    msg.pose.covariance[0] =
        (conf == tram_dr::Confidence::kLost) ? std::max(e.p_ss, 50.0 * 50.0) : e.p_ss;
    msg.twist.covariance[0] = e.p_vv;
    pub_->publish(msg);

    switch (conf) {
      case tram_dr::Confidence::kOk:
        st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
      case tram_dr::Confidence::kDegraded:
        st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
      case tram_dr::Confidence::kLost:
        st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
      default:
        st.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        break;
    }
    st.message = std::string(tram_dr::confidence_label(conf)) + " mode=" +
                  tram_dr::mode_label(e.mode);
    if (fault) {
      st.message += " (input/timing fault)";
    }
    st.values.push_back(kv("confidence", tram_dr::confidence_label(conf)));
    st.values.push_back(kv("confidence_v", tram_dr::confidence_label(conf_v)));
    st.values.push_back(kv("confidence_s", tram_dr::confidence_label(conf_s)));
    st.values.push_back(kv("chol_fail", std::to_string(e.chol_fail)));
    st.values.push_back(kv("s_unobserved_s", std::to_string(e.s_unobserved_s)));
    st.values.push_back(kv("n_omega_used", std::to_string(e.n_omega_used)));
    st.values.push_back(kv("r0_m", std::to_string(r0_m_)));
    st.values.push_back(kv("model_version", tram_dr::kModelVersion));
    st.values.push_back(kv("vehicle_profile", vehicle_profile_));
    st.values.push_back(kv("n_wheels", std::to_string(n_wheels_param_)));
    st.values.push_back(kv("mode", tram_dr::mode_label(e.mode)));
    st.values.push_back(kv("wheel_latency_s", std::to_string(wheel_latency_s_)));
    st.values.push_back(kv("age_s", std::to_string(age_s)));
    st.values.push_back(kv("wheels_fresh", last_step_used_wheels_ ? "1" : "0"));
    st.values.push_back(kv("common_mode", e.sca.common_mode ? "1" : "0"));
    st.values.push_back(kv("slip_latched", e.slip_latched ? "1" : "0"));
    st.values.push_back(kv("n_inflated", std::to_string(e.sca.n_inflated)));
    st.values.push_back(kv("n_frozen", std::to_string(e.n_frozen)));
    st.values.push_back(kv("n_slip_axles", std::to_string(e.n_slip_axles)));
    st.values.push_back(kv("relative_wheel_slide", std::to_string(e.relative_wheel_slide)));
    st.values.push_back(kv("path_disagree_m", std::to_string(e.path_disagree_m)));
    st.values.push_back(kv("path_disagree_latched", e.path_disagree_latched ? "1" : "0"));
    st.values.push_back(kv("path_integrity_latched", e.path_integrity_latched ? "1" : "0"));
    st.values.push_back(kv("zupt_at_stop", e.zupt_at_stop ? "1" : "0"));
    st.values.push_back(kv("zupt_forced", e.zupt_forced ? "1" : "0"));
    st.values.push_back(kv("s_unbounded", e.s_unbounded ? "1" : "0"));
    st.values.push_back(kv("sca_current", e.sca_current ? "1" : "0"));
    st.values.push_back(kv("v_chan_a", std::to_string(e.v_chan_a_mps)));
    st.values.push_back(kv("nis", e.nis_valid ? std::to_string(e.nis) : "nan"));
    st.values.push_back(kv("nis_valid", e.nis_valid ? "1" : "0"));
    st.values.push_back(kv("nis_cusum", std::to_string(e.nis_cusum)));
    st.values.push_back(kv("over_m", e.s_unbounded ? "unbounded" : std::to_string(e.over_m)));
    st.values.push_back(kv("under_m", std::to_string(e.under_m)));
    st.values.push_back(kv("b_s_m", e.s_unbounded ? "unbounded" : std::to_string(e.b_s_m)));
    st.values.push_back(kv("pl_s_m", e.s_unbounded ? "unbounded" : std::to_string(e.pl_s_m)));
    st.values.push_back(kv("pl_v_mps", std::to_string(e.pl_v_mps)));
    st.values.push_back(kv("al_s_m", std::to_string(e.al_s_m)));
    push_integrity_inputs(st, e);
    st.values.push_back(kv("a_kin", std::to_string(e.a_kin_mps2)));
    st.values.push_back(kv("a_unphysical", e.a_unphysical ? "1" : "0"));
    push_time_hygiene(st);
    d.status.push_back(st);
    diag_->publish(d);
  }

  // The terms a protection level is built from, so a reviewer can recompute
  // it from the topic instead of trusting it.
  //
  // PL_s = k_over * sigma_s + b_s, and the integrity risk of that bound is
  // set by k alone: 2.5 gives 1.24e-02 per epoch, 3.0 gives 2.70e-03, and
  // about 6.0 is needed for the 2e-9/h order of a SIL-4 THR. Publishing PL
  // while withholding k asks the reader to trust a bound whose confidence
  // level is unstated. See docs/integrity-risk.md.
  void push_integrity_inputs(diagnostic_msgs::msg::DiagnosticStatus& st,
                             const tram_dr::UkfEstimate& e) const {
    const double sigma_s = std::sqrt(std::max(e.p_ss, 0.0));
    const double sigma_v = std::sqrt(std::max(e.p_vv, 0.0));
    st.values.push_back(kv("k_sigma", std::to_string(k_sigma_)));
    st.values.push_back(kv("k_over", std::to_string(k_over_)));
    st.values.push_back(kv("sigma_s_m", std::to_string(sigma_s)));
    st.values.push_back(kv("sigma_v_mps", std::to_string(sigma_v)));
    st.values.push_back(kv(
        "pl_al_ratio",
        e.s_unbounded ? "unbounded"
                      : std::to_string(e.pl_s_m / std::max(e.al_s_m, 1e-9))));

    // F-23. p_mm is the variance of log m, so sigma_log_mass is directly the
    // relative 1-sigma on mass and sigma_mass_kg is the same figure in
    // kilograms. The repeated-fusion bug pinned this at 0.0293 (+-822 kg at
    // 28 t) against a declared loading range of +-12500 kg, entirely from
    // re-using one prior rather than from evidence. On the wire it is a
    // number an operator can watch; buried in the covariance it was not.
    const double sigma_log_m = std::sqrt(std::max(e.p_mm, 0.0));
    st.values.push_back(kv("m_eff_kg", std::to_string(e.x.m_eff_kg)));
    st.values.push_back(kv("p_mm", std::to_string(e.p_mm)));
    st.values.push_back(kv("sigma_log_mass", std::to_string(sigma_log_m)));
    st.values.push_back(kv(
        "sigma_mass_kg", std::to_string(sigma_log_m * std::max(e.x.m_eff_kg, 0.0))));
    st.values.push_back(kv("mass_prior_log_sigma", std::to_string(mass_prior_log_sigma_)));
    st.values.push_back(kv("mass_prior_tau_s", std::to_string(mass_prior_tau_s_)));

    // F-24. n_huber_capped separates "the axles agree" from "the axles were
    // capped into agreeing". The reported nis is now the pre-inflation
    // statistic, so a suppressed channel no longer hides inside a value
    // saturated at huber_c^2.
    st.values.push_back(kv("n_huber_capped", std::to_string(e.n_huber_capped)));
    st.values.push_back(kv("huber_c", std::to_string(huber_c_)));

    // The three states that absorb model error. If F_bias is drifting to
    // explain an unmodelled grade, the mass and adhesion estimates are being
    // poisoned to pay for it - which is only diagnosable with all three
    // visible together.
    st.values.push_back(kv("f_bias_n", std::to_string(e.x.f_bias_n)));
    st.values.push_back(kv("k_trac", std::to_string(e.x.k_trac)));
    st.values.push_back(kv("mu_hat", std::to_string(e.x.mu_hat)));
  }

  // Everything an operator needs to tell "the filter is fine" apart from
  // "the filter is being fed a broken timeline" (F-17 / F-17b).
  void push_time_hygiene(diagnostic_msgs::msg::DiagnosticStatus& st) const {
    st.values.push_back(kv("timebase", timebase_is_meas_ ? "header_stamp" : "node_clock"));
    st.values.push_back(kv("stamp_skew_s", std::to_string(stamp_skew_s_)));
    st.values.push_back(kv("stamp_skew_events", std::to_string(stamp_skew_events_)));
    st.values.push_back(kv("dt_s", std::to_string(last_dt_s_)));
    st.values.push_back(kv("stamp_regressions", std::to_string(stamp_regressions_)));
    st.values.push_back(kv("dt_gaps", std::to_string(dt_gaps_)));
    st.values.push_back(kv("dt_floored", std::to_string(dt_floored_)));
    st.values.push_back(kv("catchup_steps", std::to_string(catchup_steps_)));
    st.values.push_back(kv("bad_notch", std::to_string(bad_notch_)));
    st.values.push_back(kv("bad_brake", std::to_string(bad_brake_)));
    st.values.push_back(kv("bad_wheels", std::to_string(bad_wheels_)));
    st.values.push_back(kv("n_omega_nonfinite", std::to_string(n_omega_nonfinite_)));
    st.values.push_back(kv("input_fault", input_fault_ ? "1" : "0"));
  }

  void tick() {
    const rclcpp::Time t = now();
    // last_wall_ is written by every executed step and is always this node's
    // clock, so this comparison is single-timebase by construction.
    if ((have_wheels_ || have_notch_) && last_wall_.has_value()) {
      const double idle = (t - *last_wall_).seconds();
      if (std::isfinite(idle) && idle > 1.5 * period_s_) {
        step_filter(false);
      }
    }
    publish_state(t);
  }

  tram_dr::Ukf ukf_{};
  tram_dr::Input last_u_{};
  tram_dr::UkfEstimate last_e_{};
  std::array<double, tram_dr::kNWheels> omega_{};
  std::size_t n_omega_{0};
  bool have_notch_{false};
  bool have_brake_{false};
  bool have_wheels_{false};
  bool have_estimate_{false};
  std::size_t n_wheels_param_{4};
  double notch_max_abs_{8.0};
  tram_dr::NotchEncoding notch_enc_{tram_dr::NotchEncoding::kAuto};
  double age_degraded_s_{0.25};
  double age_lost_s_{1.0};
  double period_s_{0.02};
  double r0_m_{0.35};
  double wheel_latency_s_{0.0};
  bool twist_is_omega_{false};
  std::string vehicle_profile_{"combino_nf100"};
  bool last_step_used_wheels_{false};

  // --- integrity reporting (published, never used in a decision) ---------
  double k_sigma_{2.0};
  double k_over_{2.5};
  double huber_c_{3.0};
  double mass_prior_log_sigma_{0.3};
  double mass_prior_tau_s_{300.0};

  // --- time hygiene state (F-17b) ---------------------------------------
  double notch_stale_s_{0.25};
  double brake_stale_s_{0.25};
  double default_dt_s_{0.02};
  double dt_min_s_{0.001};
  double dt_max_s_{0.20};
  double stamp_regression_tol_s_{0.001};
  double stamp_skew_warn_s_{5.0};
  int max_catchup_steps_{18000};
  // Measurement-timebase anchor: only ever differenced against header stamps.
  std::optional<rclcpp::Time> last_meas_stamp_;
  // Node-clock anchor: only ever differenced against now().
  std::optional<rclcpp::Time> last_wall_;
  std::optional<rclcpp::Time> last_wheel_arrival_;
  std::optional<rclcpp::Time> last_notch_arrival_;
  std::optional<rclcpp::Time> last_brake_arrival_;
  double consumed_since_meas_s_{0.0};
  double stamp_skew_s_{0.0};
  double last_dt_s_{0.0};
  bool timebase_is_meas_{false};
  bool warned_skew_{false};
  bool input_fault_{false};
  int stamp_skew_events_{0};
  int stamp_regressions_{0};
  int dt_gaps_{0};
  int dt_floored_{0};
  int catchup_steps_{0};
  int bad_notch_{0};
  int bad_brake_{0};
  int bad_wheels_{0};
  int n_omega_nonfinite_{0};

  rclcpp::QoS in_qos_{1};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_notch_f32_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_notch_f64_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_notch_i8_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_notch_i16_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_brake_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_wheels_f64_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wheels_f32_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheels_js_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheels_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
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

#include "tram_dr_localization/ukf.hpp"

// UNINITIALIZED until notch or wheel_odom. No Imu / NavSatFix / PointCloud2.
// Predict is event-driven (wheel callback). Δt from header.stamp when the
// type has one; else receive time. Timer is watchdog + publish. Always
// publish (loss of wheels must not silence the topic).

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
    cfg.q_fb_wheels_n = declare_parameter("q_fb_wheels_n", 3000.0);
    cfg.stop_gate_m = declare_parameter("stop_gate_m", 40.0);
    const auto stops = declare_parameter("route_s_m", std::vector<double>{});
    cfg.n_stops = 0;
    for (double s : stops) {
      if (cfg.n_stops >= tram_dr::kMaxStops) {
        break;
      }
      cfg.stop_s_m[static_cast<std::size_t>(cfg.n_stops++)] = s;
    }
    ukf_.set_params(cfg);

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
    const auto roles = declare_parameter("axle_role", std::vector<int64_t>{});
    for (int i = 0; i < tram_dr::kNWheels; ++i) {
      sca.axle_role[static_cast<std::size_t>(i)] = 0;
      if (i < static_cast<int>(roles.size()) && roles[static_cast<std::size_t>(i)] != 0) {
        sca.axle_role[static_cast<std::size_t>(i)] = tram_dr::kAxleTrailer;
      }
    }
    ukf_.set_sca(sca);

    n_wheels_param_ = static_cast<std::size_t>(
        std::max(1, declare_parameter("n_wheels", 4)));
    n_wheels_param_ = std::min(n_wheels_param_, static_cast<std::size_t>(tram_dr::kNWheels));
    notch_max_abs_ = declare_parameter("notch_max_abs", 8.0);
    age_degraded_s_ = declare_parameter("age_degraded_s", 0.25);
    age_lost_s_ = declare_parameter("age_lost_s", 1.0);
    twist_is_omega_ = declare_parameter("twist_is_omega", false);
    const std::string notch_type = declare_parameter("notch_type", std::string("float32"));
    const std::string wheels_type = declare_parameter("wheels_type", std::string("float64_array"));
    if (notch_type != "float32" && notch_type != "float64" && notch_type != "int8" &&
        notch_type != "int16") {
      throw std::invalid_argument(
          "notch_type must be float32|float64|int8|int16, got " + notch_type);
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
          last_u_.brake = std::clamp(static_cast<double>(msg.data), 0.0, 1.0);
        });
    subscribe_wheels(wheels_type);
    RCLCPP_INFO(get_logger(),
                "state_estimator notch_type=%s wheels_type=%s model=%s vehicle=%s n=%zu "
                "stops=%d (mass_door gated)",
                notch_type.c_str(), wheels_type.c_str(), tram_dr::kModelVersion,
                vehicle_profile_.c_str(), n_wheels_param_, cfg.n_stops);
    const int period_ms =
        std::max(1, static_cast<int>(std::lround(1000.0 / std::max(rate_hz, 1.0))));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() { tick(); });
  }

 private:
  void apply_notch(double raw) {
    last_u_.notch = tram_dr::map_notch(raw, notch_max_abs_);
    last_u_.notch_valid = true;
    have_notch_ = true;
  }

  void apply_wheels(const double* data, std::size_t n, const rclcpp::Time* stamp = nullptr) {
    n_omega_ = std::min(n, n_wheels_param_);
    bool any_ok = false;
    for (std::size_t i = 0; i < n_omega_; ++i) {
      omega_[i] = data[i];
      if (std::isfinite(data[i]) && std::fabs(data[i]) <= tram_dr::kOmegaAbsMax) {
        any_ok = true;
      }
    }
    have_wheels_ = any_ok;
    if (any_ok) {
      last_wheel_stamp_ = (stamp != nullptr && stamp->nanoseconds() > 0) ? *stamp : now();
      step_filter(true, stamp);
    }
  }

  void apply_twist_vx(double vx, const rclcpp::Time* stamp = nullptr) {
    const double w = twist_is_omega_ ? vx : vx / std::max(r0_m_, 1e-6);
    double tmp[tram_dr::kNWheels];
    for (std::size_t i = 0; i < n_wheels_param_; ++i) {
      tmp[i] = w;
    }
    apply_wheels(tmp, n_wheels_param_, stamp);
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

  void step_filter(bool fresh_wheels, const rclcpp::Time* meas_stamp = nullptr) {
    const rclcpp::Time t =
        (meas_stamp != nullptr && meas_stamp->nanoseconds() > 0) ? *meas_stamp : now();
    double dt = 0.02;
    if (last_step_.has_value()) {
      dt = (t - *last_step_).seconds();
    }
    dt = std::clamp(dt, 0.005, 0.20);
    last_step_ = t;
    last_step_used_wheels_ = fresh_wheels;

    tram_dr::Input u = last_u_;
    if (!have_notch_) {
      u.notch_valid = false;
    }
    const std::size_t n = (fresh_wheels && have_wheels_) ? n_omega_ : 0;
    last_e_ = ukf_.predict_and_update(u, n > 0 ? omega_.data() : nullptr, n, dt);
    have_estimate_ = true;
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

    const double age_s =
        last_wheel_stamp_.has_value() ? std::max(0.0, (t - *last_wheel_stamp_).seconds()) : 1.0e9;

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
      d.status.push_back(st);
      diag_->publish(d);
      return;
    }

    tram_dr::UkfEstimate e = last_e_;
    tram_dr::Confidence conf = e.confidence;
    if (e.initialized && age_s > age_lost_s_) {
      conf = tram_dr::Confidence::kLost;
    } else if (e.initialized && age_s > age_degraded_s_ && conf == tram_dr::Confidence::kOk) {
      conf = tram_dr::Confidence::kDegraded;
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
    st.values.push_back(kv("confidence", tram_dr::confidence_label(conf)));
    st.values.push_back(kv("confidence_v", tram_dr::confidence_label(e.confidence_v)));
    st.values.push_back(kv("confidence_s", tram_dr::confidence_label(e.confidence_s)));
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
    st.values.push_back(kv("zupt_at_stop", e.zupt_at_stop ? "1" : "0"));
    st.values.push_back(kv("v_chan_a", std::to_string(e.v_chan_a_mps)));
    st.values.push_back(kv("nis", e.nis_valid ? std::to_string(e.nis) : "nan"));
    st.values.push_back(kv("nis_valid", e.nis_valid ? "1" : "0"));
    st.values.push_back(kv("nis_cusum", std::to_string(e.nis_cusum)));
    st.values.push_back(kv("over_m", std::to_string(e.over_m)));
    st.values.push_back(kv("under_m", std::to_string(e.under_m)));
    st.values.push_back(kv("b_s_m", std::to_string(e.b_s_m)));
    st.values.push_back(kv("pl_s_m", std::to_string(e.pl_s_m)));
    st.values.push_back(kv("pl_v_mps", std::to_string(e.pl_v_mps)));
    st.values.push_back(kv("al_s_m", std::to_string(e.al_s_m)));
    st.values.push_back(kv("a_kin", std::to_string(e.a_kin_mps2)));
    st.values.push_back(kv("a_unphysical", e.a_unphysical ? "1" : "0"));
    d.status.push_back(st);
    diag_->publish(d);
  }

  void tick() {
    const rclcpp::Time t = now();
    if ((have_wheels_ || have_notch_) && last_step_.has_value()) {
      const double idle = (t - *last_step_).seconds();
      if (idle > 0.015) {
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
  bool have_wheels_{false};
  bool have_estimate_{false};
  std::size_t n_wheels_param_{4};
  double notch_max_abs_{8.0};
  double age_degraded_s_{0.25};
  double age_lost_s_{1.0};
  double r0_m_{0.35};
  double wheel_latency_s_{0.0};
  bool twist_is_omega_{false};
  std::string vehicle_profile_{"combino_nf100"};
  bool last_step_used_wheels_{false};
  std::optional<rclcpp::Time> last_step_;
  std::optional<rclcpp::Time> last_wheel_stamp_;
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

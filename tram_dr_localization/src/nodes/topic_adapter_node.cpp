#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"

#include "tram_dr_localization/types.hpp"

// Remaps organiser topics onto canonical /tram/*. Does not run the filter.
// Does not subscribe to IMU / NavSatFix / PointCloud2.
// enable=false (default): idle — bag already on /tram/* or names unknown until 25.09.
// Never subscribe to a topic that equals the matching /tram/* output (self-loop).

class TopicAdapterNode : public rclcpp::Node {
 public:
  TopicAdapterNode() : Node("topic_adapter") {
    const bool enable = declare_parameter("enable", false);
    in_notch_ = declare_parameter("in_notch_topic", std::string("/cbt/controller_notch"));
    in_brake_ = declare_parameter("in_brake_topic", std::string("/cbt/brake_cmd"));
    in_wheels_ = declare_parameter("in_wheels_topic", std::string("/cbt/wheel_speeds"));
    const bool notch_int8_flag = declare_parameter("in_notch_int8", false);
    std::string notch_type = declare_parameter("in_notch_type", std::string("float32"));
    std::string wheels_type = declare_parameter("in_wheels_type", std::string("float64_array"));
    if (notch_int8_flag) {
      notch_type = "int8";
    }
    notch_max_abs_ = declare_parameter("notch_max_abs", 8.0);
    n_wheels_ = static_cast<std::size_t>(std::clamp(
        static_cast<int>(declare_parameter("n_wheels", 4)), 1, tram_dr::kNWheels));
    r0_m_ = declare_parameter("wheel_radius_m", 0.35);
    twist_is_omega_ = declare_parameter("twist_is_omega", false);
    if (notch_type == "int8" || notch_type == "int16") {
      notch_enc_ = tram_dr::NotchEncoding::kDiscrete;
    }
    const bool map_notch = in_notch_ != kOutNotch;
    const bool map_brake = in_brake_ != kOutBrake;
    const bool map_wheels = in_wheels_ != kOutWheels;
    if (!enable) {
      RCLCPP_INFO(get_logger(),
                  "topic_adapter idle (enable=false). Canonical /tram/*; other names → "
                  "inspect_bag --write-yaml then enable:=true.");
      return;
    }
    if (notch_type != "float32" && notch_type != "float64" && notch_type != "int8" &&
        notch_type != "int16") {
      throw std::invalid_argument(
          "in_notch_type must be float32|float64|int8|int16, got " + notch_type);
    }
    if (wheels_type != "float64_array" && wheels_type != "float32_array" &&
        wheels_type != "joint_state" && wheels_type != "twist_stamped") {
      throw std::invalid_argument(
          "in_wheels_type must be float64_array|float32_array|joint_state|twist_stamped, got " +
          wheels_type);
    }
    if (!map_notch && !map_brake && !map_wheels) {
      RCLCPP_WARN(get_logger(),
                  "topic_adapter refusing self-subscribe: all in_* equal /tram/* outputs");
      return;
    }
    if (map_notch) {
      pub_notch_ = create_publisher<std_msgs::msg::Float32>(kOutNotch, rclcpp::SensorDataQoS());
      if (notch_type == "int8") {
        sub_notch_i8_ = create_subscription<std_msgs::msg::Int8>(
            in_notch_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Int8& msg) { publish_notch(static_cast<double>(msg.data)); });
      } else if (notch_type == "int16") {
        sub_notch_i16_ = create_subscription<std_msgs::msg::Int16>(
            in_notch_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Int16& msg) {
              publish_notch(static_cast<double>(msg.data));
            });
      } else if (notch_type == "float64") {
        sub_notch_f64_ = create_subscription<std_msgs::msg::Float64>(
            in_notch_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64& msg) { publish_notch(msg.data); });
      } else {
        sub_notch_f32_ = create_subscription<std_msgs::msg::Float32>(
            in_notch_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float32& msg) {
              publish_notch(static_cast<double>(msg.data));
            });
      }
    }
    if (map_brake) {
      pub_brake_ = create_publisher<std_msgs::msg::Float32>(kOutBrake, rclcpp::SensorDataQoS());
      sub_brake_ = create_subscription<std_msgs::msg::Float32>(
          in_brake_, rclcpp::SensorDataQoS(),
          [this](const std_msgs::msg::Float32& msg) {
            if (!std::isfinite(static_cast<double>(msg.data))) {
              return;
            }
            std_msgs::msg::Float32 out;
            out.data = static_cast<float>(std::clamp(static_cast<double>(msg.data), 0.0, 1.0));
            pub_brake_->publish(out);
          });
    }
    if (map_wheels) {
      pub_wheels_ = create_publisher<std_msgs::msg::Float64MultiArray>(kOutWheels,
                                                                      rclcpp::SensorDataQoS());
      if (wheels_type == "float32_array") {
        sub_wheels_f32_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            in_wheels_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float32MultiArray& msg) {
              pub_wheels_->publish(pad_wheel_array(msg.data));
            });
      } else if (wheels_type == "joint_state") {
        sub_wheels_js_ = create_subscription<sensor_msgs::msg::JointState>(
            in_wheels_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState& msg) {
              pub_wheels_->publish(pad_wheel_array(msg.velocity));
            });
      } else if (wheels_type == "twist_stamped") {
        sub_wheels_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            in_wheels_, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::TwistStamped& msg) {
              const double vx = msg.twist.linear.x;
              if (!std::isfinite(vx)) {
                return;
              }
              const double w = twist_is_omega_ ? vx : vx / std::max(r0_m_, 1e-6);
              if (!std::isfinite(w)) {
                return;
              }
              // One TwistStamped is one channel. Repeating w n times was the
              // same silent-zero class as padding a short JointState with the
              // last live ω (F-26). Estimator pads the tail with NaN.
              std_msgs::msg::Float64MultiArray out;
              out.data = {w};
              pub_wheels_->publish(out);
            });
      } else {
        sub_wheels_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            in_wheels_, rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64MultiArray& msg) {
              pub_wheels_->publish(pad_wheel_array(msg.data));
            });
      }
    }
    RCLCPP_INFO(get_logger(), "topic_adapter %s (%s) + %s + %s (%s) → /tram/*", in_notch_.c_str(),
                notch_type.c_str(), in_brake_.c_str(), in_wheels_.c_str(), wheels_type.c_str());
  }

 private:
  static constexpr const char* kOutNotch = "/tram/controller_notch";
  static constexpr const char* kOutBrake = "/tram/brake_cmd";
  static constexpr const char* kOutWheels = "/tram/wheel_odom";

  void publish_notch(double raw) {
    if (!std::isfinite(raw)) {
      return;
    }
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(tram_dr::map_notch(raw, notch_max_abs_, notch_enc_));
    pub_notch_->publish(out);
  }

  template <typename Seq>
  std_msgs::msg::Float64MultiArray pad_wheel_array(const Seq& src) const {
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(n_wheels_, std::numeric_limits<double>::quiet_NaN());
    const std::size_t n = std::min(src.size(), n_wheels_);
    for (std::size_t i = 0; i < n; ++i) {
      out.data[i] = static_cast<double>(src[i]);
    }
    return out;
  }

  std::string in_notch_;
  std::string in_brake_;
  std::string in_wheels_;
  double notch_max_abs_{8.0};
  tram_dr::NotchEncoding notch_enc_{tram_dr::NotchEncoding::kAuto};
  std::size_t n_wheels_{4};
  double r0_m_{0.35};
  bool twist_is_omega_{false};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_notch_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_brake_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheels_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_notch_f32_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_notch_f64_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_notch_i8_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_notch_i16_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_brake_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_wheels_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_wheels_f32_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_wheels_js_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheels_twist_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicAdapterNode>());
  rclcpp::shutdown();
  return 0;
}

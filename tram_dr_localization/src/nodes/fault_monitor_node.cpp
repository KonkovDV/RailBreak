#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

// Watchdog on /tram/diagnostics. Does not re-implement the estimator.
// RB08-07: only status.name == "tram_dr" is a heartbeat. Empty/foreign arrays
// do not refresh. Negative age (clock jump) is STALE.

class FaultMonitorNode : public rclcpp::Node {
 public:
  FaultMonitorNode() : Node("fault_monitor") {
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/tram/diagnostics_watchdog", 10);
    sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/tram/diagnostics", 10,
        [this](const diagnostic_msgs::msg::DiagnosticArray& msg) {
          for (const auto& st : msg.status) {
            if (st.name == "tram_dr") {
              last_diag_ = now();
              last_level_ = st.level;
              last_message_ = st.message;
              return;
            }
          }
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      diagnostic_msgs::msg::DiagnosticArray msg;
      msg.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus st;
      st.name = "tram_dr_watchdog";
      const bool never = !last_diag_.has_value();
      const double age_s = never ? 1.0e9 : (now() - *last_diag_).seconds();
      if (never || !std::isfinite(age_s) || age_s > 0.5 || age_s < 0.0) {
        st.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        st.message = "estimator silent";
      } else {
        st.level = last_level_;
        st.message = last_message_.empty() ? "watchdog" : last_message_;
      }
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "age_s";
      kv.value = std::to_string(age_s);
      st.values.push_back(kv);
      msg.status.push_back(st);
      pub_->publish(msg);
    });
  }

 private:
  std::optional<rclcpp::Time> last_diag_;
  unsigned char last_level_{diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::string last_message_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultMonitorNode>());
  rclcpp::shutdown();
  return 0;
}

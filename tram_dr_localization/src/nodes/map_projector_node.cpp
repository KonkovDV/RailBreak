#include <chrono>
#include <memory>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tram_dr_localization/map.hpp"

// Publishes dead-reckoning lat/lon. This is not a GNSS receiver.
// status = STATUS_NO_FIX; frame_id = dead_reckoning.
// Subscribes to /tram/state_estimate (s in pose.position.x). Does not subscribe
// to a GNSS receiver. Polyline comes from route_* parameters
// (config/route_10.yaml); map.cpp fallback is the same OSM stop vertices.

class MapProjectorNode : public rclcpp::Node {
 public:
  MapProjectorNode() : Node("map_projector") {
    const auto s = declare_parameter<std::vector<double>>("route_s_m", {});
    const auto lat = declare_parameter<std::vector<double>>("route_lat_deg", {});
    const auto lon = declare_parameter<std::vector<double>>("route_lon_deg", {});
    if (s.size() >= 2 && s.size() == lat.size() && s.size() == lon.size()) {
      bool increasing = true;
      for (std::size_t i = 1; i < s.size(); ++i) {
        increasing = increasing && (s[i] > s[i - 1]);
      }
      if (increasing) {
        nodes_.reserve(s.size());
        for (std::size_t i = 0; i < s.size(); ++i) {
          nodes_.push_back({s[i], lat[i], lon[i]});
        }
      }
    }
    if (nodes_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "route_* missing; using built-in OSM stop vertices");
    }
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/tram/fix", 1);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/tram/state_estimate", rclcpp::QoS(1).reliable(),
        [this](const nav_msgs::msg::Odometry& odom) {
          last_s_ = odom.pose.pose.position.x;
          last_pss_ = odom.pose.covariance[0];
          have_odom_ = true;
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!have_odom_) {
        return;
      }
      sensor_msgs::msg::NavSatFix msg;
      msg.header.stamp = now();
      msg.header.frame_id = "dead_reckoning";
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      msg.status.service = 0;
      if (nodes_.empty()) {
        tram_dr::project_s(last_s_, msg.latitude, msg.longitude);
      } else {
        tram_dr::project_s(last_s_, msg.latitude, msg.longitude, nodes_);
      }
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      const double pss = have_odom_ ? last_pss_ : 1.0e4;
      msg.position_covariance[0] = pss;
      msg.position_covariance[4] = pss;
      msg.position_covariance[8] = 1.0e6;  // no height from 1D DR
      pub_->publish(msg);
    });
  }

 private:
  double last_s_{0.0};
  double last_pss_{1.0e4};
  bool have_odom_{false};
  std::vector<tram_dr::MapNode> nodes_{};
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapProjectorNode>());
  rclcpp::shutdown();
  return 0;
}

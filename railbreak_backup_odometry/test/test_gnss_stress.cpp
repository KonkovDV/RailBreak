// ROS executor stress for the start-window drain. Not a direct GnssWindow call:
// fixes and bogies are published on the real subscriptions, and the node runs
// in a SingleThreadedExecutor.
#define RAILBREAK_ODOMETRY_NO_MAIN
#include "backup_odometry_node.cpp"

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

struct GnssDrainProbe {
  static int master_fixes(const BackupOdometryNode& n) { return n.win_.master_fixes(); }
  static bool closed(const BackupOdometryNode& n) { return n.win_.closed; }
  static int fixes_used(const BackupOdometryNode& n) { return n.n_fix_used_; }
  static int behind(const BackupOdometryNode& n) { return static_cast<int>(n.n_behind_out_); }
};

namespace {

builtin_interfaces::msg::Time to_stamp(double t) {
  builtin_interfaces::msg::Time out;
  out.sec = static_cast<int32_t>(std::floor(t));
  out.nanosec = static_cast<uint32_t>(std::llround((t - std::floor(t)) * 1e9));
  if (out.nanosec >= 1000000000u) {
    out.nanosec -= 1000000000u;
    ++out.sec;
  }
  return out;
}

double stamp_sec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

template <typename Pred>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& exec, Pred pred, int ms) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(20));
    if (pred()) return true;
  }
  exec.spin_some(std::chrono::milliseconds(20));
  return pred();
}

sensor_msgs::msg::NavSatFix make_fix(double t, double lat, double lon, int8_t status) {
  sensor_msgs::msg::NavSatFix m;
  m.header.stamp = to_stamp(t);
  m.header.frame_id = "gnss";
  m.status.status = status;
  m.latitude = lat;
  m.longitude = lon;
  m.altitude = 160.0;
  return m;
}

tram_vehicle_msgs::msg::VelocitySensor make_wheel(double t, double raw) {
  tram_vehicle_msgs::msg::VelocitySensor m;
  m.header.stamp = to_stamp(t);
  m.header.frame_id = "base_link";
  m.velocity = raw;
  return m;
}

std::string diag_value(const diagnostic_msgs::msg::DiagnosticArray& a, const char* key) {
  for (const auto& st : a.status) {
    for (const auto& kv : st.values) {
      if (kv.key == key) return kv.value;
    }
  }
  return {};
}

}  // namespace

TEST(GnssDrain, queued_fix_survives_wheels_and_invalid_after_close_is_ignored) {
  struct RosInit {
    RosInit() {
      if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    }
    ~RosInit() {
      if (rclcpp::ok()) rclcpp::shutdown();
    }
  } ros;
  (void)ros;
  const auto assets = railbreak::load_assets(RAILBREAK_ASSETS_DIR);
  double lat = 0.0, lon = 0.0;
  assets.map.latlon(assets.map.x.front(), assets.map.y.front(), lat, lon);

  rclcpp::NodeOptions options;
  options.append_parameter_override("assets_dir", std::string(RAILBREAK_ASSETS_DIR));
  options.append_parameter_override("diagnostics_every_n", 1);
  auto node = std::make_shared<BackupOdometryNode>(options);
  auto io = std::make_shared<rclcpp::Node>("gnss_stress_io");

  const auto in_qos = rclcpp::SensorDataQoS().keep_last(500);
  const auto out_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
  auto pub_fix = io->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/master/fix", in_qos);
  auto pub_wheel = io->create_publisher<tram_vehicle_msgs::msg::VelocitySensor>(
      "/vehicle/front_bogie_velocity", in_qos);

  std::vector<double> pose_stamps;
  diagnostic_msgs::msg::DiagnosticArray last_diag;
  auto sub_pose = io->create_subscription<nav_msgs::msg::Odometry>(
      "/result/position", out_qos, [&](const nav_msgs::msg::Odometry& m) {
        pose_stamps.push_back(stamp_sec(m.header.stamp));
      });
  auto sub_diag = io->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/result/diagnostics", 10, [&](const diagnostic_msgs::msg::DiagnosticArray& m) { last_diag = m; });
  (void)sub_pose;
  (void)sub_diag;

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(io);

  ASSERT_TRUE(spin_until(exec, [&] { return pub_fix->get_subscription_count() > 0 &&
                                             pub_wheel->get_subscription_count() > 0; },
                         20000))
      << "subscriptions did not match";

  pub_fix->publish(make_fix(0.0, lat, lon, sensor_msgs::msg::NavSatStatus::STATUS_FIX));
  ASSERT_TRUE(spin_until(exec, [&] { return GnssDrainProbe::master_fixes(*node) >= 1; }, 5000))
      << "the opening fix was not delivered";

  // Two more in-window fixes go out with the wheels, before the executor runs.
  // Whichever subscription the executor serves first, take() on the wheel path
  // or the fix callback itself must keep both samples. Eight quiet wheels are
  // not treated as proof that this queue is empty.
  pub_fix->publish(make_fix(2.0, lat, lon, sensor_msgs::msg::NavSatStatus::STATUS_FIX));
  pub_fix->publish(make_fix(2.5, lat, lon, sensor_msgs::msg::NavSatStatus::STATUS_FIX));
  for (int i = 0; i < 40; ++i) pub_wheel->publish(make_wheel(3.2 + 0.05 * i, 36.0));

  ASSERT_TRUE(spin_until(
      exec,
      [&] {
        return GnssDrainProbe::closed(*node) && diag_value(last_diag, "gnss") == "closed";
      },
      10000))
      << "the start window did not close";
  EXPECT_GE(GnssDrainProbe::fixes_used(*node), 3);
  const int used = GnssDrainProbe::fixes_used(*node);
  const std::size_t n_pose = pose_stamps.size();

  for (int i = 0; i < 30; ++i) {
    pub_fix->publish(make_fix(6.0 + 0.05 * i, lat, lon, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX));
  }
  pub_fix->publish(make_fix(8.0, lat, lon, sensor_msgs::msg::NavSatStatus::STATUS_FIX));
  exec.spin_some(std::chrono::milliseconds(300));
  // The newest stamp stays queued until a later one covers stamp_reorder_s.
  pub_wheel->publish(make_wheel(8.2, 36.0));
  pub_wheel->publish(make_wheel(8.31, 36.0));
  ASSERT_TRUE(spin_until(
      exec,
      [&] {
        return !pose_stamps.empty() && pose_stamps.back() > 8.1 && !last_diag.status.empty() &&
               stamp_sec(last_diag.header.stamp) > 8.1;
      },
      5000));
  EXPECT_EQ(GnssDrainProbe::fixes_used(*node), used);
  EXPECT_TRUE(GnssDrainProbe::closed(*node));
  EXPECT_EQ(diag_value(last_diag, "gnss"), "closed");
  EXPECT_GT(pose_stamps.size(), n_pose);

  const int behind_before = GnssDrainProbe::behind(*node);
  pub_wheel->publish(make_wheel(7.5, 36.0));
  exec.spin_some(std::chrono::milliseconds(200));
  pub_wheel->publish(make_wheel(8.40, 36.0));
  pub_wheel->publish(make_wheel(8.51, 36.0));
  ASSERT_TRUE(spin_until(
      exec,
      [&] {
        return !pose_stamps.empty() && pose_stamps.back() > 8.3 && !last_diag.status.empty() &&
               stamp_sec(last_diag.header.stamp) > 8.3;
      },
      5000));
  for (double s : pose_stamps) EXPECT_GT(std::fabs(s - 7.5), 1e-3) << "regressed stamp was published";
  EXPECT_GT(GnssDrainProbe::behind(*node), behind_before);
  EXPECT_EQ(diag_value(last_diag, "gnss"), "closed");
}

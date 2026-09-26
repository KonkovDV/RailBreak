"""Burst GNSS and wheels at the live node. The node log carries the fix count."""
import time

import rclpy
from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor


def stamp(t):
    out = Time()
    out.sec = int(t)
    out.nanosec = int(round((t - int(t)) * 1e9))
    return out


def main():
    rclpy.init()
    node = rclpy.create_node("gnss_queue_stress")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=500,
    )
    gm = node.create_publisher(NavSatFix, "/sensing/gnss/master/fix", qos)
    gr = node.create_publisher(NavSatFix, "/sensing/gnss/rover/fix", qos)
    pf = node.create_publisher(VelocitySensor, "/vehicle/front_bogie_velocity", qos)
    pr = node.create_publisher(VelocitySensor, "/vehicle/rear_bogie_velocity", qos)
    pc = node.create_publisher(DriverControllerCommand, "/vehicle/driver_position_cmd", qos)
    deadline = time.time() + 5.0
    while gm.get_subscription_count() < 1 and time.time() < deadline:
        time.sleep(0.05)
    lat, lon = 55.810417, 37.462308
    lat_r, lon_r = 55.8105, 37.4625
    t0 = 1_000_000.0

    def fix(pub, t, latitude, longitude):
        m = NavSatFix()
        m.header.stamp = stamp(t)
        m.header.frame_id = "gnss"
        m.status.status = NavSatStatus.STATUS_FIX
        m.latitude = latitude
        m.longitude = longitude
        m.altitude = 160.0
        pub.publish(m)

    def wheel(pub, t):
        m = VelocitySensor()
        m.header.stamp = stamp(t)
        m.header.frame_id = "base_link"
        m.velocity = 0.0
        pub.publish(m)

    def cmd(t):
        m = DriverControllerCommand()
        m.header.stamp = stamp(t)
        m.header.frame_id = "base_link"
        m.position = 0
        pc.publish(m)

    # Arm, let those callbacks run, then queue a burst of in-window fixes
    # before any wheel that is already past the window.
    for t in (t0, t0 + 2.5):
        fix(gm, t, lat, lon)
        fix(gr, t, lat_r, lon_r)
    time.sleep(0.4)
    for _ in range(100):
        fix(gm, t0 + 2.8, lat, lon)
        fix(gr, t0 + 2.8, lat_r, lon_r)
    time.sleep(0.2)
    for i in range(20):
        wheel(pf, t0 + 4.0 + 0.001 * i)
        wheel(pr, t0 + 4.0 + 0.001 * i)
        cmd(t0 + 4.0 + 0.001 * i)
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""Does one executor cycle deliver eight wheel callbacks before queued GNSS?

The node has three wheel/command subscriptions and two GNSS subscriptions.
Humble's single-threaded executor runs one ready subscription per turn.
"""
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32


def longest_wheel(seq):
    best = cur = 0
    for c in seq:
        if c in ("F", "R", "C"):
            cur += 1
            best = max(best, cur)
        else:
            cur = 0
    return best


def run(gnss_first):
    node = rclpy.create_node("exec_order_probe")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=500,
    )
    seq = []
    topics = ["/gm", "/gr", "/f", "/r", "/c"] if gnss_first else ["/f", "/r", "/c", "/gm", "/gr"]
    marks = {"/f": "F", "/r": "R", "/c": "C", "/gm": "G", "/gr": "M"}
    for topic in topics:
        node.create_subscription(
            Int32, topic, lambda _m, mark=marks[topic]: seq.append(mark), qos)
    pubs = {topic: node.create_publisher(Int32, topic, qos) for topic in topics}
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    deadline = time.time() + 3.0
    while pubs["/f"].get_subscription_count() < 1 and time.time() < deadline:
        ex.spin_once(timeout_sec=0.05)
    msg = Int32()
    for i in range(100):
        msg.data = i
        for topic in ("/gm", "/gr", "/f", "/r", "/c"):
            pubs[topic].publish(msg)
    time.sleep(0.4)
    seq.clear()
    idle = 0
    while idle < 8:
        before = len(seq)
        ex.spin_once(timeout_sec=0.0)
        idle = idle + 1 if len(seq) == before else 0
    text = "".join(seq)
    print(
        f"gnss_first={gnss_first} n={len(text)} longest_wheel_run={longest_wheel(text)} "
        f"head={text[:70]}"
    )
    ex.remove_node(node)
    node.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    run(False)
    run(True)
    rclpy.shutdown()

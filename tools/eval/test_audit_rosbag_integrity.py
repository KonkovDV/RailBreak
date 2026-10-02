"""Regression tests for audit_rosbag_integrity."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from audit_rosbag_integrity import audit_bag
from rosbag2_io import write_bag


class RosbagIntegrityTests(unittest.TestCase):
    def _bag(self) -> Path:
        temp = Path(tempfile.mkdtemp(prefix="railbreak-bag-audit-"))
        rows = [
            ("/a", "std_msgs/msg/Float32", 100, b"x"),
            ("/b", "std_msgs/msg/Float32", 200, b"y"),
            ("/a", "std_msgs/msg/Float32", 300, b"z"),
        ]
        write_bag(temp, rows)
        return temp

    def test_clean_writer_output_passes(self) -> None:
        bag = self._bag()
        report = audit_bag(bag)
        self.assertTrue(report["ok"], report["issues"])
        self.assertEqual(report["actual_message_count"], 3)

    def test_file_message_count_mismatch_is_fatal(self) -> None:
        bag = self._bag()
        metadata = (bag / "metadata.yaml").read_text(encoding="utf-8")
        metadata = metadata.replace("message_count: 3\n", "message_count: 3\n", 1)
        metadata = metadata.replace("      message_count: 3\n", "      message_count: 99\n", 1)
        (bag / "metadata.yaml").write_text(metadata, encoding="utf-8")
        report = audit_bag(bag)
        self.assertFalse(report["ok"])
        self.assertTrue(any("file bag_0.db3 message_count" in x for x in report["issues"]))

    def test_topic_count_mismatch_is_fatal(self) -> None:
        bag = self._bag()
        metadata = (bag / "metadata.yaml").read_text(encoding="utf-8")
        metadata = metadata.replace("      message_count: 2\n", "      message_count: 7\n", 1)
        (bag / "metadata.yaml").write_text(metadata, encoding="utf-8")
        report = audit_bag(bag)
        self.assertFalse(report["ok"])
        self.assertTrue(any("topic /a" in x for x in report["issues"]))


if __name__ == "__main__":
    unittest.main()

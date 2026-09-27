"""Ablation matrix: ten variants, measured columns, no claimed percentile."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from ablation import VARIANTS, ablation_matrix  # noqa: E402


class AblationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = ablation_matrix()
        cls.by = {row["variant"]: row for row in cls.rows}

    def test_ten_variants(self):
        self.assertEqual([row["variant"] for row in self.rows], list(VARIANTS))

    def test_open_loop_has_no_interval_and_no_alarm(self):
        for name in ("mean_odometry", "front_only", "rear_only", "model_only"):
            row = self.by[name]
            self.assertIsNone(row["interval_coverage"])
            self.assertIsNone(row["false_degraded"])
            self.assertIsNone(row["missed_slip"])
            self.assertIsInstance(row["rmse_v"], float)
            self.assertIsInstance(row["rmse_s"], float)

    def test_monitor_does_not_move_the_filter(self):
        base = self.by["current_filter"]
        monitor = self.by["integrity_monitor"]
        self.assertAlmostEqual(base["rmse_s"], monitor["rmse_s"])
        self.assertAlmostEqual(base["rmse_v"], monitor["rmse_v"])
        self.assertAlmostEqual(base["final_drift"], monitor["final_drift"])
        self.assertIsNotNone(monitor["false_degraded"])
        self.assertIsNotNone(monitor["missed_slip"])
        self.assertGreaterEqual(monitor["false_degraded"], 0.0)
        self.assertLessEqual(monitor["missed_slip"], 1.0)

    def test_anchor_fires_only_when_a_station_is_given(self):
        self.assertGreaterEqual(self.by["current_filter"]["n_anchor"], 1)
        self.assertEqual(self.by["no_anchors"]["n_anchor"], 0)
        self.assertNotAlmostEqual(
            self.by["current_filter"]["final_drift"],
            self.by["no_anchors"]["final_drift"],
            places=3,
        )

    def test_grade_changes_the_arc(self):
        self.assertGreater(
            abs(self.by["no_grade"]["rmse_s"] - self.by["current_filter"]["rmse_s"]),
            1.0e-3,
        )

    def test_coverage_is_not_a_claimed_percentile(self):
        for row in self.rows:
            self.assertFalse(row["claimed_percentile"])
            self.assertFalse(row["replaces_published_val"])
            self.assertEqual(row["reference"], "truth_sim")
            self.assertGreater(row["latency_s"], 0.0)
            self.assertGreater(row["memory_bytes"], 0)
            if row["interval_coverage"] is not None:
                self.assertGreaterEqual(row["interval_coverage"], 0.0)
                self.assertLessEqual(row["interval_coverage"], 1.0)


if __name__ == "__main__":
    unittest.main()

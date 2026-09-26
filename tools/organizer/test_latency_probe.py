"""Latency percentiles. No ROS graph and no stamp written into an output."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import latency_probe as lp  # noqa: E402


class LatencyProbeTests(unittest.TestCase):
    def test_steady_percentiles_leave_the_warmup_out(self):
        summary = lp.split_warmup([0.10, 0.20, 0.01, 0.02, 0.03, 0.04], warmup_n=2)
        self.assertEqual(summary["warmup"]["n"], 2)
        self.assertAlmostEqual(summary["warmup"]["p50_s"], 0.15)
        self.assertAlmostEqual(summary["warmup"]["max_s"], 0.20)
        self.assertEqual(summary["steady"]["n"], 4)
        self.assertAlmostEqual(summary["steady"]["p50_s"], 0.025)
        self.assertAlmostEqual(summary["steady"]["max_s"], 0.04)
        self.assertGreater(summary["steady"]["p99_s"], summary["steady"]["p50_s"])
        self.assertLess(summary["steady"]["p99_s"], summary["steady"]["max_s"] + 1e-12)

    def test_one_steady_sample_fills_every_percentile(self):
        summary = lp.split_warmup([0.5, 0.002], warmup_n=1)
        self.assertEqual(summary["steady"]["n"], 1)
        self.assertAlmostEqual(summary["steady"]["p50_s"], 0.002)
        self.assertAlmostEqual(summary["steady"]["p95_s"], 0.002)
        self.assertAlmostEqual(summary["steady"]["p99_s"], 0.002)
        self.assertAlmostEqual(summary["steady"]["max_s"], 0.002)

    def test_match_is_one_output_receive_minus_one_input_receive(self):
        inputs = [(0.0, 10.0), (0.1, 10.1), (0.1, 10.2)]
        outputs = [(0.1, 10.105), (0.0, 10.004)]
        lat = lp.match_receives(inputs, outputs)
        self.assertEqual(len(lat), 2)
        self.assertAlmostEqual(lat[0], 0.004)
        self.assertAlmostEqual(lat[1], 10.105 - 10.1)
        text = Path(lp.__file__).read_text(encoding="utf-8")
        self.assertNotIn("header.stamp =", text)
        report = lp.report(inputs, outputs, warmup_n=1)
        self.assertFalse(report["stamp_modified"])
        self.assertFalse(report["callback_max_us_is_this"])
        self.assertEqual(report["n_matched"], 2)
        self.assertEqual(report["warmup"]["n"], 1)
        self.assertEqual(report["steady"]["n"], 1)

    def test_short_record_has_an_empty_steady_part(self):
        summary = lp.split_warmup([0.01], warmup_n=10)
        self.assertEqual(summary["warmup"]["n"], 1)
        self.assertEqual(summary["steady"]["n"], 0)
        self.assertNotEqual(summary["steady"]["p50_s"], summary["steady"]["p50_s"])


if __name__ == "__main__":
    unittest.main()

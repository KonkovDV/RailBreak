"""Sigma coverage and the empirical multiplier. No bag and no filter retune."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sigma_calibration as sc  # noqa: E402


class SigmaCalibrationTests(unittest.TestCase):
    def test_gaussian_matches_the_two_sided_table(self):
        self.assertAlmostEqual(sc.gaussian_coverage(1.0), math.erf(1.0 / math.sqrt(2.0)))
        self.assertAlmostEqual(sc.gaussian_coverage(1.0), 0.6827, places=4)
        self.assertAlmostEqual(sc.gaussian_coverage(2.0), 0.9545, places=4)
        self.assertAlmostEqual(sc.gaussian_coverage(3.0), 0.9973, places=4)

    def test_a_long_ride_does_not_set_the_quantile(self):
        long_ride = (np.ones(100), np.ones(100))
        short_ride = (np.full(2, 10.0), np.ones(2))
        pooled = sc.weighted_quantile(
            np.concatenate([np.ones(100), np.full(2, 10.0)]),
            np.ones(102),
            0.95,
        )
        by_ride = sc.equal_ride_quantile([long_ride, short_ride], 0.95)
        self.assertLess(pooled, 2.0)
        self.assertAlmostEqual(by_ride, 10.0)

    def test_worse_vehicle_sets_the_multiplier(self):
        sig = np.ones(4)
        calm = ("30618_a", np.ones(4), sig)
        wide = ("30639_b", np.full(4, 4.0), sig)
        report = sc.summarize([calm, wide])
        self.assertFalse(report["certification_claim"])
        self.assertFalse(report["hidden_test_used"])
        self.assertFalse(report["filter_changed"])
        self.assertAlmostEqual(report["c_0_95_by_vehicle"]["30618"], 1.0)
        self.assertAlmostEqual(report["c_0_95_by_vehicle"]["30639"], 4.0)
        self.assertAlmostEqual(report["c_0_95"], 4.0)
        self.assertAlmostEqual(report["coverage_equal_ride"]["1.0"], 0.5)

    def test_diagram_names_the_normal_law_and_refuses_a_certificate(self):
        svg = sc.reliability_svg([1.0, 2.0, 3.0], [0.62, 0.88, 0.95], [0.683, 0.954, 0.997])
        self.assertIn("нормальный", svg)
        self.assertIn("Не сертификат", svg)
        self.assertIn("<polyline", svg)


if __name__ == "__main__":
    unittest.main()

"""Trip splits, an independent truth generator, and fault-scenario metrics."""

from __future__ import annotations

import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from integrity import STATUSES  # noqa: E402
from scenario_campaign import (  # noqa: E402
    SCENARIOS,
    detection_metrics,
    run_scenario,
)
from trip_split import (  # noqa: E402
    ClosedTest,
    RowLeak,
    assert_no_row_leak,
    assign_trips,
    bind_rows,
    open_for_score,
)
from truth_sim import (  # noqa: E402
    TruthParams,
    TruthSample,
    brake_accel,
    laplace,
    simulate,
    traction_accel,
)


class TripSplitTests(unittest.TestCase):
    def test_whole_rides_stay_together(self):
        rows = []
        for k in range(6):
            rows.append({"ride_id": "A", "t": k * 0.02})
            rows.append({"ride_id": "B", "t": k * 0.02 + 0.01})
        assignment = assign_trips(["B", "A"])
        bound = bind_rows(rows, assignment)
        self.assertTrue(all(row["ride_id"] == "A" for row in bound["train"]))
        self.assertTrue(all(row["ride_id"] == "B" for row in bound["validation"]))
        self.assertEqual(bound["test"], [])

    def test_a_row_cut_is_rejected(self):
        rows = [{"ride_id": "A", "t": 0.0}, {"ride_id": "A", "t": 0.02}, {"ride_id": "B", "t": 0.01}]
        with self.assertRaises(RowLeak):
            assert_no_row_leak(rows, {"train": [0], "validation": [1], "test": [2]})

    def test_five_rides_hold_out_a_whole_test_ride(self):
        assignment = assign_trips(["r3", "r1", "r5", "r2", "r4"])
        self.assertEqual(assignment["test"], ["r5"])
        self.assertNotIn("r5", assignment["train"] + assignment["validation"])

    def test_closed_test_is_not_scored(self):
        assignment = assign_trips(["r1", "r2", "r3", "r4", "r5"])
        self.assertEqual(open_for_score("validation", assignment), assignment["validation"])
        with self.assertRaises(ClosedTest):
            open_for_score("test", assignment)


class TruthSimTests(unittest.TestCase):
    def test_module_does_not_import_the_filter_or_the_plant(self):
        text = (Path(__file__).resolve().parent / "truth_sim.py").read_text(encoding="utf-8")
        imports = [
            line.strip() for line in text.splitlines()
            if line.startswith("import ") or line.startswith("from ")
        ]
        joined = "\n".join(imports)
        for banned in ("odometer", "plant_ref", "track_odometer"):
            self.assertNotIn(banned, joined)

    def test_traction_is_not_a_hyperbola_and_brake_is_not_linear(self):
        plant_ratio = min(1.0, 6.0 / 6.0) / min(1.0, 6.0 / 12.0)
        truth_ratio = traction_accel(8, 6.0, 0.9, 6.0, 14.0) / traction_accel(8, 12.0, 0.9, 6.0, 14.0)
        self.assertGreater(abs(truth_ratio - plant_ratio), 0.2)
        linear = 15.0 / 7.0
        quad = brake_accel(-15, 0.35, 1.1) / brake_accel(-7, 0.35, 1.1)
        self.assertGreater(abs(quad - linear), 0.5)

    def test_lag_grade_mass_radii_and_slip_delay(self):
        params = TruthParams(
            grade_steps=((4.0, 0.0), (1.0e9, 0.02)),
            mass_steps=((4.0, 34000.0), (1.0e9, 27000.0)),
            outlier_every_s=100.0,
            dropout_on_s=100.0,
            dropout_off_s=100.0,
        )
        inputs, truth = simulate(12.0, 0.05, params)
        self.assertTrue(any(abs(row.z2) > 1.0e-4 for row in truth))
        early = truth[2]
        late = [row for row in truth if row.s > 4.0][-1]
        self.assertNotEqual(early.grade, late.grade)
        self.assertNotEqual(early.mass_kg, late.mass_kg)
        self.assertNotEqual(params.r_front_m, params.r_rear_m)
        rising = [row.slip for row in truth if row.z1 > 0.05]
        self.assertGreater(len(rising), 2)
        self.assertGreater(rising[-1], rising[0])
        self.assertTrue(any(math.isnan(sample.front_kmh) for sample in simulate(16.0, 0.1)[0]))
        self.assertTrue(any(sample.notch < 0 for sample in simulate(22.0, 0.1)[0]))
        self.assertFalse(hasattr(inputs[0], "mass_kg"))
        self.assertFalse(hasattr(TruthSample(0, 0, 0, 0, 0, 0, 0, 0, 0), "front_kmh"))

    def test_noise_is_laplace_not_only_gaussian(self):
        rng = random.Random(1)
        draws = [laplace(rng, 1.0) for _ in range(4000)]
        mean = sum(draws) / len(draws)
        m2 = sum((x - mean) ** 2 for x in draws) / len(draws)
        m4 = sum((x - mean) ** 4 for x in draws) / len(draws)
        excess = m4 / (m2 * m2) - 3.0
        self.assertGreater(excess, 1.5)

    def test_disturbance_changes_speed(self):
        base = TruthParams(disturb_mps2=0.0, seed=2, outlier_every_s=100.0, dropout_on_s=100.0, dropout_off_s=100.1)
        loud = TruthParams(disturb_mps2=0.4, seed=2, outlier_every_s=100.0, dropout_on_s=100.0, dropout_off_s=100.1)
        _, quiet = simulate(12.0, 0.1, base)
        _, loud_run = simulate(12.0, 0.1, loud)
        self.assertGreater(abs(loud_run[-1].v - quiet[-1].v), 0.05)


class MetricTests(unittest.TestCase):
    def test_rates_on_a_labelled_trace(self):
        rows = []
        for t, fault, alarm, lost, inside, conf in (
            (0.0, False, False, False, True, "HIGH"),
            (1.0, True, False, False, True, "HIGH"),
            (2.0, True, True, False, False, "HIGH"),
            (3.0, False, True, False, True, "LOW"),
            (4.0, False, False, True, False, "NONE"),
        ):
            rows.append({
                "t": t,
                "alarm": alarm,
                "lost": lost,
                "s_true": 0.0 if inside else 10.0,
                "s_hat": 0.0,
                "s_min": -1.0,
                "s_max": 1.0,
                "interval_valid": True,
                "position_confidence": conf,
            })
        m = detection_metrics(rows, 1.0, 2.0)
        self.assertAlmostEqual(m["false_alarm_rate"], 1.0 / 3.0)
        self.assertAlmostEqual(m["missed_detection_rate"], 0.5)
        self.assertAlmostEqual(m["time_to_detection"], 1.0)
        self.assertAlmostEqual(m["time_to_recovery"], 2.0)
        self.assertAlmostEqual(m["time_to_LOST"], 3.0)
        self.assertAlmostEqual(m["interval_coverage"], 3.0 / 5.0)
        self.assertFalse(m["claimed_percentile"])
        self.assertEqual(m["maximum_unsafe_confidence"], "HIGH")

    def test_no_fault_does_not_invent_a_miss_rate(self):
        rows = [{
            "t": 0.0, "alarm": False, "lost": False, "s_true": 0.0, "s_hat": 0.0,
            "s_min": -1.0, "s_max": 1.0, "interval_valid": True, "position_confidence": "HIGH",
        }]
        m = detection_metrics(rows, None, None)
        self.assertIsNone(m["missed_detection_rate"])
        self.assertIsNone(m["time_to_detection"])
        self.assertEqual(m["false_alarm_rate"], 0.0)
        self.assertEqual(m["maximum_unsafe_confidence"], "NONE")


class CampaignTests(unittest.TestCase):
    def test_every_scenario_returns_measured_fields(self):
        self.assertEqual(len(SCENARIOS), 19)
        for name in SCENARIOS:
            m = run_scenario(name, duration_s=8.0, dt=0.2)
            self.assertEqual(m["scenario"], name)
            self.assertGreater(m["n"], 5)
            self.assertFalse(m["claimed_percentile"])
            self.assertFalse(m["brake_field_sent"])
            self.assertIsInstance(m["rmse_m"], float)
            self.assertTrue(math.isfinite(m["rmse_m"]))
            if m["interval_coverage"] is not None:
                self.assertGreaterEqual(m["interval_coverage"], 0.0)
                self.assertLessEqual(m["interval_coverage"], 1.0)
        lost = run_scenario("missing_assets", duration_s=6.0, dt=0.2)
        self.assertIsNotNone(lost["time_to_LOST"])
        self.assertIn("LOST", STATUSES)
        self.assertEqual(lost["status"], "LOST")
        self.assertEqual(lost["integrity_status"], "POSITION_UNTRUSTED")


if __name__ == "__main__":
    unittest.main()

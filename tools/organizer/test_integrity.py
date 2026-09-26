"""Shadow integrity monitor: statuses, reasons, and the empirical bound."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integrity import (  # noqa: E402
    REASONS,
    STATUSES,
    BoundCoeff,
    IntegrityMonitor,
    Obs,
    coeff_path,
    load_coeff,
)


def obs(**kw) -> Obs:
    base = dict(
        t=10.0,
        sigma_s=2.0,
        front_age_s=0.05,
        rear_age_s=0.05,
        pair_fresh=True,
        bogies_agree=True,
        absolute_start=True,
        map_in_domain=True,
    )
    base.update(kw)
    return Obs(**base)


class IntegrityTests(unittest.TestCase):
    def test_names(self):
        self.assertEqual(len(STATUSES), 7)
        self.assertEqual(len(REASONS), 10)
        self.assertIn("POSITION_UNTRUSTED", STATUSES)
        self.assertIn("BOGIES_AGREE_MODEL_DISAGREES", REASONS)

    def test_nominal(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs())
        self.assertEqual(r.status, "NOMINAL")
        self.assertEqual(r.reasons, [])
        self.assertTrue(r.use_position)
        self.assertFalse(r.certification_claim)
        self.assertIsNone(r.along_bound_m)

    def test_single_bogie_nis(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(nis_front=20.0, slip_front=True))
        self.assertEqual(r.status, "DEGRADED_SINGLE_BOGIE")
        self.assertIn("FRONT_NIS_HIGH", r.reasons)
        self.assertNotIn("REAR_NIS_HIGH", r.reasons)

    def test_stale_one_side(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(rear_age_s=1.0, pair_fresh=False, bogies_agree=False))
        self.assertEqual(r.status, "DEGRADED_SINGLE_BOGIE")
        self.assertEqual(r.reasons, ["STALE_REAR"])

    def test_model_carry_when_both_stale(self):
        r = IntegrityMonitor(BoundCoeff()).update(
            obs(front_age_s=2.0, rear_age_s=2.0, pair_fresh=False, bogies_agree=False)
        )
        self.assertEqual(r.status, "DEGRADED_MODEL_CARRY")
        self.assertEqual(r.reasons, ["STALE_FRONT", "STALE_REAR"])

    def test_both_past_max_gap_is_a_refusal(self):
        r = IntegrityMonitor(BoundCoeff()).update(
            obs(front_age_s=31.0, rear_age_s=31.0, pair_fresh=False, bogies_agree=False)
        )
        self.assertEqual(r.status, "POSITION_UNTRUSTED")
        self.assertFalse(r.use_position)

    def test_stamp_regression(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(stamp_regressed=True))
        self.assertEqual(r.status, "POSITION_UNTRUSTED")
        self.assertIn("STAMP_REGRESSION", r.reasons)

    def test_relative_only(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(absolute_start=False))
        self.assertEqual(r.status, "DEGRADED_RELATIVE_ONLY")
        self.assertEqual(r.reasons, ["NO_ABSOLUTE_START"])
        self.assertTrue(r.use_position)

    def test_no_map(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(map_in_domain=False))
        self.assertEqual(r.status, "DEGRADED_NO_MAP")
        self.assertIn("MAP_OUT_OF_DOMAIN", r.reasons)

    def test_no_start_and_no_map_refuses(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(absolute_start=False, map_in_domain=False))
        self.assertEqual(r.status, "POSITION_UNTRUSTED")
        self.assertIn("NO_ABSOLUTE_START", r.reasons)
        self.assertIn("MAP_OUT_OF_DOMAIN", r.reasons)
        self.assertFalse(r.use_position)

    def test_disagree(self):
        r = IntegrityMonitor(BoundCoeff()).update(
            obs(bogies_agree=False, slip_front=True, nis_front=4.0)
        )
        self.assertEqual(r.status, "DEGRADED_SINGLE_BOGIE")
        self.assertIn("BOGIES_DISAGREE", r.reasons)
        self.assertNotIn("FRONT_NIS_HIGH", r.reasons)

    def test_common_mode_then_latch_until_disagree(self):
        mon = IntegrityMonitor(BoundCoeff())
        for t in (0.0, 1.0, 2.0):
            r = mon.update(obs(
                t=t, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0,
            ))
            self.assertEqual(r.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
            self.assertIn("BOGIES_AGREE_MODEL_DISAGREES", r.reasons)
        # The filter clears the slip bit once the episode has lasted recover_s.
        held = mon.update(obs(t=3.0, slip_front=False, slip_rear=False))
        self.assertEqual(held.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        still = mon.update(obs(t=4.0))
        self.assertEqual(still.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        cleared = mon.update(obs(t=5.0, bogies_agree=False, slip_front=True, nis_front=20.0))
        self.assertEqual(cleared.status, "DEGRADED_SINGLE_BOGIE")
        self.assertNotIn("BOGIES_AGREE_MODEL_DISAGREES", cleared.reasons)

    def test_short_common_mode_does_not_latch(self):
        mon = IntegrityMonitor(BoundCoeff())
        mon.update(obs(t=0.0, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0))
        later = mon.update(obs(t=0.2))
        self.assertEqual(later.status, "NOMINAL")

    def test_anchor_clears_the_latch(self):
        mon = IntegrityMonitor(BoundCoeff())
        mon.update(obs(t=0.0, slip_front=True, slip_rear=True, nis_front=30.0, nis_rear=30.0))
        mon.update(obs(t=3.0))
        self.assertTrue(mon.latched)
        cleared = mon.update(obs(t=4.0, n_anchor=1))
        self.assertEqual(cleared.status, "NOMINAL")
        self.assertFalse(mon.latched)

    def test_ambiguous_station_is_a_reason_not_a_refusal(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(dwell=True, station_candidates=2))
        self.assertEqual(r.status, "NOMINAL")
        self.assertEqual(r.reasons, ["AMBIGUOUS_STATION_ANCHOR"])
        self.assertTrue(r.use_position)

    def test_bound_adds_the_three_margins(self):
        coeff = BoundCoeff(
            calibrated=True,
            q99=2.5,
            b_mode={"DEGRADED_SINGLE_BOGIE": 4.0, "DEGRADED_NO_MAP": 1.0},
            b_time_m=3.0,
            b_map_m=0.5,
            coverage="0.99",
            split="train",
        )
        fresh = IntegrityMonitor(coeff).update(obs(nis_front=20.0, slip_front=True, sigma_s=2.0))
        self.assertAlmostEqual(fresh.along_bound_m, 2.5 * 2.0 + 4.0)
        self.assertEqual(fresh.calibrated_coverage, "0.99")
        self.assertEqual(fresh.calibration_split, "train")
        self.assertFalse(fresh.certification_claim)
        stale = IntegrityMonitor(coeff).update(obs(
            nis_front=20.0, slip_front=True, sigma_s=2.0, front_age_s=2.0, rear_age_s=2.0,
            pair_fresh=False, bogies_agree=False,
        ))
        # Both stale, under the gap limit: model carry, plus the dropout term.
        self.assertEqual(stale.status, "DEGRADED_MODEL_CARRY")
        self.assertAlmostEqual(stale.along_bound_m, 2.5 * 2.0 + 3.0)
        off = IntegrityMonitor(coeff).update(obs(map_in_domain=False, sigma_s=2.0))
        self.assertAlmostEqual(off.along_bound_m, 2.5 * 2.0 + 1.0 + 0.5)

    def test_shipped_coefficients_are_not_a_certificate(self):
        path = coeff_path()
        if not path.is_file():
            self.skipTest("coefficients are not written yet")
        raw = load_coeff(path)
        self.assertTrue(raw.calibrated)
        self.assertGreater(raw.q99, 0.0)
        self.assertEqual(raw.split, "train")
        self.assertEqual(raw.coverage, "0.99")
        text = path.read_text(encoding="utf-8")
        self.assertIn('"certification_claim": false', text)
        header = path.parents[1] / "include" / "railbreak_backup_odometry" / "integrity_bound.hpp"
        hpp = header.read_text(encoding="utf-8")
        self.assertIn("c.calibrated = true", hpp)
        self.assertIn(f"c.q99 = {raw.q99:.6f}", hpp)
        self.assertNotIn("SIL", hpp)


if __name__ == "__main__":
    unittest.main()

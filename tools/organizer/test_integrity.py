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


def soak(mon: IntegrityMonitor, t1: float, **kw):
    t0 = float(kw.pop("t", 0.0))
    last = None
    n = int(round((t1 - t0) / 0.1))
    for i in range(n + 1):
        last = mon.update(obs(t=t0 + 0.1 * i, **kw))
    return last


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
        self.assertEqual(len(STATUSES), 8)
        self.assertEqual(len(REASONS), 13)
        self.assertIn("LOST", STATUSES)
        self.assertIn("POSITION_UNTRUSTED", STATUSES)
        self.assertEqual(
            IntegrityMonitor(BoundCoeff()).update(obs()).bound_statement,
            "empirical bound, not certified protection level",
        )
        self.assertIn("BOGIES_AGREE_MODEL_DISAGREES", REASONS)

    def test_rear_does_not_borrow_the_front_timer(self):
        from integrity import FaultScore

        def run(nis_rear_from):
            score = FaultScore()
            seen = []
            for i in range(21):
                t = 0.1 * i
                seen.append(score.update(obs(
                    t=t, pair_fresh=True, bogies_agree=True,
                    nis_front=20.0, nis_rear=20.0 if t + 1e-12 >= nis_rear_from else 0.0,
                )))
            return seen

        clean = FaultScore()
        last = None
        for i in range(21):
            last = clean.update(obs(t=0.1 * i, pair_fresh=True, bogies_agree=True))
        self.assertFalse(last["front_latched"] or last["rear_latched"] or last["common_latched"])
        single = run(99.0)
        self.assertTrue(single[-1]["front_latched"])
        self.assertFalse(single[-1]["rear_latched"])
        stagger = run(0.6)
        early = [row for row in stagger if row["rear_latched"]]
        self.assertTrue(stagger[10]["front_latched"])
        self.assertFalse(stagger[10]["rear_latched"])
        self.assertFalse(any(row["rear_latched"] for i, row in enumerate(stagger) if i * 0.1 < 1.1))
        self.assertFalse(early and min(i * 0.1 for i, row in enumerate(stagger) if row["rear_latched"]) < 1.1)

    def test_nominal(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs())
        self.assertEqual(r.status, "NOMINAL")
        self.assertEqual(r.reasons, [])
        self.assertTrue(r.use_position)
        self.assertFalse(r.certification_claim)
        self.assertIsNone(r.along_bound_m)

    def test_single_bogie_nis(self):
        mon = IntegrityMonitor(BoundCoeff())
        one = mon.update(obs(t=10.0, nis_front=20.0, slip_front=True))
        self.assertEqual(one.status, "NOMINAL")
        self.assertIn("FRONT_NIS_HIGH", one.reasons)
        early = soak(
            IntegrityMonitor(BoundCoeff()), 10.4, t=10.0, nis_front=20.0, slip_front=True,
        )
        self.assertEqual(early.status, "NOMINAL")
        held = soak(mon, 11.0, t=10.0, nis_front=20.0, slip_front=True)
        self.assertEqual(held.status, "DEGRADED_SINGLE_BOGIE")
        self.assertNotIn("REAR_NIS_HIGH", held.reasons)

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
        self.assertEqual(r.integrity_mode, "LOST")

    def test_stamp_regression(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(stamp_regressed=True))
        self.assertEqual(r.status, "POSITION_UNTRUSTED")
        self.assertIn("STAMP_REGRESSION", r.reasons)

    def test_relative_only(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(absolute_start=False))
        self.assertEqual(r.status, "DEGRADED_RELATIVE_ONLY")
        self.assertEqual(r.reasons, ["NO_ABSOLUTE_START"])
        self.assertEqual(r.confidence_position, "degraded")
        self.assertEqual(r.position_confidence, "LOW")
        self.assertTrue(r.use_position)

    def test_no_map(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(map_in_domain=False))
        self.assertEqual(r.status, "DEGRADED_NO_MAP")
        self.assertIn("MAP_OUT_OF_DOMAIN", r.reasons)
        self.assertEqual(r.confidence_position, "degraded")
        self.assertEqual(r.position_confidence, "LOW")

    def test_no_start_and_no_map_refuses(self):
        r = IntegrityMonitor(BoundCoeff()).update(obs(absolute_start=False, map_in_domain=False))
        self.assertEqual(r.status, "POSITION_UNTRUSTED")
        self.assertIn("NO_ABSOLUTE_START", r.reasons)
        self.assertIn("MAP_OUT_OF_DOMAIN", r.reasons)
        self.assertFalse(r.use_position)

    def test_disagree(self):
        mon = IntegrityMonitor(BoundCoeff())
        one = mon.update(obs(t=0.0, bogies_agree=False, slip_front=True, nis_front=4.0))
        self.assertEqual(one.status, "NOMINAL")
        self.assertIn("BOGIES_DISAGREE", one.reasons)
        held = soak(mon, 1.0, bogies_agree=False, slip_front=True, nis_front=4.0)
        self.assertEqual(held.status, "DEGRADED_SINGLE_BOGIE")
        self.assertNotIn("FRONT_NIS_HIGH", held.reasons)

    def test_common_mode_then_latch_until_disagree(self):
        mon = IntegrityMonitor(BoundCoeff())
        during = soak(
            mon, 4.5, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0,
        )
        self.assertEqual(during.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        self.assertIn("BOGIES_AGREE_MODEL_DISAGREES", during.reasons)
        quiet = soak(mon, 8.5, t=4.5)
        self.assertEqual(quiet.status, "NOMINAL")
        self.assertEqual(quiet.confidence_velocity, "ok")
        self.assertEqual(quiet.confidence_position, "degraded")
        self.assertEqual(quiet.integrity_mode, "POSITION_DEGRADED")
        self.assertIn("POSITION_OPEN", quiet.reasons)

    def test_micro_slip_does_not_hold_position(self):
        mon = IntegrityMonitor(BoundCoeff())
        slipped = mon.update(obs(t=0.0, slip_front=True, nis_front=20.0))
        self.assertEqual(slipped.status, "NOMINAL")
        self.assertEqual(slipped.confidence_velocity, "ok")
        self.assertEqual(slipped.confidence_position, "ok")
        self.assertFalse(mon.latched)
        back = mon.update(obs(t=0.2))
        self.assertEqual(back.status, "NOMINAL")
        self.assertEqual(back.confidence_velocity, "ok")
        self.assertEqual(back.confidence_position, "ok")

    def test_position_margin_stays_after_velocity_recovers(self):
        coeff = BoundCoeff(
            calibrated=True,
            q99=2.0,
            b_mode={"NOMINAL": 0.0, "DEGRADED_COMMON_MODE_UNOBSERVABLE": 32.606},
        )
        mon = IntegrityMonitor(coeff)
        soak(mon, 4.5, sigma_s=1.0, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0)
        held = soak(mon, 8.5, t=4.5, sigma_s=1.0)
        self.assertEqual(held.confidence_velocity, "ok")
        self.assertEqual(held.confidence_position, "degraded")
        self.assertAlmostEqual(held.along_bound_m, 34.606)

    def test_short_common_mode_does_not_latch(self):
        mon = IntegrityMonitor(BoundCoeff())
        mon.update(obs(t=0.0, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0))
        later = mon.update(obs(t=0.2))
        self.assertEqual(later.status, "NOMINAL")

    def test_degraded_waits_and_recovers(self):
        mon = IntegrityMonitor(BoundCoeff())
        early = mon.update(obs(t=0.0, slip_front=True, nis_front=20.0))
        self.assertEqual(early.status, "NOMINAL")
        late = soak(mon, 1.0, slip_front=True, nis_front=20.0)
        self.assertEqual(late.status, "DEGRADED_SINGLE_BOGIE")
        self.assertEqual(late.fault_level, "degraded")
        quiet = mon.update(obs(t=1.1))
        self.assertEqual(quiet.status, "DEGRADED_SINGLE_BOGIE")
        clear = soak(mon, 5.0, t=1.1)
        self.assertEqual(clear.status, "NOMINAL")
        self.assertEqual(clear.fault_level, "nominal")

    def test_anchor_clears_the_latch(self):
        mon = IntegrityMonitor(BoundCoeff())
        soak(mon, 4.5, slip_front=True, slip_rear=True, nis_front=30.0, nis_rear=30.0)
        self.assertTrue(mon.latched)
        cleared = mon.update(obs(t=4.6, n_anchor=1, nis_front=30.0, nis_rear=30.0,
                                 slip_front=True, slip_rear=True))
        self.assertFalse(mon.latched)
        self.assertTrue(cleared.use_position)

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
        bound_mon = IntegrityMonitor(coeff)
        fresh = soak(bound_mon, 11.0, t=10.0, nis_front=20.0, slip_front=True, sigma_s=2.0)
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

    def test_blind_budget_is_from_the_last_anchor(self):
        mon = IntegrityMonitor(BoundCoeff())
        inside = soak(
            mon, 1.0, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0,
            distance_since_anchor=10.0,
        )
        self.assertEqual(inside.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        self.assertEqual(inside.integrity_mode, "VELOCITY_DEGRADED")
        self.assertTrue(inside.use_position)
        self.assertGreater(inside.time_to_lost, 0.0)
        self.assertEqual(inside.blind_warning, "")
        far = mon.update(obs(
            t=1.1, slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0,
            distance_since_anchor=150.0,
        ))
        self.assertEqual(far.status, "LOST")
        self.assertIn(far.status, STATUSES)
        self.assertFalse(far.use_position)
        self.assertEqual(far.time_to_lost, 0.0)
        self.assertAlmostEqual(far.distance_since_last_trusted_anchor, 150.0)
        self.assertEqual(far.velocity_confidence, "LOW")
        self.assertEqual(far.position_confidence, "NONE")
        self.assertEqual(far.fault_level, "lost")
        self.assertIn("BLIND_BUDGET", far.reasons)
        stuck = IntegrityMonitor(BoundCoeff())
        opened = stuck.update(obs(
            t=3.0, slip_front=True, slip_rear=True, common_unobservable=True,
            distance_since_anchor=10.0,
        ))
        self.assertEqual(opened.status, "DEGRADED_COMMON_MODE_UNOBSERVABLE")
        self.assertEqual(opened.integrity_mode, "VELOCITY_DEGRADED")
        self.assertEqual(opened.velocity_confidence, "LOW")
        self.assertEqual(opened.position_confidence, "LOW")
        self.assertEqual(opened.time_to_lost, 5.0)
        self.assertTrue(opened.use_position)
        self.assertEqual(opened.blind_time_s, 0.0)
        refused = stuck.update(obs(
            t=9.0, slip_front=True, slip_rear=True, common_unobservable=True,
            distance_since_anchor=10.0,
        ))
        self.assertEqual(refused.status, "LOST")
        self.assertEqual(refused.integrity_mode, "LOST")
        self.assertEqual(refused.velocity_confidence, "LOW")
        self.assertEqual(refused.position_confidence, "NONE")
        self.assertFalse(refused.use_position)
        self.assertEqual(refused.time_to_lost, 0.0)
        self.assertFalse(refused.bound_valid)
        lone = IntegrityMonitor(BoundCoeff())
        one = soak(
            lone, 2.0, t=1.0, slip_front=True, slip_rear=False, nis_front=20.0,
            bogies_agree=False, distance_since_anchor=500.0,
        )
        self.assertEqual(one.integrity_mode, "WHEEL_DEGRADED")
        self.assertNotEqual(one.fault_level, "lost")
        reset = mon.update(obs(
            t=1.2, n_anchor=1, distance_since_anchor=0.0,
            slip_front=True, slip_rear=True, nis_front=20.0, nis_rear=20.0,
        ))
        self.assertNotEqual(reset.integrity_mode, "LOST")
        self.assertTrue(reset.use_position)

    def test_ring_path_does_not_fold_after_a_lap(self):
        import numpy as np
        from odometer import Odometer, Params

        n = 102
        s = np.arange(n, dtype=float)
        od = Odometer(
            s, np.zeros(n), np.zeros((31, 18)), np.arange(-15, 16),
            np.arange(18) + 0.5, [], Params(), ring_len=101.0,
        )
        od.init(0.0, 1.0)
        od.t = 0.0
        for i in range(110):
            t = 0.1 * i
            od.on_cmd(t + 0.025, 0)
            od.on_bogie(t, "front", 36.0)
            od.on_bogie(t + 0.05, "rear", 36.0)
        self.assertGreater(od.distance_since_anchor(), 100.0)
        self.assertLess(od.state()[0], 20.0)

    def test_gnss_anchor_moves_s_like_a_station(self):
        import numpy as np
        from odometer import Odometer, Params

        n = 400
        s = np.arange(n, dtype=float)
        od = Odometer(
            s, np.zeros(n), np.zeros((31, 18)), np.arange(-15, 16),
            np.arange(18) + 0.5, [], Params(), ring_len=float(n - 1),
        )
        od.init(0.0, 1.0)
        od.t = 0.0
        for i in range(50):
            t = 0.1 * i
            od.on_cmd(t + 0.025, 0)
            od.on_bogie(t, "front", 36.0)
            od.on_bogie(t + 0.05, "rear", 36.0)
        before = od.state()[0]
        self.assertFalse(od.gnss_anchor(before + 8.0, 0.0))
        self.assertTrue(od.gnss_anchor(before + 8.0, 2.0))
        self.assertEqual(od.n_gnss_anchor, 1)
        self.assertGreater(od.state()[0], before + 1.0)
        self.assertLess(od.state()[0], before + 8.0)
        self.assertLess(od.distance_since_anchor(), 1.0)


if __name__ == "__main__":
    unittest.main()

"""Adhesion proxy labels. No bag, no mu, no filter retune."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import adhesion  # noqa: E402


def obs(**kw) -> adhesion.AdhesionObs:
    base = dict(
        t=10.0,
        pair_fresh=True,
        bogies_agree=True,
        have_consensus=True,
        wheel_consensus_residual=0.05,
        have_model=True,
        model_consistency_residual=-0.02,
        front_nis=1.0,
        rear_nis=1.2,
        notch=4,
        speed=8.5,
    )
    base.update(kw)
    return adhesion.AdhesionObs(**base)


class AdhesionProxyTests(unittest.TestCase):
    def test_mu_stays_null_when_the_pattern_is_common(self):
        proxy = adhesion.AdhesionProxy()
        first = proxy.update(obs(t=10.0, slip_front=True, slip_rear=True, front_nis=40.0, rear_nis=36.0,
                                  model_consistency_residual=-2.4))
        second = proxy.update(obs(t=11.7, slip_front=True, slip_rear=True, front_nis=40.0, rear_nis=36.0,
                                   model_consistency_residual=-2.4))
        self.assertEqual(second.classification, "COMMON_MODE_SUSPECTED")
        self.assertIsNone(second.mu_estimate)
        self.assertFalse(second.mu_observable)
        self.assertEqual(second.reason, "both bogies agree but differ from model")
        self.assertAlmostEqual(first.duration_s, 0.0)
        self.assertAlmostEqual(second.duration_s, 1.7)
        self.assertAlmostEqual(second.common_mode_duration_s, 1.7)
        payload = json.loads(second.json())
        self.assertIsNone(payload["mu_estimate"])
        self.assertEqual(payload["notch"], 4)
        self.assertAlmostEqual(payload["speed"], 8.5)
        self.assertAlmostEqual(payload["wheel_consensus_residual"], 0.05)
        self.assertAlmostEqual(payload["model_consistency_residual"], -2.4)

    def test_a_break_in_the_episode_resets_the_clock(self):
        proxy = adhesion.AdhesionProxy()
        proxy.update(obs(t=10.0, slip_front=True, slip_rear=True))
        proxy.update(obs(t=11.0, slip_front=True, slip_rear=True))
        quiet = proxy.update(obs(t=11.1, slip_front=False, slip_rear=False))
        again = proxy.update(obs(t=12.0, slip_front=True, slip_rear=True))
        self.assertEqual(quiet.classification, "CONSISTENT")
        self.assertAlmostEqual(quiet.duration_s, 0.0)
        self.assertAlmostEqual(again.duration_s, 0.0)

    def test_disagreement_is_not_called_adhesion(self):
        report = adhesion.AdhesionProxy().update(
            obs(bogies_agree=False, slip_front=True, slip_rear=False, wheel_consensus_residual=1.4)
        )
        self.assertEqual(report.classification, "BOGIES_DISAGREE")
        self.assertIsNone(report.mu_estimate)
        self.assertAlmostEqual(report.common_mode_duration_s, 0.0)

    def test_one_bogie_stays_a_single_anomaly(self):
        report = adhesion.AdhesionProxy().update(obs(slip_front=True, slip_rear=False, front_nis=20.0))
        self.assertEqual(report.classification, "SINGLE_BOGIE_ANOMALY")
        self.assertIsNone(report.mu_estimate)

    def test_missing_pair_publishes_null_residuals_as_absent(self):
        report = adhesion.AdhesionProxy().update(
            adhesion.AdhesionObs(t=1.0, pair_fresh=False, have_consensus=False, have_model=False)
        )
        self.assertEqual(report.classification, "NO_FRESH_PAIR")
        payload = json.loads(report.json())
        self.assertIsNone(payload["wheel_consensus_residual"])
        self.assertIsNone(payload["model_consistency_residual"])
        self.assertIsNone(payload["mu_estimate"])

    def test_module_does_not_import_the_filter(self):
        text = Path(adhesion.__file__).read_text(encoding="utf-8")
        self.assertNotIn("import odometer", text)
        self.assertNotIn("track_odometer", text)


if __name__ == "__main__":
    unittest.main()

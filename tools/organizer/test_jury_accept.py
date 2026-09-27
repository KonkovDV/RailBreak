"""Fail-closed acceptance reasons. No bag and no Docker."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jury_accept as ja  # noqa: E402


def clean(**kw) -> dict:
    stamps = [i / 20.0 for i in range(40)]
    obs = {
        "node_alive": True,
        "have_velocity": True,
        "have_position": True,
        "require_position": True,
        "velocity_stamps": stamps,
        "position_stamps": list(stamps),
        "position_finite_xyz": True,
        "scorer_status": 0,
        "frame_id": "map",
        "child_frame_id": "base_link",
        "twist_linear_x": 1.5,
        "gnss_closed": True,
        "db3_in_git": [],
    }
    obs.update(kw)
    return obs


class JuryAcceptTests(unittest.TestCase):
    def test_a_complete_observation_passes(self):
        self.assertEqual(ja.acceptance_failures(clean()), [])

    def test_each_broken_check_is_a_reason(self):
        cases = [
            ({"node_alive": False}, "node died"),
            ({"have_velocity": False}, "no /result/velocity"),
            ({"have_position": False}, "no /result/position"),
            ({"velocity_stamps": [0.2, 0.1, 0.3]}, "stamp regressed"),
            ({"velocity_stamps": [0.0, 0.2]}, "frequency"),
            ({"velocity_stamps": [0.0, 0.05, 0.40]}, "max gap"),
            ({"scorer_status": 2}, "scorer failed"),
            ({"frame_id": "odom"}, "not map"),
            ({"child_frame_id": "base_footprint"}, "not base_link"),
            ({"twist_linear_x": None}, "twist.linear.x"),
            ({"twist_linear_x": math.nan}, "twist.linear.x"),
            ({"gnss_closed": False}, "GNSS"),
            ({"db3_in_git": ["C:/repo/out.db3"]}, ".db3"),
            ({"position_stamps": [0.2, 0.1, 0.3]}, "position stamp regressed"),
            ({"position_stamps": [0.0, 0.2]}, "position frequency"),
            ({"position_stamps": [0.0, 0.05, 0.40]}, "position max gap"),
            ({"position_finite_xyz": False}, "not finite"),
        ]
        for patch, needle in cases:
            reasons = ja.acceptance_failures(clean(**patch))
            self.assertTrue(any(needle in reason for reason in reasons), reasons)

    def test_position_is_optional_only_when_the_scenario_says_so(self):
        self.assertEqual(ja.acceptance_failures(clean(have_position=False, require_position=False)), [])

    def test_a_new_db3_outside_the_record_directory_fails(self):
        root = Path("C:/repo")
        before = [str(root / "old.db3")]
        after = [
            str(root / "old.db3"),
            str(root / "jury_out" / "result" / "result_0.db3"),
            str(root / "src" / "leak.db3"),
        ]
        extra = ja.unexpected_db3(before, after, [root / "jury_out"])
        self.assertEqual(extra, [str((root / "src" / "leak.db3").resolve())])


if __name__ == "__main__":
    unittest.main()

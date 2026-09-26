"""The hypothesis bank is an offline prototype. It does not call the ring filter."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import path_hypotheses as ph  # noqa: E402


class PathHypothesisTests(unittest.TestCase):
    def test_weight_is_the_gaussian_factor(self):
        got = ph.unnormalized_weight(0.4, [2.0, 0.0], np.eye(2))
        self.assertAlmostEqual(got, 0.4 * math.exp(-2.0))

    def test_normalize_keeps_equal_weights_equal(self):
        w = ph.normalize([1.0, 1.0])
        self.assertTrue(np.allclose(w, [0.5, 0.5]))

    def test_predict_moves_s_and_keeps_the_weight(self):
        nxt = ph.predict(ph.Hypothesis("main", 3.0, 10.0, 0.25), 0.5)
        self.assertEqual(nxt.branch, "main")
        self.assertAlmostEqual(nxt.s, 8.0)
        self.assertAlmostEqual(nxt.v, 10.0)
        self.assertAlmostEqual(nxt.w, 0.25)

    def test_shared_stem_keeps_both_hypotheses(self):
        demo = ph.fork_demo()
        self.assertFalse(demo["default_filter"])
        for row in demo["rows"]:
            self.assertAlmostEqual(row["w_main"] + row["w_side"], 1.0)
            if row["s_m"] <= ph.STEM_M:
                self.assertAlmostEqual(row["w_main"], 0.5)
                self.assertAlmostEqual(row["w_side"], 0.5)
                self.assertAlmostEqual(row["residual_side_m"], 0.0)

    def test_after_the_fork_the_straight_branch_takes_the_weight(self):
        demo = ph.fork_demo()
        end = demo["rows"][-1]
        self.assertGreater(end["s_m"], ph.STEM_M)
        self.assertGreater(end["w_main"], 0.99)
        self.assertLess(end["w_side"], 0.01)
        past = [r for r in demo["rows"] if r["s_m"] > ph.STEM_M]
        self.assertGreater(past[-1]["w_main"], past[0]["w_main"])

    def test_module_does_not_import_the_ring_filter(self):
        self.assertNotIn("odometer", sys.modules)
        text = Path(ph.__file__).read_text(encoding="utf-8")
        self.assertNotIn("import odometer", text)
        self.assertNotIn("track_odometer", text)


if __name__ == "__main__":
    unittest.main()

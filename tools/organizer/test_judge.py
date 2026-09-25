import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from judge import pair, score  # noqa: E402


class JudgeTests(unittest.TestCase):
    def test_identity_is_zero(self) -> None:
        t = np.arange(0.0, 10.0, 0.1)
        ref = {"t": t, "x": t, "y": 0.1 * t, "z": np.zeros_like(t), "v": np.ones_like(t)}
        got = score(ref, ref)
        self.assertEqual(got["coverage"], 1.0)
        self.assertAlmostEqual(got["rmse_3d"], 0.0, places=9)
        self.assertAlmostEqual(got["rmse_v"], 0.0, places=9)

    def test_tolerance_drops_far_stamps(self) -> None:
        idx = pair(np.array([0.0, 1.0]), np.array([0.0, 0.2, 1.04]))
        self.assertEqual(idx.tolist(), [0, -1, 1])

    def test_known_offset(self) -> None:
        t = np.array([0.0, 1.0, 2.0])
        ref = {"t": t, "x": np.zeros(3), "y": np.zeros(3), "z": np.zeros(3), "v": np.zeros(3)}
        est = {"t": t, "x": np.full(3, 3.0), "y": np.full(3, 4.0), "z": np.zeros(3), "v": np.ones(3)}
        got = score(est, ref)
        self.assertAlmostEqual(got["rmse_3d"], 5.0, places=9)
        self.assertAlmostEqual(got["rmse_v"], 1.0, places=9)


if __name__ == "__main__":
    unittest.main()

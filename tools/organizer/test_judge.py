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

    def test_one_estimate_covers_a_faster_reference(self) -> None:
        # Output 5 Hz, reference 20 Hz, tolerance 50 ms. A one-to-one pair
        # would cover at most 25 %. Nearest-stamp pairing has no used-mask.
        t_est = np.arange(0.0, 10.0, 0.2)
        t_ref = np.arange(0.0, 10.0, 0.05)
        idx = pair(t_est, t_ref, 0.05)
        paired = idx[idx >= 0]
        counts = np.bincount(paired)
        self.assertEqual(paired.size, 108)
        self.assertAlmostEqual(paired.size / t_ref.size, 0.54)
        self.assertGreater(paired.size / t_ref.size, 2.0 * (t_est.size / t_ref.size))
        self.assertEqual(int(counts.max()), 3)

        def zeros(t: np.ndarray) -> dict:
            z = np.zeros_like(t)
            return {"t": t, "x": z, "y": z.copy(), "z": z.copy(), "v": z.copy()}

        bad = int(np.argmax(counts))
        est = zeros(t_est)
        est["x"] = est["x"].copy()
        est["x"][bad] = 10.0
        aligned = score(est, zeros(t_est))
        fast = score(est, zeros(t_ref))
        self.assertAlmostEqual(aligned["rmse_3d"], np.sqrt(100.0 / 50.0), places=9)
        self.assertAlmostEqual(fast["rmse_3d"], np.sqrt(300.0 / 108.0), places=9)
        self.assertGreater(fast["rmse_3d"], aligned["rmse_3d"])

        extras = np.array([t_est[bad] - 0.04, t_est[bad] + 0.04])
        clustered = score(est, zeros(np.sort(np.concatenate([t_est, extras]))))
        self.assertEqual(clustered["n_pair"], 52)
        self.assertAlmostEqual(clustered["rmse_3d"], np.sqrt(300.0 / 52.0), places=9)

    def test_known_offset(self) -> None:
        t = np.array([0.0, 1.0, 2.0])
        ref = {"t": t, "x": np.zeros(3), "y": np.zeros(3), "z": np.zeros(3), "v": np.zeros(3)}
        est = {"t": t, "x": np.full(3, 3.0), "y": np.full(3, 4.0), "z": np.zeros(3), "v": np.ones(3)}
        got = score(est, ref)
        self.assertAlmostEqual(got["rmse_3d"], 5.0, places=9)
        self.assertAlmostEqual(got["rmse_v"], 1.0, places=9)


if __name__ == "__main__":
    unittest.main()

"""Regression contracts for HMI exposure and GT pairing; no UKF import."""
from __future__ import annotations

import csv
import io
import json
import math
import random
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from check_envelope import check_rows, main as check_main

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "synth"))
from score import _fmt, _pair_ukf_gt, metrics, score_run


def estimate(confidence: str = "OK", t: float = 0.0, s: float = 0.0) -> dict:
    return dict(kind="est", confidence=confidence, t=t, s=s, v=0.0, p_ss=1.0, p_vv=0.1)


def gt(t: float = 0.0, s: float = 0.0) -> dict:
    return dict(kind="gt", t=t, s=s)


class HmiExposureTests(unittest.TestCase):
    def assert_no_exposure(self, result) -> None:
        self.assertEqual((result.n_ok, result.n_hmi), (0, 0))
        self.assertIn("HMI-rate=N/A (0/0; no matched OK exposure)", result.notes)
        self.assertFalse(any("HMI-rate=0.000000" in n for n in result.notes))

    def test_degraded_only_is_not_zero_risk(self) -> None:
        result = check_rows([gt(), estimate("DEGRADED")], require_gt=True)
        self.assertEqual(result.hits, [])
        self.assert_no_exposure(result)

    def test_lost_and_uninitialized_are_not_zero_risk(self) -> None:
        for confidence in ("LOST", "UNINITIALIZED"):
            with self.subTest(confidence=confidence):
                result = check_rows([gt(), estimate(confidence)], require_gt=True)
                self.assert_no_exposure(result)

    def test_unmatched_ok_is_dirty_and_not_zero_risk(self) -> None:
        result = check_rows([gt(), estimate(t=10.0)], require_gt=True)
        self.assertTrue(any("UNMATCHED_GT" in h for h in result.hits))
        self.assert_no_exposure(result)

    def test_invalid_path_is_dirty_and_not_zero_risk(self) -> None:
        result = check_rows([gt(), estimate(s=float("nan"))], require_gt=True)
        self.assertTrue(any("INVALID_STATE" in h for h in result.hits))
        self.assert_no_exposure(result)

    def test_no_estimates_is_dirty_and_not_zero_risk(self) -> None:
        result = check_rows([gt()], require_gt=True)
        self.assertIn("NO_ESTIMATE", result.hits)
        self.assert_no_exposure(result)

    def test_matched_ok_retains_numeric_rate(self) -> None:
        result = check_rows([gt(), estimate(), estimate(s=100.0)], require_gt=True)
        self.assertEqual((result.n_ok, result.n_hmi), (2, 1))
        self.assertIn("HMI-rate=0.500000 (1/2)", result.notes)

    def test_matched_zero_is_a_defined_zero(self) -> None:
        result = check_rows([gt(), estimate()], require_gt=True)
        self.assertIn("HMI-rate=0.000000 (0/1)", result.notes)

    def test_no_gt_still_skips_instead_of_inventing_rate(self) -> None:
        result = check_rows([estimate()], require_gt=False)
        self.assertIn("ENVELOPE_GT skipped: no GT", result.notes)
        self.assertFalse(any(n.startswith("HMI-rate=") for n in result.notes))

    def test_required_gt_is_still_usage_error(self) -> None:
        result = check_rows([estimate()], require_gt=True)
        self.assertTrue(any(h.startswith("usage:") for h in result.hits))

    def test_cli_keeps_exit_status_and_prefix_contract(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "input.jsonl"
            path.write_text("\n".join(map(json.dumps, [gt(), estimate("DEGRADED")])) + "\n", encoding="utf-8")
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                rc = check_main([str(path), "--require-gt"])
            self.assertEqual(rc, 0)  # No contract violation; not a measured zero HMI.
            self.assertEqual(stderr.getvalue(), "")
            lines = stdout.getvalue().splitlines()
            forwarded = [line for line in lines if line.startswith("HMI-rate")]
            self.assertEqual(forwarded, ["HMI-rate=N/A (0/0; no matched OK exposure)"])
            self.assertIn("checker empty", lines)


class ScoreExposureTests(unittest.TestCase):
    """Small recording fixtures test joins/counts, not estimator accuracy."""

    def run_fixture(self, estimates, times=(0.0, 0.02, 0.04, 0.06)) -> dict:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            with (root / "run.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(["t_s", "gt_s", "gt_v", "notch", "brake", "w0"])
                for t in times:
                    writer.writerow([t, 10.0 * t, 10.0, 0.0, 0.0, 10.0 / 0.35])
            if estimates is not None:
                (root / "ukf.jsonl").write_text(
                    "".join(json.dumps(row) + "\n" for row in estimates), encoding="utf-8"
                )
            return score_run(root)

    @staticmethod
    def sample(t: float, s: float | None = None) -> dict:
        return dict(t=t, s=10.0 * t if s is None else s, v=10.0, p_ss=1.0)

    def test_partial_coverage_uses_original_gt_denominator(self) -> None:
        block = self.run_fixture([self.sample(0.0), self.sample(0.04)])["ukf"]
        self.assertEqual((block["n"], block["n_hat"], block["n_gt"]), (2, 2, 4))
        self.assertEqual(block.get("n_gt_matched"), 2)
        self.assertEqual(block["coverage"], 0.5)
        self.assertEqual(block.get("estimate_coverage"), 1.0)

    def test_unmatched_estimates_stay_in_original_count(self) -> None:
        block = self.run_fixture([self.sample(0.0), self.sample(100.0)])["ukf"]
        self.assertEqual((block["n"], block["n_hat"], block["n_gt"]), (1, 2, 4))
        self.assertEqual(block["coverage"], 0.25)
        self.assertEqual(block.get("estimate_coverage"), 0.5)

    def test_reusing_gt_cannot_inflate_reference_coverage(self) -> None:
        block = self.run_fixture([self.sample(0.0), self.sample(0.001)])["ukf"]
        self.assertEqual(block["n"], 2)
        self.assertEqual(block["coverage"], 0.25)
        self.assertEqual(block.get("n_gt_matched"), 1)

    def test_all_unmatched_is_visible(self) -> None:
        out = self.run_fixture([self.sample(100.0)])
        self.assertIn("ukf", out)
        block = out["ukf"]
        self.assertEqual((block["n"], block["n_hat"], block["n_gt"]), (0, 1, 4))
        self.assertEqual(block["coverage"], 0.0)
        self.assertEqual(block.get("estimate_coverage"), 0.0)
        self.assertTrue(math.isnan(block["env_s"]))

    def test_empty_estimator_file_is_distinct_from_missing_file(self) -> None:
        out = self.run_fixture([])
        self.assertIn("ukf", out)
        block = out["ukf"]
        self.assertEqual((block["n"], block["n_hat"], block["n_gt"]), (0, 0, 4))
        self.assertEqual(block["coverage"], 0.0)
        self.assertTrue(math.isnan(block["estimate_coverage"]))
        self.assertIn("estimate_coverage=N/A", _fmt(block))
        self.assertNotIn("ukf", self.run_fixture(None))

    def test_invalid_estimates_do_not_disappear_from_counts(self) -> None:
        out = self.run_fixture([self.sample(0.0, float("nan"))])
        self.assertIn("ukf", out)
        self.assertEqual((out["ukf"]["n_hat"], out["ukf"]["n"]), (1, 0))

    def test_full_coverage_remains_full(self) -> None:
        block = self.run_fixture([self.sample(t) for t in (0.0, 0.02, 0.04, 0.06)])["ukf"]
        self.assertEqual(block["coverage"], 1.0)
        self.assertEqual(block["rmse_s"], 0.0)
        self.assertEqual(block["nees_s"], 0.0)

    def test_no_reference_has_undefined_ratios(self) -> None:
        block = metrics([], [], [], [])
        for key in ("coverage", "env_s", "env_v"):
            with self.subTest(key=key):
                self.assertTrue(math.isnan(block[key]))
        self.assertIn("coverage=N/A", _fmt(block))

    def test_pairing_accepts_unsorted_inputs_and_keeps_covariance_aligned(self) -> None:
        got = _pair_ukf_gt([0.04, 0.0, 0.02], [4.0, 0.0, 2.0], [1.0]*3,
                           [4.0, 1.0, 2.0], [0.02, 0.04, 0.0], [20.0, 40.0, 0.0], [10.0]*3)
        self.assertEqual(got, ([4.0, 0.0, 2.0], [1.0]*3, [4.0, 1.0, 2.0], [40.0, 0.0, 20.0], [10.0]*3))

    def test_nonfinite_gt_timestamp_cannot_match(self) -> None:
        for t in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(t=t):
                got = _pair_ukf_gt([0.0], [0.0], [1.0], [1.0], [t], [0.0], [1.0])
                self.assertEqual(got, ([], [], [], [], []))

    def test_duplicate_gt_timestamps_do_not_block_later_times(self) -> None:
        got = _pair_ukf_gt([0.02], [2.0], [1.0], [1.0], [0.0, 0.0, 0.02], [0.0, 0.0, 2.0], [1.0]*3)
        self.assertEqual(got[3], [2.0])

    def test_tolerance_is_inclusive_and_finite(self) -> None:
        for t, expected in ((0.011, 1), (math.nextafter(0.011, math.inf), 0)):
            got = _pair_ukf_gt([t], [0.0], [1.0], [1.0], [0.0], [0.0], [1.0])
            self.assertEqual(len(got[0]), expected)
        for tol in (-1.0, float("nan"), float("inf")):
            with self.subTest(tol=tol), self.assertRaises(ValueError):
                _pair_ukf_gt([0.0], [0.0], [1.0], [1.0], [0.0], [0.0], [1.0], tol)

    def test_pairing_matches_independent_brute_force_reference(self) -> None:
        rng = random.Random(907)
        tg = [float(i) / 10 for i in range(20)]
        rng.shuffle(tg)
        te = [rng.uniform(-0.2, 2.2) for _ in range(100)]
        expected = []
        for i, t in enumerate(te):
            j = min(range(len(tg)), key=lambda j: (abs(tg[j]-t), tg[j], j))
            if abs(tg[j]-t) <= 0.04:
                expected.append((float(i), 1.0, float(i+1), tg[j]*10, 2.0))
        got = _pair_ukf_gt(te, list(map(float, range(100))), [1.0]*100,
                           [float(i+1) for i in range(100)], tg, [t*10 for t in tg], [2.0]*20, 0.04)
        self.assertEqual(list(zip(*got)), expected)


if __name__ == "__main__":
    unittest.main()

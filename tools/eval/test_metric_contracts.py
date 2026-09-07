"""Regression contracts for metric exposure; no UKF or plant imports."""
from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from check_envelope import check_rows, main as check_main


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


if __name__ == "__main__":
    unittest.main()

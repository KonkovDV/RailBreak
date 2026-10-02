"""The offline checker pairs stamps the way the official synchronizer does."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools.eval.offline_checker import self_test


class OfflineCheckerTests(unittest.TestCase):
    def test_sync_matches_the_official_slop(self) -> None:
        self_test()


if __name__ == "__main__":
    unittest.main()

"""Round-trip is in tools/eval/test_eval.py. This checks a real bag when present."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bags import FRONT, MASTER_FIX, iter_decoded  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DROP = ROOT / "files" / "data"


class OrganizerDecodeTests(unittest.TestCase):
    def test_one_bag_matches_declared_types(self) -> None:
        if not DROP.is_dir():
            self.skipTest("organiser drop is not on this machine")
        bag = next(p for p in sorted(DROP.iterdir()) if p.is_dir() and (p / "metadata.yaml").is_file())
        seen = {FRONT: 0, MASTER_FIX: 0}
        for _ts, topic, rec in iter_decoded(bag):
            if topic == FRONT and seen[FRONT] == 0:
                self.assertIn("velocity", rec)
                self.assertTrue(rec["stamp_s"] > 1e9)
                seen[FRONT] = 1
            elif topic == MASTER_FIX and seen[MASTER_FIX] == 0:
                self.assertGreater(rec["lat"], 50.0)
                self.assertLess(rec["lat"], 60.0)
                seen[MASTER_FIX] = 1
            if all(seen.values()):
                break
        self.assertEqual(seen[FRONT], 1)


if __name__ == "__main__":
    unittest.main()

"""lint_claims.py catches banned wording, unknown claim ids and numbers missing from their source."""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lint_claims  # noqa: E402

REGISTRY = """| № | Утверждение | Источник | Как | Граница |
|---|---|---|---|---|
| C-01 | Положение 1.157702 м, скорость 0.024691 м/с | [r](results.md) | x | y |
"""


def tree(root: Path, readme: str, results: str = "1.157702 0.024691") -> Path:
    (root / "BANNED_CLAIMS.txt").write_text("# comment\nпобедитель хакатона\n", encoding="utf-8")
    (root / "CLAIMS_REGISTRY.md").write_text(REGISTRY, encoding="utf-8")
    (root / "results.md").write_text(results, encoding="utf-8")
    for name in lint_claims.HEADLINE_FILES:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_text("1.157702 0.024691", encoding="utf-8")
    (root / "README.md").write_text(readme, encoding="utf-8")
    return root


class LintClaimsTests(unittest.TestCase):
    def test_clean_tree_passes(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = tree(Path(d), "1.157702 0.024691 (C-01)")
            self.assertEqual(lint_claims.check(root), [])

    def test_banned_phrase_fails(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = tree(Path(d), "1.157702 0.024691 Мы ПОБЕДИТЕЛЬ ХАКАТОНА")
            self.assertTrue(any("banned claim" in e for e in lint_claims.check(root)))

    def test_unknown_claim_id_fails(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = tree(Path(d), "1.157702 0.024691 (C-99)")
            self.assertTrue(any("C-99" in e for e in lint_claims.check(root)))

    def test_number_missing_from_source_fails(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = tree(Path(d), "1.157702 0.024691", results="1.2 0.024691")
            self.assertTrue(any("1.157702 is not in results.md" in e for e in lint_claims.check(root)))

    def test_headline_drift_fails(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = tree(Path(d), "1.16 0.0247")
            self.assertTrue(any("README.md: headline" in e for e in lint_claims.check(root)))


if __name__ == "__main__":
    unittest.main()

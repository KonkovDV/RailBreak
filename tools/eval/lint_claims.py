"""Claims gate: banned wording, claim references and numbers against their source.

Ported from KonkovDV/Mandat and KonkovDV/SynAPS-ProgramPlan (scripts/lint_claims.py).

1. No phrase from BANNED_CLAIMS.txt appears in a tracked text file (case-insensitive).
2. Every C-xx, M-xx and L-xx reference in a document exists in CLAIMS_REGISTRY.md.
3. Every decimal number in a registry claim appears in the file that row names as its source.
4. The headline checker numbers in README.md, README.en.md, the pitch and results agree.

Audit records kept verbatim (docs/external-audit/) are not rewritten and are skipped.
No third-party packages.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from urllib.parse import unquote

SKIP_FILES = {"CLAIMS_REGISTRY.md", "BANNED_CLAIMS.txt", "lint_claims.py"}
SKIP_PREFIXES = ("docs/external-audit/", ".git/")
SUFFIXES = {".md", ".py", ".cpp", ".hpp", ".txt", ".yaml", ".yml", ".sh", ".ps1"}
HEADLINE = ("1.157702", "0.024691")
HEADLINE_FILES = ("README.md", "README.en.md", "docs/solution/pitch.md", "docs/solution/results.md",
                  "CLAIMS_REGISTRY.md")
ID = re.compile(r"\b([CML]-\d{2})\b")
DECIMAL = re.compile(r"(?<![\w.])\d+\.\d+(?![\w.])")
LINK = re.compile(r"\[[^\]]*\]\(([^)#\s]+)")


def banned(root: Path) -> list[str]:
    lines = (root / "BANNED_CLAIMS.txt").read_text(encoding="utf-8").splitlines()
    return [ln.strip().lower() for ln in lines if ln.strip() and not ln.lstrip().startswith("#")]


def tracked(root: Path) -> list[Path]:
    out = []
    for path in sorted(root.rglob("*")):
        rel = path.relative_to(root).as_posix()
        if not path.is_file() or path.suffix not in SUFFIXES or path.name in SKIP_FILES:
            continue
        if rel.startswith(SKIP_PREFIXES) or any(p in {"build", "install", "log", "node_modules", "__pycache__"}
                                                 for p in path.relative_to(root).parts):
            continue
        out.append(path)
    return out


def registry(root: Path) -> tuple[set[str], list[tuple[str, str, str]]]:
    ids: set[str] = set()
    rows: list[tuple[str, str, str]] = []
    for line in (root / "CLAIMS_REGISTRY.md").read_text(encoding="utf-8").splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 3 and ID.fullmatch(cells[0]):
            ids.add(cells[0])
            link = LINK.search(cells[2])
            rows.append((cells[0], cells[1], unquote(link.group(1)) if link else ""))
    return ids, rows


def check(root: Path) -> list[str]:
    errors: list[str] = []
    phrases = banned(root)
    ids, rows = registry(root)
    for path in tracked(root):
        rel = path.relative_to(root).as_posix()
        text = path.read_text(encoding="utf-8", errors="ignore")
        low = text.lower()
        for phrase in phrases:
            if phrase in low:
                line = low[: low.index(phrase)].count("\n") + 1
                errors.append(f"{rel}:{line}: banned claim «{phrase}»")
        if path.suffix == ".md":
            for ref in sorted(set(ID.findall(text))):
                if ref not in ids:
                    errors.append(f"{rel}: {ref} is not in CLAIMS_REGISTRY.md")
    for claim_id, claim, source in rows:
        if not source:
            errors.append(f"{claim_id}: no source link")
            continue
        src = root / source
        if not src.is_file():
            errors.append(f"{claim_id}: source {source} does not exist")
            continue
        body = src.read_text(encoding="utf-8")
        for number in DECIMAL.findall(claim):
            if number not in body:
                errors.append(f"{claim_id}: {number} is not in {source}")
    for name in HEADLINE_FILES:
        path = root / name
        if not path.is_file():
            errors.append(f"headline file missing: {name}")
            continue
        text = path.read_text(encoding="utf-8")
        for number in HEADLINE:
            if number not in text:
                errors.append(f"{name}: headline number {number} is missing")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    errors = check(args.root.resolve())
    for error in errors:
        print(error)
    if errors:
        return 1
    print("claims: banned wording absent, references resolve, registry numbers match their sources")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Structural checks for current documentation (not historical audit reports).

No network access or third-party packages. Does not validate external URLs,
GitHub heading anchors, LaTeX semantics, or the truth of prose.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

CURRENT_DOCS = (
    "README.md", "docs/architecture.md", "docs/brief.md", "docs/checker.md",
    "docs/estimator-priors.md",
    "docs/integrity-risk.md", "docs/math.md",
    "docs/verification.md", "docs/refs.md",
    "docs/data-contract.md",
    "docs/solution/model.md", "docs/solution/assumptions.md",
    "docs/solution/results.md", "docs/solution/form.md", "docs/solution/pitch.md",
)


def check(root: Path) -> list[str]:
    root = root.resolve()
    errors: list[str] = []
    for name in CURRENT_DOCS:
        source = root / name
        if not source.is_file():
            errors.append(f"missing current document: {name}")
            continue
        text = source.read_text(encoding="utf-8")
        fence = None
        prose = []
        for line in text.splitlines():
            marker = re.match(r"^\s*(`{3,}|~{3,})(.*)$", line)
            if marker and fence is None:
                fence = marker.group(1)
            elif marker and fence is not None and not marker.group(2).strip() and \
                    marker.group(1)[0] == fence[0] and len(marker.group(1)) >= len(fence):
                fence = None
            elif fence is None:
                prose.append(line)
        if fence is not None:
            errors.append(f"unclosed code fence: {name}")
        body = "\n".join(prose)
        for target in re.findall(r"\[[^\]\n]+\]\(([^)\n]+)\)", body):
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            path = (source.parent / unquote(parsed.path)).resolve()
            if not path.is_relative_to(root) or not path.exists():
                errors.append(f"broken relative link: {name}: {target}")
        for token in re.findall(r"`([^`\n]+)`", body):
            if re.fullmatch(r"(?:standalone|tram_dr_localization|tools)/[\w./-]+\.(?:cpp|hpp|py|yaml|yml|xml)", token):
                if not (root / token).is_file():
                    errors.append(f"missing source reference: {name}: {token}")
    try:
        version = ET.parse(root / "tram_dr_localization/package.xml").getroot().findtext("version")
        header = (root / "tram_dr_localization/include/tram_dr_localization/types.hpp").read_text(encoding="utf-8")
        model = re.search(r'kModelVersion\s*=\s*"tramDR-([^"]+)"', header)
        if not version or not model or model.group(1) != version:
            errors.append("model/package version mismatch")
        if f"`tramDR-{version}`" not in (root / "README.md").read_text(encoding="utf-8"):
            errors.append("README does not identify package version")
        cmake = (root / "standalone/CMakeLists.txt").read_text(encoding="utf-8")
        targets = re.findall(r"add_test\(NAME\s+(\w+)", cmake)
        verification = (root / "docs/verification.md").read_text(encoding="utf-8")
        if not targets:
            errors.append("no registered CTest targets")
        for target in targets:
            if f"| `{target}` |" not in verification:
                errors.append(f"undocumented CTest target: {target}")
    except (OSError, ET.ParseError) as exc:
        errors.append(f"cannot validate build metadata: {exc}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    errors = check(args.root)
    for error in errors:
        print(error)
    if errors:
        return 1
    print(f"docs: {len(CURRENT_DOCS)} current documents; structural checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

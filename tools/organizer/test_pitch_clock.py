"""The timed pitch keeps five published numbers and does not claim a HIL run."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PITCH = (ROOT / "docs" / "solution" / "pitch.md").read_text(encoding="utf-8")
RESULTS = (ROOT / "docs" / "solution" / "results.md").read_text(encoding="utf-8")
FORM = (ROOT / "docs" / "solution" / "form.md").read_text(encoding="utf-8")

HEADINGS = (
    "0:00–0:30",
    "0:30–1:15",
    "1:15–2:00",
    "2:00–3:00",
    "3:00–4:00",
    "4:00–4:40",
    "4:40–5:20",
    "5:20–конец",
)
BLOCKS = ("notch table", "grade", "wheel consensus", "station anchors")
LIMITS = ("common-mode", "только маршрут 10", "station topology", "no certification claim")
ROADMAP = ("integrity bound", "branch hypotheses", "calibrated uncertainty", "HIL/WSP fault library")
FIVE = ("1.467 м", "5.828 м", "29.5 м", "0.6–3.3 мс", "23–24 МБ")
CLOSE = (
    "Мы сдаём не красивую траекторию, а воспроизводимый резервный канал: "
    "минимальные входы, ограниченный GNSS, отказоустойчивый runtime, "
    "измеренная точность и честная оценка доверия."
)


def main() -> int:
    errors: list[str] = []
    for token in (*HEADINGS, *BLOCKS, *LIMITS, *ROADMAP, *FIVE):
        if token not in PITCH:
            errors.append(f"pitch missing {token}")
    if CLOSE not in PITCH:
        errors.append("closing line missing")
    if "10017" not in PITCH or "8195" not in PITCH or "0.771" not in PITCH or "0.775" not in PITCH:
        errors.append("raw/gated retention counts missing")
    if "Стенд HIL не запускался" not in PITCH:
        errors.append("HIL is not marked as not run")
    if "integrity_certification_claim" not in PITCH:
        errors.append("certification flag missing")
    for number in ("1.467", "5.828"):
        if number not in FORM:
            errors.append(f"{number} is not in form.md")
    for number in ("29.5", "0.6–3.3", "23–24", "0.771", "0.775", "10017", "8195"):
        if number not in RESULTS:
            errors.append(f"{number} is not in results.md")
    for error in errors:
        print(error)
    if errors:
        return 1
    print("pitch: five numbers, raw/gated beside them, HIL not claimed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

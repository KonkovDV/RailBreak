"""The timed pitch keeps its clock, the official checker numbers and the scope lines.

The numbers the pitch speaks must exist in results.md, so a stale pitch fails here.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PITCH = (ROOT / "docs" / "solution" / "pitch.md").read_text(encoding="utf-8")
RESULTS = (ROOT / "docs" / "solution" / "results.md").read_text(encoding="utf-8")
FORM = (ROOT / "docs" / "solution" / "form.md").read_text(encoding="utf-8")
META = (ROOT / "railbreak_backup_odometry" / "assets" / "meta.yaml").read_text(encoding="utf-8")

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
# Official checker, current tree and deadline submission; offline val; archived 6af0037 row.
CHECKER = ("1.157702 м", "0.024691 м/с", "2.151 м", "0.051 м/с", "0.20", "24.1 МБ")
OFFLINE = ("1.47 м", "1.467 м", "5.828 м", "29.5 м", "0.6–3.3 мс", "23–24 МБ")
CLOSE = (
    "Мы сдаём не красивую траекторию, а воспроизводимый резервный канал: "
    "минимальные входы, ограниченный GNSS, отказоустойчивый runtime, "
    "измеренная точность и честная оценка доверия."
)


def main() -> int:
    errors: list[str] = []
    for token in (*HEADINGS, *BLOCKS, *LIMITS, *ROADMAP, *CHECKER, *OFFLINE):
        if token not in PITCH:
            errors.append(f"pitch missing {token}")
    if CLOSE not in PITCH:
        errors.append("closing line missing")
    if "integrity_certification_claim" not in PITCH:
        errors.append("certification flag missing")
    if "HIL пройден" in PITCH or "HIL passed" in PITCH:
        errors.append("pitch claims a HIL run")
    for number in ("1.467", "5.828"):
        if number not in FORM:
            errors.append(f"{number} is not in form.md")
    for number in ("1.157702", "0.024691", "2.151", "0.051", "0.20 мс", "24.1 МБ",
                   "1.47", "29.5", "0.6–3.3", "23–24"):
        if number not in RESULTS:
            errors.append(f"{number} is not in results.md")
    k0 = next((line.split(":", 1)[1].strip() for line in META.splitlines()
               if line.startswith("k0:")), "")
    if not k0 or k0 not in PITCH:
        errors.append("pitch does not state the k0 of assets/meta.yaml")
    for stale in ("10928.09", "k₀ = 1.0027 почти целиком закрывает"):
        if stale in PITCH:
            errors.append(f"pitch carries a stale map figure: {stale}")
    for error in errors:
        print(error)
    if errors:
        return 1
    print("pitch: clock, checker numbers from results.md, k0 from assets, HIL not claimed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

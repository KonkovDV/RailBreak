"""The jury demo script keeps the three lines, both GNSS modes and the published fault rows."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEMO = ROOT / "docs" / "solution" / "demo.md"
RESULTS = (ROOT / "docs" / "solution" / "results.md").read_text(encoding="utf-8")
INTEGRITY = (ROOT / "docs" / "solution" / "integrity.md").read_text(encoding="utf-8")

LINES = (
    "После стартового окна GNSS не ведёт оценку: скорость и путь дают тележки и модель, а редкая пачка RTK на оси только подправляет дугу. В строгом режиме GNSS отписывается полностью.",
    "Мы не переключаемся жёстко между датчиками. Недостоверное измерение получает пониженный вес, а модель сохраняет непрерывность.",
    "Два согласованных датчика не позволяют наблюдать общую моду. Система это распознаёт: через три секунды режим помечается как degraded, и integrity bound растёт. Следующая абсолютная путевая привязка ограничивает накопленный дрейф.",
)


def main() -> int:
    text = DEMO.read_text(encoding="utf-8")
    errors: list[str] = []
    for line in LINES:
        if line not in text:
            errors.append("missing spoken line")
    for token in (
        "gnss=closed",
        "gnss=correcting",
        "gnss_correction:=false",
        "map",
        "base_link",
        "mgrs",
        "scale_rear_5pct",
        "drop_front_20s",
        "slide_both_20pct",
        "DEGRADED_COMMON_MODE_UNOBSERVABLE",
        "32.606",
        "integrity_certification_claim",
        "n_anchor",
        "84.0",
        "1.60",
        "6.46",
    ):
        if token not in text:
            errors.append(f"demo missing {token}")
    if "84.0" not in RESULTS or "1.60" not in RESULTS or "6.46" not in RESULTS:
        errors.append("cited fault rows are not in results.md")
    if "32.606" not in INTEGRITY:
        errors.append("bound addend is not in integrity.md")
    if "input_sha256" in text or "bag_sha256" in text:
        errors.append("demo stores a bag hash")
    if "protection level" in text and "не protection level" not in text:
        errors.append("demo names a protection level")
    params = (ROOT / "railbreak_backup_odometry" / "config" / "params.yaml").read_text(encoding="utf-8")
    if "gnss_correction: true" in params and "gnss=correcting" not in text:
        errors.append("default keeps rare GNSS corrections, but the demo does not show gnss=correcting")
    for error in errors:
        print(error)
    if errors:
        return 1
    print("demo: three acts, both GNSS modes, published fault rows, no bag hashes")
    return 0


if __name__ == "__main__":
    sys.exit(main())

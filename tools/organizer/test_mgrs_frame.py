"""The published MGRS frame stays continuous across the 100 km square boundary."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from geo import utm_forward

LAT = 55.8088325462547
LON = 37.4602768500852
X = 103501.6309
Y = 85876.1201


def square(zone: int, easting: float, northing: float) -> str:
    east_sets = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")
    north_sets = ("ABCDEFGHJKLMNPQRSTUV", "FGHJKLMNPQRSTUVABCDE")
    col = int(easting // 100000)
    row = int(northing // 100000) % 20
    return east_sets[(zone - 1) % 3][(col - 1) % 8] + north_sets[(zone - 1) % 2][row]


def main() -> int:
    easting, northing = utm_forward(LAT, LON, 37)
    x = float(easting) - 300000.0
    y = float(northing) - 6100000.0
    errors: list[str] = []
    if abs(x - X) > 1e-4 or abs(y - Y) > 1e-4:
        errors.append(f"frame {x:.4f} {y:.4f}")
    if abs((float(easting) % 100000.0) - X) < 1.0:
        errors.append("x collapsed to easting mod 100000")
    if square(37, float(easting), float(northing)) != "DB":
        errors.append("sample is not in square DB")
    if square(37, 303500.0, 6185800.0) != "CB":
        errors.append("37UCB cell was not column C")
    for error in errors:
        print(error)
    if errors:
        return 1
    print("mgrs frame: continuous x 103501.6309, square DB, corner stays 300000")
    return 0


if __name__ == "__main__":
    sys.exit(main())

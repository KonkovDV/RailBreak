"""The Python twin rejects the same configurations as validate_params in the C++ core.

Every "<name> must be ..." message in validate_params names a parameter. Setting that
parameter to NaN must raise ValueError in the twin too, so the two cores cannot drift
on what counts as a valid configuration.
"""

from __future__ import annotations

import dataclasses
import math
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from odometer import Params, _require_config  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "railbreak_backup_odometry" / "include" / "railbreak_backup_odometry" / "track_odometer.hpp"
# C++ message word -> Python Params field.
ALIASES = {"wheel_unit_scale": "unit", "initial": "k0", "davis": "davis_a"}


def cpp_names() -> list[str]:
    text = HEADER.read_text(encoding="utf-8")
    body = text[text.index("inline void validate_params"):text.index("inline void validate_geometry")]
    names = re.findall(r'invalid_argument\("([a-z_0-9]+)', body)
    return sorted({ALIASES.get(n, n) for n in names})


class ConfigParityTests(unittest.TestCase):
    def test_default_passes(self) -> None:
        _require_config(Params())

    def test_every_cpp_gate_is_a_python_gate(self) -> None:
        names = cpp_names()
        self.assertGreater(len(names), 25)
        fields = {f.name for f in dataclasses.fields(Params)}
        for name in names:
            with self.subTest(name=name):
                self.assertIn(name, fields)
                with self.assertRaises(ValueError):
                    _require_config(dataclasses.replace(Params(), **{name: math.nan}))

    def test_dt_max_zero_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            _require_config(dataclasses.replace(Params(), dt_max=0.0))


if __name__ == "__main__":
    unittest.main()

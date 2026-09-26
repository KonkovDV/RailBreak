#!/usr/bin/env bash
# Missing /result/position must fail smoke and play. no-assets may omit it.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
JURY_POSITION_LIB=1
# shellcheck disable=SC1091
source "${ROOT}/scripts/jury_inside.sh"

tmp=$(mktemp)
trap 'rm -f "${tmp}"' EXIT

: > "${tmp}"
if position_required 1 "${tmp}"; then
  echo "empty position file was accepted" >&2
  exit 1
fi
printf 'x: 99123.0\ny: 1.0\n' > "${tmp}"
if ! position_required 1 "${tmp}"; then
  echo "a pose was rejected" >&2
  exit 1
fi
: > "${tmp}"
if ! position_required 0 "${tmp}"; then
  echo "no-assets was required to publish position" >&2
  exit 1
fi

for wrapper in "${ROOT}/scripts/jury.sh" "${ROOT}/scripts/jury.ps1"; do
  if ! grep -q "REQUIRE_POSITION" "${wrapper}"; then
    echo "${wrapper} does not mention REQUIRE_POSITION" >&2
    exit 1
  fi
done
if ! grep -q 'REQUIRE_POSITION=1' "${ROOT}/scripts/jury.sh"; then
  echo "jury.sh does not require position by default" >&2
  exit 1
fi
if ! grep -q "REQUIRE_POSITION = '1'" "${ROOT}/scripts/jury.ps1"; then
  echo "jury.ps1 does not require position by default" >&2
  exit 1
fi
if ! grep -q 'REQUIRE_POSITION=0' "${ROOT}/scripts/jury.sh"; then
  echo "no-assets exception missing from jury.sh" >&2
  exit 1
fi
if ! grep -q "REQUIRE_POSITION = '0'" "${ROOT}/scripts/jury.ps1"; then
  echo "no-assets exception missing from jury.ps1" >&2
  exit 1
fi
python3 - "${ROOT}/scripts/jury_inside.sh" <<'PY'
import sys
text = open(sys.argv[1], encoding="utf-8").read().replace("\r\n", "\n")
none = text.find("position: none. /result/position")
err = text.find("no /result/position", none)
exit1 = text.find("exit 1", err)
if none < 0 or err < 0 or exit1 < 0 or exit1 - err > 200:
    raise SystemExit("position: none is not followed by exit 1")
for line in text.splitlines():
    if "score_ros.py" in line and "|| true" in line:
        raise SystemExit("score_ros failure is swallowed")
PY
got=$(score_scenario_status acceptance 2)
if [ "${got}" != "2" ]; then
  echo "acceptance did not keep scorer status ${got}" >&2
  exit 1
fi
got=$(score_scenario_status exploratory 2)
if [ "${got}" != "0" ]; then
  echo "exploratory did not stay green, status ${got}" >&2
  exit 1
fi
got=$(score_scenario_status acceptance 0)
if [ "${got}" != "0" ]; then
  echo "a successful score was failed" >&2
  exit 1
fi
got=$(score_scenario_status other 2)
if [ "${got}" != "2" ]; then
  echo "an unknown mode hid the scorer failure" >&2
  exit 1
fi
if ! grep -q 'SCORE_MODE=acceptance' "${ROOT}/scripts/jury.sh"; then
  echo "jury.sh record is not acceptance" >&2
  exit 1
fi
if ! grep -q "SCORE_MODE = 'acceptance'" "${ROOT}/scripts/jury.ps1"; then
  echo "jury.ps1 record is not acceptance" >&2
  exit 1
fi
echo ok

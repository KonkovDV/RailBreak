#!/usr/bin/env bash
# Same scenarios on Linux, macOS and Windows (Git Bash), via Docker.
#   scripts/jury.sh smoke --bag /path/to/bag --msgs /path/to/tram_vehicle_msgs
#   scripts/jury.sh core
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SCENARIO=${1:-}
shift || true

usage() {
  cat <<'EOF'
scenarios: play smoke clock fast frame no-gnss no-assets arc record core
  --bag PATH    rosbag2 directory (metadata.yaml inside)
  --msgs PATH   tram_vehicle_msgs package
  --initial-s M arc length for scenario arc (default 0)
core does not need Docker, a bag, or the message package.
EOF
}

if [ -z "${SCENARIO}" ] || [ "${SCENARIO}" = "-h" ] || [ "${SCENARIO}" = "--help" ]; then
  usage
  exit 0
fi

BAG=""
MSGS=""
INITIAL_S_ARG="0"
while [ $# -gt 0 ]; do
  case "$1" in
    --bag|--msgs|--initial-s)
      if [ $# -lt 2 ]; then echo "missing value for $1" >&2; usage; exit 2; fi
      case "$1" in
        --bag) BAG=$2 ;;
        --msgs) MSGS=$2 ;;
        --initial-s) INITIAL_S_ARG=$2 ;;
      esac
      shift 2
      ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

run_core() {
  cmake -S "${ROOT}/railbreak_backup_odometry/tools" -B "${ROOT}/build/rbo" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${ROOT}/build/rbo" --config Release --parallel
  ctest --test-dir "${ROOT}/build/rbo" -C Release --output-on-failure \
    || ctest --test-dir "${ROOT}/build/rbo" --output-on-failure
}

if [ "${SCENARIO}" = "core" ]; then
  run_core
  exit 0
fi

case "${SCENARIO}" in
  play|smoke|clock|fast|frame|no-gnss|no-assets|arc|record) ;;
  *) echo "unknown scenario: ${SCENARIO}" >&2; usage; exit 2 ;;
esac

if [ -z "${MSGS}" ]; then
  if [ -f "${ROOT}/tram_vehicle_msgs/package.xml" ]; then
    MSGS=${ROOT}/tram_vehicle_msgs
  elif [ -f "${ROOT}/files/tram_vehicle_msgs/package.xml" ]; then
    MSGS=${ROOT}/files/tram_vehicle_msgs
  else
    echo "pass --msgs /path/to/tram_vehicle_msgs" >&2
    exit 2
  fi
fi
if [ -z "${BAG}" ] || [ ! -f "${BAG}/metadata.yaml" ]; then
  echo "pass --bag /path/to/rosbag2_directory (metadata.yaml inside)" >&2
  exit 2
fi

to_host() {
  local p
  p=$(cd "$1" && pwd)
  if command -v cygpath >/dev/null 2>&1; then
    cygpath -w "${p}"
  else
    printf '%s\n' "${p}"
  fi
}
MSGS_DIR=$(to_host "${MSGS}")
BAG=$(to_host "${BAG}")
export MSGS_DIR BAG
export OUT=${OUT:-${ROOT}/jury_out}
if command -v cygpath >/dev/null 2>&1; then
  OUT=$(cygpath -w "${OUT}")
fi
export RATE=1 CLOCK=0 OUTPUT_FRAME=mgrs ASSETS_DIR= TOPICS=
export GNSS_WINDOW=3.0 GNSS_WAIT= INITIAL_S= DURATION= RECORD=0 SCORE=0 REQUIRE_POSITION=1

case "${SCENARIO}" in
  play) ;;
  smoke) DURATION=25 ;;
  clock) CLOCK=1 ;;
  fast) RATE=10 ;;
  frame) OUTPUT_FRAME=mkrs_start ;;
  no-gnss)
    GNSS_WAIT=2
    TOPICS="/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd"
    ;;
  no-assets) ASSETS_DIR=/nonexistent; REQUIRE_POSITION=0 ;;
  arc)
    GNSS_WAIT=2
    INITIAL_S=${INITIAL_S_ARG}
    TOPICS="/vehicle/front_bogie_velocity /vehicle/rear_bogie_velocity /vehicle/driver_position_cmd"
    ;;
  record) RECORD=1; SCORE=1 ;;
esac

export RATE CLOCK OUTPUT_FRAME ASSETS_DIR TOPICS GNSS_WINDOW GNSS_WAIT INITIAL_S DURATION RECORD SCORE REQUIRE_POSITION
cd "${ROOT}"
docker compose -f docker-compose.jury.yml run -T --rm --build jury

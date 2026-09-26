#!/usr/bin/env bash
# Runs inside the Humble image. Host wrappers: scripts/jury.sh and scripts/jury.ps1.

# Missing /result/position fails the scenario. no-assets sets REQUIRE_POSITION=0.
position_required() {
  local require="${1:-1}"
  local pos_file="$2"
  if [ "${require}" = "1" ] && ! grep -q "x:" "${pos_file}" 2>/dev/null; then
    return 1
  fi
  return 0
}

# acceptance keeps the scorer status. exploratory warns and stays green.
# Any other mode is acceptance, so a forgotten flag cannot hide a failure.
score_scenario_status() {
  local mode="${1:-acceptance}"
  local status="${2:-0}"
  if [ "${status}" -eq 0 ]; then
    printf '0\n'
    return 0
  fi
  if [ "${mode}" = "exploratory" ]; then
    echo "score_ros status ${status}: exploratory mode does not fail the scenario" >&2
    printf '0\n'
    return 0
  fi
  printf '%s\n' "${status}"
  return 0
}

if [ "${JURY_POSITION_LIB:-0}" = "1" ]; then
  return 0 2>/dev/null || exit 0
fi

set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u

if [ ! -f /bag/metadata.yaml ]; then
  echo "bag directory /bag has no metadata.yaml" >&2
  exit 2
fi
if [ ! -f /org_msgs/package.xml ]; then
  echo "tram_vehicle_msgs is not mounted at /org_msgs" >&2
  exit 2
fi

mkdir -p /ws/src
rm -rf /ws/src/tram_vehicle_msgs /ws/src/railbreak_backup_odometry
cp -a /org_msgs /ws/src/tram_vehicle_msgs
cp -a /opt/src/railbreak_backup_odometry /ws/src/railbreak_backup_odometry
if ! grep -q "<maintainer" /ws/src/tram_vehicle_msgs/package.xml; then
  sed -i 's#<license>#<maintainer email="organiser@example.invalid">organiser</maintainer>\n  <license>#' \
    /ws/src/tram_vehicle_msgs/package.xml
fi

cd /ws
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source /ws/install/setup.bash
set -u

# rclcpp rejects an integer token for a double parameter ("2" is not "2.0").
as_double() {
  case "$1" in
    *.*) printf '%s' "$1" ;;
    *) printf '%s.0' "$1" ;;
  esac
}

args=(-p "output_frame:=${OUTPUT_FRAME:-mgrs}" -p "gnss_init_window_s:=$(as_double "${GNSS_WINDOW:-3.0}")")
if [ "${CLOCK:-0}" = "1" ]; then
  args+=(-p use_sim_time:=true)
fi
if [ -n "${ASSETS_DIR:-}" ]; then
  args+=(-p "assets_dir:=${ASSETS_DIR}")
fi
if [ -n "${GNSS_WAIT:-}" ]; then
  args+=(-p "gnss_wait_s:=$(as_double "${GNSS_WAIT}")")
fi
if [ -n "${INITIAL_S:-}" ]; then
  args+=(-p "initial_s_m:=$(as_double "${INITIAL_S}")")
fi

ros2 run railbreak_backup_odometry backup_odometry_node --ros-args "${args[@]}" \
  > /tmp/node.log 2>&1 &
node_pid=$!
sleep 2
if ! kill -0 "${node_pid}" 2>/dev/null; then
  echo "NODE DIED" >&2
  cat /tmp/node.log >&2
  exit 1
fi

timeout 180 ros2 topic echo /result/position --once > /tmp/pos.txt 2>&1 &
echo_pid=$!

if [ "${RECORD:-0}" = "1" ]; then
  mkdir -p /out
  rm -rf /out/result
  ros2 bag record -o /out/result /result/velocity /result/position /result/diagnostics \
    > /tmp/rec.log 2>&1 &
  rec_pid=$!
  sleep 2
fi

play=(ros2 bag play /bag --rate "${RATE:-1}")
if [ "${CLOCK:-0}" = "1" ]; then
  play+=(--clock)
fi
if [ -n "${TOPICS:-}" ]; then
  # shellcheck disable=SC2206
  extra=(${TOPICS})
  play+=(--topics "${extra[@]}")
fi

set +e
if [ -n "${DURATION:-}" ]; then
  timeout "${DURATION}" "${play[@]}"
else
  "${play[@]}"
fi
play_status=$?
set -e

sleep 2
if [ "${RECORD:-0}" = "1" ]; then
  kill -INT "${rec_pid}" 2>/dev/null || true
  wait "${rec_pid}" 2>/dev/null || true
fi
kill -INT "${echo_pid}" 2>/dev/null || true
pkill -INT -f lib/railbreak_backup_odometry/backup_odometry_node 2>/dev/null || true
wait "${node_pid}" 2>/dev/null || true

echo "----- node (head) -----"
head -n 15 /tmp/node.log
echo "----- node (tail) -----"
tail -n 25 /tmp/node.log
echo "----- first /result/position -----"
head -n 40 /tmp/pos.txt || true
if position_required "${REQUIRE_POSITION:-1}" /tmp/pos.txt; then
  if grep -q "x:" /tmp/pos.txt 2>/dev/null; then
    echo "position: received"
  else
    echo "position: none (REQUIRE_POSITION=0, scenario no-assets). Velocity does not wait."
  fi
else
  echo "position: none. /result/position is published once a stamp passes gnss_init_window_s after the first fix, or gnss_wait_s with no fix. Velocity does not wait."
  echo "no /result/position" >&2
  exit 1
fi
if [ "${RECORD:-0}" = "1" ] && [ -f /out/result/metadata.yaml ]; then
  echo "----- recorded /result -----"
  ros2 bag info /out/result | head -n 25
fi
score_status=0
if [ "${SCORE:-0}" = "1" ]; then
  echo "----- score_ros -----"
  if [ ! -f /out/result/metadata.yaml ]; then
    echo "no recorded /result bag" >&2
    score_status=2
  else
    python3 /opt/tools/organizer/score_ros.py /out/result /bag --frame "${OUTPUT_FRAME:-mgrs}" || score_status=$?
  fi
  code=$(score_scenario_status "${SCORE_MODE:-acceptance}" "${score_status}")
  if [ "${code}" -ne 0 ]; then
    exit "${code}"
  fi
fi
if [ "${play_status}" -eq 124 ]; then
  play_status=0
fi
exit "${play_status}"

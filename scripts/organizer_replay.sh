#!/usr/bin/env bash
# Build tram_vehicle_msgs + railbreak_backup_odometry and replay one organiser bag.
# Runs inside osrf/ros:humble. Expects:
#   /ws/src/tram_vehicle_msgs            (organiser package)
#   /ws/src/railbreak_backup_odometry    (this package, assets/ inside)
#   BAG=/bags/<id>                        rosbag2 directory
#   OUT=/out/<id>                         where /result/* is recorded
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
cd /ws
# The organiser package.xml has no <maintainer>; catkin_pkg rejects it and
# colcon cannot build it on Humble. Build a copy with one added, only if absent.
if [ -d /org_msgs ]; then
  rm -rf /ws/src/tram_vehicle_msgs
  cp -r /org_msgs /ws/src/tram_vehicle_msgs
  _ensure=""
  for _c in "$(dirname "$0")/ensure_maintainer.sh" /opt/ensure_maintainer.sh /ws/scripts/ensure_maintainer.sh; do
    if [ -f "${_c}" ]; then _ensure="${_c}"; break; fi
  done
  if [ -z "${_ensure}" ]; then
    echo "ensure_maintainer.sh not found" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${_ensure}"
  ensure_maintainer /ws/src/tram_vehicle_msgs/package.xml
fi
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -n 3
source /ws/install/setup.bash
rm -rf "${OUT}"
ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
  -p gnss_init_window_s:="${GNSS_WINDOW:-3.0}" > /tmp/node.log 2>&1 &
NODE=$!
ros2 bag record -o "${OUT}" /result/velocity /result/position /result/diagnostics > /tmp/rec.log 2>&1 &
REC=$!
sleep 3
ros2 bag play "${BAG}" --rate "${RATE:-1.0}" > /tmp/play.log 2>&1
sleep 2
kill -INT "${REC}" || true
wait "${REC}" || true
# `ros2 run` does not forward SIGINT to the node binary.
pkill -INT -f lib/railbreak_backup_odometry/backup_odometry_node || true
wait "${NODE}" || true
cat /tmp/node.log | head -n 20
ros2 bag info "${OUT}" | sed -n '1,20p'

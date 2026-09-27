#!/usr/bin/env bash
# The node must not crash without its track assets: speed and relative odometry only.
# Mounts: /org_msgs, /ws/src/railbreak_backup_odometry, /bags/<one bag>.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
cd /ws
cp -r /org_msgs src/tram_vehicle_msgs
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
ensure_maintainer src/tram_vehicle_msgs/package.xml
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry 2>&1 | tail -n 3
source install/setup.bash
BIN=install/railbreak_backup_odometry/lib/railbreak_backup_odometry/backup_odometry_node
"${BIN}" --ros-args -p assets_dir:=/nonexistent -p gnss_wait_s:=5.0 > /tmp/node.log 2>&1 &
NODE=$!
sleep 2
timeout 30 ros2 topic echo /result/position --field pose.pose.position > /tmp/pos.txt 2>&1 &
BAG=$(ls -d /bags/*/ | head -n 1)
# Humble ros2 bag play has no --playback-duration. timeout exits 124; that is success here.
timeout 25 ros2 bag play "${BAG}" --rate 10 > /dev/null 2>&1 || true
sleep 3
if kill -0 "${NODE}" 2>/dev/null; then echo "node alive after replay"; else echo "NODE DIED"; fi
head -n 4 /tmp/node.log
echo "last positions:"; tail -n 8 /tmp/pos.txt
kill -INT "${NODE}" 2>/dev/null || true

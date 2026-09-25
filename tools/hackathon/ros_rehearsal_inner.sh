#!/usr/bin/env bash
# Runs inside the Humble image. Host wrapper: scripts/ros_rehearsal.sh
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install --packages-select tram_dr_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source /ws/install/setup.bash
set -u

BAGDIR="${BAG:-/data/bags/synth_rehearsal}"
echo "=== IMAGE / ROS ==="
echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "BAGDIR=${BAGDIR}"
echo "=== BAG INFO ==="
ros2 bag info "${BAGDIR}"

OUT=/tmp/rehearsal_record
rm -rf "${OUT}"
echo "=== RECORD + REPLAY ==="
timeout 22 ros2 bag record -o "${OUT}" /tram/state_estimate /tram/diagnostics &
RPID=$!
sleep 2
timeout 18 ros2 launch tram_dr_localization replay.launch.py \
  bag:="${BAGDIR}" use_sim_time:=true &
LPID=$!
sleep 4
echo "=== NODES ==="
ros2 node list || true
echo "=== TOPICS ==="
ros2 topic list -t || true
echo "=== STATE ONCE ==="
timeout 6 ros2 topic echo --once /tram/state_estimate || true
echo "=== DIAG ONCE ==="
timeout 6 ros2 topic echo --once /tram/diagnostics || true
kill "${RPID}" "${LPID}" 2>/dev/null || true
wait "${RPID}" 2>/dev/null || true
wait "${LPID}" 2>/dev/null || true

echo "=== RECORDED ==="
if [[ -d "${OUT}" ]]; then
  ros2 bag info "${OUT}" || true
  mkdir -p /data/bags/rehearsal_record
  rm -rf /data/bags/rehearsal_record
  cp -a "${OUT}" /data/bags/rehearsal_record
else
  echo "RECORD_MISSING"
fi
echo REHEARSAL_REPLAY_DONE

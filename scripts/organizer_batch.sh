#!/usr/bin/env bash
# Build once, then replay every bag under /bags through the node and record /result/*.
# Mounts: /org_msgs (organiser tram_vehicle_msgs), /ws/src/railbreak_backup_odometry
# (with assets/), /bags (bag directories), /out (results). RATE defaults to 1.0.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
cd /ws
rm -rf /ws/src/tram_vehicle_msgs
cp -r /org_msgs /ws/src/tram_vehicle_msgs
if ! grep -q "<maintainer" /ws/src/tram_vehicle_msgs/package.xml; then
  sed -i 's#<license>#<maintainer email="organiser@example.invalid">organiser</maintainer>\n  <license>#' \
    /ws/src/tram_vehicle_msgs/package.xml
fi
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -n 2
source /ws/install/setup.bash
NODE_BIN=/ws/install/railbreak_backup_odometry/lib/railbreak_backup_odometry/backup_odometry_node
for BAG in /bags/*/; do
  ID=$(basename "${BAG}")
  OUT="/out/${ID}"
  rm -rf "${OUT}"
  "${NODE_BIN}" --ros-args -r __node:=backup_odometry > "/out/${ID}.node.log" 2>&1 &
  NODE=$!
  T_START=$(date +%s.%N)
  ros2 bag record -o "${OUT}" /result/velocity /result/position /result/diagnostics \
    > /tmp/rec.log 2>&1 &
  REC=$!
  sleep 3
  ros2 bag play "${BAG}" --rate "${RATE:-1.0}" > /tmp/play.log 2>&1
  sleep 2
  kill -INT "${REC}" || true
  wait "${REC}" || true
  # Peak RSS and CPU time of the node process itself, from /proc.
  HWM=$(grep VmHWM "/proc/${NODE}/status" | tr -s ' ' | cut -d' ' -f2)
  TICKS=$(awk '{print $14 + $15}' "/proc/${NODE}/stat")
  HZ=$(getconf CLK_TCK)
  WALL=$(awk -v a="$(date +%s.%N)" -v b="${T_START}" 'BEGIN{printf "%.1f", a-b}')
  CPU=$(awk -v t="${TICKS}" -v h="${HZ}" 'BEGIN{printf "%.2f", t/h}')
  echo "${ID} peak_rss_kb=${HWM} cpu_s=${CPU} wall_s=${WALL}" | tee "/out/${ID}.res.txt"
  kill -INT "${NODE}" 2>/dev/null || true
  wait "${NODE}" || true
done

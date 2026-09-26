#!/usr/bin/env bash
# Adversarial GNSS queue: 100 in-window fixes beside wheels past the window.
# Host: docker run with the jury image, source and tram_vehicle_msgs mounted.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
mkdir -p /ws/src
rm -rf /ws/src/tram_vehicle_msgs /ws/src/railbreak_backup_odometry
cp -a /org_msgs /ws/src/tram_vehicle_msgs
cp -a /opt/src/railbreak_backup_odometry /ws/src/railbreak_backup_odometry
if ! grep -q "<maintainer" /ws/src/tram_vehicle_msgs/package.xml; then
  sed -i 's#<license>#<maintainer email="organiser@example.invalid">organiser</maintainer>\n  <license>#' \
    /ws/src/tram_vehicle_msgs/package.xml
fi
cd /ws
rm -rf /ws/build/railbreak_backup_odometry /ws/install/railbreak_backup_odometry
colcon build --packages-select tram_vehicle_msgs railbreak_backup_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source /ws/install/setup.bash
set -u

one() {
  local first="$1" drain="$2" rep="$3"
  timeout 12 ros2 run railbreak_backup_odometry backup_odometry_node --ros-args \
    -p gnss_subscribe_first:="$first" \
    -p drain_gnss_queue:="$drain" \
    >"/tmp/gnss_${first}_${drain}_${rep}.log" 2>&1 &
  local pid=$!
  sleep 1
  python3 /opt/scripts/gnss_queue_stress.py
  sleep 0.5
  kill "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
  local line
  line=$(grep -o 's0 [-0-9.]* m (snap [-0-9.]*), d0 [-0-9.]* m, [0-9]* fixes' "/tmp/gnss_${first}_${drain}_${rep}.log" | tail -1 || true)
  echo "first=${first} drain=${drain} rep=${rep} ${line:-NO_INIT}"
}

for first in false true; do
  for drain in true false; do
    for rep in 1 2 3 4 5; do
      one "$first" "$drain" "$rep"
    done
  done
done

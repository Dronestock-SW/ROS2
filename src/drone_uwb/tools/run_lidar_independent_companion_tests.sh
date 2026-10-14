#!/usr/bin/env bash
# Isolated ROS/mock-FC checks. Does not connect to physical ROS domain 2.
set -eo pipefail
workspace=${1:?workspace required}
native=${2:?native binary directory required}
evidence=${3:?new evidence directory required}
deps=${4:?Python dependency directory required}
cd "$workspace"
workspace=$PWD
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export PYTHONPATH="$workspace/src/drone_mission:$workspace/src/drone_platform_link:$workspace/src/drone_demo:$workspace/src/drone_uwb:$workspace/src/sangwon_AI/tests:$deps:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export SANGWON_CPP_AI_BINARY="$native/sangwon_field_mission_ai"
test -x "$SANGWON_CPP_AI_BINARY"
test -x "$native/sangwon_field_executor"
mkdir "$evidence"
python3 -m pytest -q src/drone_mission/test \
  --ignore=src/drone_mission/test/test_local_flight_ros.py \
  src/drone_platform_link/test src/drone_demo/test \
  src/drone_uwb/test/processing/test_lidar_motion.py \
  src/drone_uwb/test/integration/test_lidar2d_fc_setup.py \
  src/drone_uwb/test/integration/test_lidar_yaw_timing.py > "$evidence/python.log" 2>&1
tail -2 "$evidence/python.log"
python3 src/sangwon_AI/tests/field_executor_contracts.py \
  "$native/sangwon_field_executor_replay" > "$evidence/contracts.log" 2>&1
tail -4 "$evidence/contracts.log"
python3 src/sangwon_AI/tests/field_executor_ros.py \
  "$native/sangwon_field_executor" "$evidence/ros" \
  separate_lidar_takeoff,separate_lidar_uwb_recovery,separate_lidar_loss,separate_lidar_malformed \
  > "$evidence/ros.log" 2>&1
cat "$evidence/ros.log"
sha256sum "$native/sangwon_field_executor" "$SANGWON_CPP_AI_BINARY" \
  "$native/sangwon_field_executor_replay" > "$evidence/binaries.sha256"

#!/usr/bin/env bash
set -eo pipefail
bundle=$(cd -- "$(dirname -- "$0")" && pwd)
source /opt/ros/humble/setup.bash
export PYTHONPATH="$bundle/python-deps:$bundle/source:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec python3 -m drone_uwb.integration.ros.lidar_velocity_bridge \
  --config "$bundle/source/config/runtime/lidar_velocity_field.json" \
  --record-directory "$bundle/${1:?new capture directory name}" --seconds "${2:-40}"

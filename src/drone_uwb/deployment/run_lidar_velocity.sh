#!/usr/bin/env bash
set -eo pipefail
: "${ROS2_WORKSPACE:?}" "${LIDAR_VELOCITY_CONFIG:?}"
source /opt/ros/humble/setup.bash
source "$ROS2_WORKSPACE/install/setup.bash"
export MAVLINK20=1 ROS_LOCALHOST_ONLY=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="${LIDAR_PYTHON_DEPS:+$LIDAR_PYTHON_DEPS:}${PYTHONPATH:-}"
case "${MANUAL_TAG:-B}" in A) export ROS_DOMAIN_ID=1;; B) export ROS_DOMAIN_ID=2;; *) exit 64;; esac
# This node only subscribes to the running sensor stack and publishes measured
# LiDAR observations. It never owns /dev/pixhawk, sets a mode or starts a mission.
exec python3 -m drone_uwb.integration.ros.lidar_velocity_bridge --config "$LIDAR_VELOCITY_CONFIG" "$@"

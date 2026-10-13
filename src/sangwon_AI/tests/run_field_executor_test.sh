#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 MAVLINK20=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
SANGWON_EXECUTOR_SOURCE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
export PYTHONPATH="$SANGWON_EXECUTOR_SOURCE/drone_mission:$SANGWON_EXECUTOR_SOURCE/drone_uwb:$SANGWON_EXECUTOR_SOURCE/drone_platform_link:$SANGWON_EXECUTOR_SOURCE/drone_demo:$SANGWON_EXECUTOR_SOURCE/sangwon_AI/tests:${PYTHONPATH:-}"
exec /usr/bin/python3 "$SANGWON_EXECUTOR_SOURCE/sangwon_AI/tests/field_executor_ros.py" "$1"

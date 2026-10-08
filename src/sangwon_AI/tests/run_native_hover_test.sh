#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec /usr/bin/python3 "$SANGWON_ROOT/tests/native_hover_ros.py" "$1"

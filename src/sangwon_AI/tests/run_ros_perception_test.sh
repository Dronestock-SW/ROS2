#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$SANGWON_ROOT/python:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$SANGWON_ROOT/tests/ros_perception_integration.py"

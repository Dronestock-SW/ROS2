#!/usr/bin/env bash
set -eo pipefail
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="$SANGWON_ROOT/python:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
SANGWON_CONFIG="$SANGWON_ROOT/config/perception.observe.json"
if [[ -f "$SANGWON_ROOT/config/perception.local.json" ]]; then
  SANGWON_CONFIG="$SANGWON_ROOT/config/perception.local.json"
fi
exec /usr/bin/python3 -u "$SANGWON_ROOT/ops/perception_monitor.py" --config "$SANGWON_CONFIG"

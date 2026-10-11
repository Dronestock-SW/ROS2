#!/usr/bin/env bash
set -eo pipefail
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
set -u
SANGWON_CONFIG="$SANGWON_ROOT/config/px4.observe.json"
if [[ -f "$SANGWON_ROOT/config/px4.local.json" ]]; then
  SANGWON_CONFIG="$SANGWON_ROOT/config/px4.local.json"
fi
source "$SANGWON_ROOT/deployment/binary_path.sh"
exec "$(sangwon_binary sangwon_px4_observer)" --root "$SANGWON_ROOT" --config "$SANGWON_CONFIG"

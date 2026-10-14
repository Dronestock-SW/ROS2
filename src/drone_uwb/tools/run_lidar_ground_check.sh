#!/usr/bin/env bash
# Bounded measured-input check. No ARM, flight command or parameter write.
set -eo pipefail
workspace=$(cd -- "$(dirname -- "$0")/../../.." && pwd)
profile=${1:?ground_static config required}
evidence=${2:?new evidence directory required}
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
if [[ -n ${DRONESTOCK_PYTHON_DEPS:-} ]]; then
  [[ -d "$DRONESTOCK_PYTHON_DEPS" ]]
  export PYTHONPATH="$DRONESTOCK_PYTHON_DEPS:${PYTHONPATH:-}"
fi
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
python3 - "$profile" <<'PY'
import json, sys
import pymavlink
from drone_uwb.processing.lidar_motion import LidarMotionSettings
cfg = LidarMotionSettings(**json.load(open(sys.argv[1], encoding='utf-8'))['settings'])
if cfg.floor_policy != 'ground_static' or cfg.tag_id != '6' or not cfg.enabled:
    raise SystemExit('enabled_tag6_ground_static_profile_required')
if cfg.geometry_gate() != 'ready':
    raise SystemExit(cfg.geometry_gate())
PY
mkdir "$evidence"
cp "$profile" "$evidence/profile.json"
sha256sum "$profile" > "$evidence/profile.sha256"
python3 -m drone_uwb.integration.ros.lidar_velocity_bridge \
  --same-user-dds --config "$profile" --seconds 45 \
  --record-directory "$evidence/sender" > "$evidence/sender.log" 2>&1 &
sender_pid=$!
trap 'kill -INT "$sender_pid" 2>/dev/null || true; wait "$sender_pid" 2>/dev/null || true' EXIT
sleep 12
python3 "$workspace/src/drone_uwb/tools/lidar_hardware_readback.py" \
  --same-user-dds --ops-directory "$workspace/src/sangwon_AI/ops" \
  --output "$evidence/fc-during-01.json" > "$evidence/fc-during-01.log"
sleep 10
python3 "$workspace/src/drone_uwb/tools/lidar_hardware_readback.py" \
  --same-user-dds --ops-directory "$workspace/src/sangwon_AI/ops" \
  --output "$evidence/fc-during-02.json" > "$evidence/fc-during-02.log"
wait "$sender_pid"
trap - EXIT
python3 "$workspace/src/drone_uwb/tools/lidar_hardware_readback.py" \
  --same-user-dds --ops-directory "$workspace/src/sangwon_AI/ops" \
  --output "$evidence/fc-after-stop.json" > "$evidence/fc-after-stop.log"
printf 'Bounded ground check finished: %s\n' "$evidence"

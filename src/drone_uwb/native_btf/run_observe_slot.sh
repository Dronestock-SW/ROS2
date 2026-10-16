#!/usr/bin/env bash
# Invoked by the existing sensor service only after a fresh DISARM deployment gate.
set -eo pipefail
: "${BTF_NATIVE_SLOT:?}" "${ROS2_WORKSPACE:?}" "${MANUAL_TAG:?}" "${MANUAL_ANCHORS:?}"
source /opt/ros/humble/setup.bash
source "$ROS2_WORKSPACE/install/setup.bash"
export PYTHONPATH="${MANUAL_PYTHON_DEPS:+$MANUAL_PYTHON_DEPS:}${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
case "$MANUAL_TAG" in A) export ROS_DOMAIN_ID=1;; B) export ROS_DOMAIN_ID=2;; *) exit 64;; esac
for device in /dev/pixhawk /dev/uwb; do
  test -e "$device" || { echo "Waiting for $device"; exit 75; }
  if fuser "$device" >/dev/null 2>&1; then echo "Port already owned: $device"; exit 75; fi
done
test -f "$BTF_NATIVE_SLOT/field-btf-cpp.json"
exec ros2 launch "$BTF_NATIVE_SLOT/source/src/drone_uwb/launch/manual_observe.launch.py" \
  "tag:=$MANUAL_TAG" "btf_config:=$BTF_NATIVE_SLOT/field-btf-cpp.json" \
  "anchor_file:=$MANUAL_ANCHORS" "btf_python_path:=$BTF_NATIVE_SLOT/source/src/drone_uwb"

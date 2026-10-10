#!/usr/bin/env bash
set -eo pipefail
: "${ROS2_WORKSPACE:?}" "${MANUAL_CAPTURE_ROOT:?}" "${MANUAL_TAG:?}" "${MANUAL_BTF_CONFIG:?}" "${MANUAL_ANCHORS:?}"
cd "$ROS2_WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="${MANUAL_PYTHON_DEPS:+$MANUAL_PYTHON_DEPS:}${PYTHONPATH:-}"
export ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
case "$MANUAL_TAG" in A) export ROS_DOMAIN_ID=1;; B) export ROS_DOMAIN_ID=2;; *) exit 64;; esac
mkdir -p "$MANUAL_CAPTURE_ROOT"
case "${1:-}" in
  observe)
    # Do not steal either serial device from the preserved original workspace.
    for device in /dev/pixhawk /dev/uwb; do
      test -e "$device" || { echo "Waiting for $device"; exit 75; }
      if fuser "$device" >/dev/null 2>&1; then echo "Port already owned: $device"; exit 75; fi
    done
    boot=$(cat /proc/sys/kernel/random/boot_id)
    probe="$MANUAL_CAPTURE_ROOT/fc-before-$boot.json"
    if ! test -e "$probe"; then
      python3 src/sangwon_AI/ops/px4_sensor_readback.py --profile manual-flight --output "$probe" || true
    fi
    exec ros2 launch drone_uwb manual_observe.launch.py "tag:=$MANUAL_TAG" \
      "btf_config:=$MANUAL_BTF_CONFIG" "anchor_file:=$MANUAL_ANCHORS"
    ;;
  capture)
    extra=()
    if [[ "${MANUAL_CAPTURE_LIDAR:-0}" == 1 ]]; then extra+=(--with-lidar); fi
    exec python3 -m drone_uwb.integration.boot_capture --root "$MANUAL_CAPTURE_ROOT" \
      --tag "$MANUAL_TAG" --config "$MANUAL_BTF_CONFIG" --config "$MANUAL_ANCHORS" \
      --source-revision "$(git rev-parse HEAD)" "${extra[@]}"
    ;;
  *) echo 'Expected observe or capture'; exit 64;;
esac

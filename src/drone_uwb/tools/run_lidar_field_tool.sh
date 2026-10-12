#!/usr/bin/env bash
set -eo pipefail
bundle=$(cd -- "$(dirname -- "$0")" && pwd)
source /opt/ros/humble/setup.bash
export PYTHONPATH="$bundle/python-deps:$bundle/source:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1 MAVLINK20=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
tool=${1:?tool filename required}
shift
case "$tool" in
  lidar_mount_calibration.py|probe_lidar_hardware.py|lidar_hardware_readback.py) ;;
  *) echo 'Unsupported field tool'; exit 64;;
esac
exec python3 "$bundle/$tool" "$@"

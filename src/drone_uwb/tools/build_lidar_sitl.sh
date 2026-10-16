#!/usr/bin/env bash
# A private checkout and a Python environment are required; no service changes.
set -euo pipefail
root=$(realpath "$1")
python_exe=$2
[[ $python_exe = /* && -x $python_exe ]]
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PATH="$(dirname "$python_exe"):$PATH"
board=px4_sitl_lidar
extra=()
if [[ ${3:-} = lockstep ]]; then
    board=px4_sitl_evbench
    extra=(--lockstep)
fi
"$python_exe" "$script_dir/prepare_lidar_sitl.py" --px4-root "$root" "${extra[@]}"
cd "$root"
git submodule update --init --recursive src/lib/heatshrink/heatshrink src/modules/mavlink/mavlink src/lib/events/libevents
make "$board" -j4
if [[ ${3:-} = lockstep ]]; then
    grep -q -- '-DENABLE_LOCKSTEP_SCHEDULER' "build/$board/compile_commands.json"
fi

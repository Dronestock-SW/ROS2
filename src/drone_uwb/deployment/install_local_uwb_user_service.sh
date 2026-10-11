#!/usr/bin/env bash
set -euo pipefail
workspace=${1:?workspace absolute path required}
configuration=${2:?configuration absolute path required}
[[ $workspace == /home/"$(id -un)"/* && -f $workspace/install/setup.bash ]]
[[ $configuration == /home/"$(id -un)"/* && -f $configuration ]]
# Validate the exact inherited dependency path before replacing a working service.
bash -c 'source /opt/ros/humble/setup.bash && source "$1/install/setup.bash" &&
    python3 -c "from pymavlink import mavutil; from mavros.mavlink import convert_to_rosmsg"' \
    _ "$workspace"
export XDG_RUNTIME_DIR=/run/user/$(id -u)
mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/dronestock-local-uwb"
install -m 0644 "$workspace/src/drone_uwb/deployment/dronestock-local-uwb.service" \
    "$HOME/.config/systemd/user/dronestock-local-uwb.service"
printf 'UWB_WORKSPACE=%s\nUWB_LOCAL_CONFIG=%s\nPYTHONPATH=%s\n' \
    "$workspace" "$configuration" "${PYTHONPATH:-}" \
    > "$HOME/.config/dronestock-local-uwb/environment"
systemctl --user daemon-reload
systemctl --user enable --now dronestock-local-uwb.service
systemctl --user status dronestock-local-uwb.service --no-pager

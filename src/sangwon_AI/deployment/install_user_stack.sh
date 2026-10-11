#!/usr/bin/env bash
set -euo pipefail
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
SANGWON_EXPECTED="$(getent passwd "$(id -un)" | cut -d: -f6)/Desktop/ROS2/src/sangwon_AI"
[[ "$SANGWON_ROOT" == "$SANGWON_EXPECTED" ]] || { echo 'Review installation root'; exit 2; }
source "$SANGWON_ROOT/deployment/binary_path.sh"
sangwon_binary sangwon_companiond >/dev/null
sangwon_binary sangwon_px4_observer >/dev/null
"$SANGWON_ROOT/.venv/bin/python" -c 'import websockets; assert websockets.__version__ == "15.0.1"'
for SANGWON_NAME in sangwon-core.service sangwon-web-adapter.service sangwon-health-monitor.service sangwon-perception-monitor.service sangwon-px4-observer.service sangwon-autonomy.target; do
  SANGWON_DEST="$HOME/.config/systemd/user/$SANGWON_NAME"
  SANGWON_SOURCE="$SANGWON_ROOT/deployment/$SANGWON_NAME"
  if [[ -e "$SANGWON_DEST" || -L "$SANGWON_DEST" ]]; then
    [[ "$(readlink -f -- "$SANGWON_DEST")" == "$SANGWON_SOURCE" ]] || { echo "Preserving different unit: $SANGWON_DEST"; exit 3; }
  fi
  systemctl --user link "$SANGWON_SOURCE"
done
systemctl --user daemon-reload
systemd-analyze --user verify "$SANGWON_ROOT/deployment/sangwon-core.service" "$SANGWON_ROOT/deployment/sangwon-web-adapter.service" "$SANGWON_ROOT/deployment/sangwon-health-monitor.service" "$SANGWON_ROOT/deployment/sangwon-perception-monitor.service" "$SANGWON_ROOT/deployment/sangwon-px4-observer.service" "$SANGWON_ROOT/deployment/sangwon-autonomy.target"
systemctl --user enable --now sangwon-autonomy.target
# Pull newly added observers into an already running target without restarting
# healthy services. Linking/reloading an active target alone is insufficient.
systemctl --user start sangwon-core.service sangwon-web-adapter.service sangwon-health-monitor.service sangwon-perception-monitor.service sangwon-px4-observer.service
systemctl --user is-active sangwon-core.service sangwon-web-adapter.service sangwon-health-monitor.service sangwon-perception-monitor.service sangwon-px4-observer.service
loginctl show-user "$(id -un)" -p Linger
echo 'Host observation only. Cold-boot verification is a separate test. No automatic START.'

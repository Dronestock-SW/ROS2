#!/usr/bin/env bash
set -euo pipefail

SANGWON_DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
SANGWON_EXPECTED_ROOT="$(getent passwd "$(id -un)" | cut -d: -f6)/Desktop/ROS2/src/sangwon_AI"
if [[ "$SANGWON_DEPLOY_ROOT" != "$SANGWON_EXPECTED_ROOT" ]]; then
  echo "Unit path must be reviewed for this installation: $SANGWON_DEPLOY_ROOT" >&2
  exit 2
fi
SANGWON_UNIT="$SANGWON_DEPLOY_ROOT/deployment/sangwon-health-monitor.service"
SANGWON_USER_UNIT="$HOME/.config/systemd/user/sangwon-health-monitor.service"
if [[ -e "$SANGWON_USER_UNIT" || -L "$SANGWON_USER_UNIT" ]]; then
  if [[ "$(readlink -f -- "$SANGWON_USER_UNIT")" != "$SANGWON_UNIT" ]]; then
    echo "Existing different service is preserved: $SANGWON_USER_UNIT" >&2
    exit 3
  fi
fi
PYTHONDONTWRITEBYTECODE=1 python3 "$SANGWON_DEPLOY_ROOT/tests/test_health_monitor.py"
systemd-analyze --user verify "$SANGWON_UNIT"
systemctl --user link "$SANGWON_UNIT"
systemctl --user daemon-reload
systemctl --user enable --now sangwon-health-monitor.service
systemctl --user is-active sangwon-health-monitor.service

# Do not prompt for or collect administrator passwords.
if loginctl --no-ask-password enable-linger "$(id -un)"; then
  loginctl show-user "$(id -un)" -p Linger
else
  echo "User service is active; login-independent boot still needs administrator enable-linger."
fi
python3 "$SANGWON_DEPLOY_ROOT/ops/health_monitor.py" --status

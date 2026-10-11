#!/usr/bin/env bash
# Source from Bash, or use run_env.sh for commands/services.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Use: source deployment/env.sh' >&2
  exit 2
fi
_sangwon_activate() {
  local task_root
  task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)" || return
  if [[ "${ROS_DOMAIN_ID:-1}" != 1 ]]; then
    echo 'This unit uses ROS_DOMAIN_ID=1; another domain is already set.' >&2
    return 2
  fi
  if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$task_root/.venv/bin/activate" ]]; then
    echo 'ROS Humble or local .venv missing. Run ops/setup_python_env.py first.' >&2
    return 2
  fi
  export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1 ROS_DOMAIN_ID=1
  source /opt/ros/humble/setup.bash || return
  if [[ -f "$task_root/.build/colcon-install/local_setup.bash" ]]; then
    source "$task_root/.build/colcon-install/local_setup.bash" || return
  fi
  source "$task_root/.venv/bin/activate" || return
  export SANGWON_AI_ROOT="$task_root"
  export SANGWON_LOG_PYTHON="$task_root/.venv-log/bin/python"
}
if _sangwon_activate; then
  unset -f _sangwon_activate
else
  unset -f _sangwon_activate
  return 2
fi

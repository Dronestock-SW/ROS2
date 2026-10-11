#!/usr/bin/env bash
# Source after defining SANGWON_ROOT. Supports ROS2 workspace and legacy builds.
sangwon_binary() {
  local name="$1"
  local prefix="$SANGWON_ROOT/../../install/sangwon_ai_replay"
  if [[ -x "$prefix/bin/$name" ]]; then
    printf '%s\n' "$prefix/bin/$name"
  elif [[ -x "$SANGWON_ROOT/.build/colcon-install/sangwon_ai_replay/bin/$name" ]]; then
    printf '%s\n' "$SANGWON_ROOT/.build/colcon-install/sangwon_ai_replay/bin/$name"
  else
    echo "Build sangwon_ai_replay in the ROS2 workspace first" >&2
    return 2
  fi
}

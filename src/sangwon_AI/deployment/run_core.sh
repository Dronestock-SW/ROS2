#!/usr/bin/env bash
set -euo pipefail
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
SANGWON_CONFIG="$SANGWON_ROOT/config/companion.host.json"
if [[ -f "$SANGWON_ROOT/config/companion.local.json" ]]; then
  SANGWON_CONFIG="$SANGWON_ROOT/config/companion.local.json"
fi
source "$SANGWON_ROOT/deployment/binary_path.sh"
exec "$(sangwon_binary sangwon_companiond)" \
  --config "$SANGWON_CONFIG"

#!/usr/bin/env bash
set -euo pipefail
SANGWON_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export PYTHONPATH="$SANGWON_ROOT/python"
export PYTHONDONTWRITEBYTECODE=1
if [[ -f "$SANGWON_ROOT/config/web.local.json" ]]; then
  exec "$SANGWON_ROOT/.venv/bin/python" -m sangwon_web.adapter \
    --config "$SANGWON_ROOT/config/web.local.json" --identity "$SANGWON_ROOT/.runtime/private/device.env"
fi
exec "$SANGWON_ROOT/.venv/bin/python" -m sangwon_web.adapter --config "$SANGWON_ROOT/config/web.observe.json"

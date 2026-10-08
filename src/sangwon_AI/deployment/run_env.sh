#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
if [[ $# -eq 0 ]]; then
  echo 'Usage: bash deployment/run_env.sh command [arguments...]' >&2
  exit 2
fi
exec "$@"

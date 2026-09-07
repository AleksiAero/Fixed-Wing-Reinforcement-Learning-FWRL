#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
fwrl_python="${FWRL_VENV:-$HOME/.local/share/fwrl/venv}/bin/python"
[[ -x "$fwrl_python" ]] || { echo 'Run bash scripts/setup_wsl.sh first' >&2; exit 1; }
exec "$fwrl_python" "$@"

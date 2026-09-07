#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ -n "${FWRL_VENV:-}" ]]; then
  fwrl_venv="$FWRL_VENV"
elif [[ -n "${VIRTUAL_ENV:-}" ]]; then
  fwrl_venv="$VIRTUAL_ENV"
elif [[ -x "$PWD/.venv-linux/bin/python" ]]; then
  fwrl_venv="$PWD/.venv-linux"
else
  fwrl_venv="$HOME/.local/share/fwrl/venv"
fi
fwrl_python="$fwrl_venv/bin/python"
[[ -x "$fwrl_python" ]] || { echo 'Run bash scripts/setup_linux.sh first (or set FWRL_VENV).' >&2; exit 1; }
exec "$fwrl_python" "$@"

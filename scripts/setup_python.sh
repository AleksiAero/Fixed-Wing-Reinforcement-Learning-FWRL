#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
fwrl_venv="${FWRL_VENV:-$HOME/.local/share/fwrl/venv}"
python3 -m venv --system-site-packages "$fwrl_venv"
"$fwrl_venv/bin/python" -m pip install --upgrade pip
"$fwrl_venv/bin/python" -m pip install -e '.[test]'
"$fwrl_venv/bin/python" -m pip install torch --index-url https://download.pytorch.org/whl/cu130

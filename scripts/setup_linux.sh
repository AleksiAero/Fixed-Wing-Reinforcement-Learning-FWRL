#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# No root, ROS, WSL or GPU required; system Python must support venv.
fwrl_venv="${FWRL_VENV:-$PWD/.venv-linux}"
python3 -m venv "$fwrl_venv"
"$fwrl_venv/bin/python" -m pip install --upgrade pip
"$fwrl_venv/bin/python" -m pip install -e '.[test]'
"$fwrl_venv/bin/python" -m pip install 'torch>=2.7,<3' --index-url "${FWRL_TORCH_INDEX_URL:-https://download.pytorch.org/whl/cpu}"
printf 'Ready. Run: bash scripts/python.sh -m pytest -q\n'

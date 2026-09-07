# Native Linux development

WSL, Windows, ROS and MATLAB are not required for core simulation, training or the browser viewer. Use Python 3.10+ with venv support. Blender is optional for the browser workflow; the Tk desktop app additionally requires your distribution's Python Tk package.

## Install

On Ubuntu/Debian, install prerequisites with `sudo apt install python3 python3-venv python3-pip git`. Add `python3-tk blender` if you want the desktop app and Blender viewport. Other distributions should install equivalent packages using their package manager.

```bash
git clone https://github.com/AleksiAero/Fixed-Wing-Reinforcement-Learning-FWRL.git
cd Fixed-Wing-Reinforcement-Learning-FWRL
bash scripts/setup_linux.sh
bash scripts/python.sh -m pytest -q
```

Setup creates `.venv-linux` inside the checkout and installs CPU PyTorch by default. It does not change system packages or GPU drivers. Override `FWRL_VENV` to choose another virtual environment. The Python wrapper prefers that override, then an activated environment, then `.venv-linux`, and finally the historical WSL environment at `~/.local/share/fwrl/venv`.

For GPU development, install the PyTorch build matching your hardware/driver into that environment. `FWRL_TORCH_INDEX_URL` can override the setup script's CPU wheel index. Training/evaluation default to `--device auto`, which selects CUDA when PyTorch reports it available and CPU otherwise. `--device cpu` explicitly disables GPU use; requesting unavailable CUDA fails clearly. The GUI and Blender launcher also accept `FWRL_DEVICE=cpu` or `FWRL_DEVICE=cuda`.

## Live browser workflow (no Blender required)

From the repository root, start training in one terminal:

```bash
bash scripts/python.sh -m fwrl.training --device auto --envs 8 --steps 1000000 --realtime --output runs/linux-dev --live-state runs/live/linux-dev.json
```

Once telemetry has appeared, start the viewer in a second terminal:

```bash
bash scripts/python.sh -m fwrl.viewer --live-state runs/live/linux-dev.json
```

Open `http://127.0.0.1:8765/` in a WebGL-capable browser. The server listens on localhost only; use `--port 8766` if another viewer is running. Three.js currently loads from a CDN, so the browser needs internet access. The viewer shows actual training state, not a replay. CPU training may run slower than wall time; reduce the environment count when needed.

Each new training run needs a fresh output directory. To request a checkpoint and graceful stop, run `touch runs/linux-dev/STOP`. A `PAUSE` file pauses training; remove it to resume, or use the viewer's pause button. Stop the viewer with Ctrl+C. Checkpoints and generated scenes remain local and are excluded from Git.

## Blender or desktop GUI

```bash
bash scripts/python.sh -m fwrl.session
# Or open the desktop workbench:
bash scripts/python.sh -m fwrl.app
```

Blender must be on PATH; its scripts use Blender 4.x APIs. A desktop display is required for these UI commands. Blender starts a fresh random policy, while the browser workflow above can also be run on a headless machine. Core training does not require a display.

ROS integration remains optional and separate: existing ROS scripts target ROS 2 Jazzy on Ubuntu 24.04. Do not run `setup_wsl.sh` for the basic native Linux workflow.

## Validation scope

CPU training, telemetry/viewer HTTP serving, and the test suite are exercised in Ubuntu Linux under WSL with native Linux processes and paths. This is not a claim that every Linux distribution, GPU driver or physical Linux desktop has been tested. CI also runs the unit suite on Ubuntu with CPU PyTorch.

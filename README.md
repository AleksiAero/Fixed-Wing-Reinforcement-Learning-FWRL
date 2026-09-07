# SYNTHOSAR Flight Lab

## Current training cycle

Use `python -m fwrl.learn --device auto --envs 64 --steps 10000000 --output runs/new-training --live-state runs/live/mountain-zero-new.json` from the repository root (on Linux, prefix with `bash scripts/python.sh` instead of `python`). In another terminal, run `python -m fwrl.viewer --live-state runs/live/mountain-zero-new.json`. Blender's Start learning action uses the same curriculum trainer.

Every curriculum run starts with random weights: no resume, demonstrations, baseline guidance or flight controller. Stages are 70 mph straight flight, 80 mph gentle turns, then the mountain course at 100, 150 and 200 mph. Physics and the 65 mph ground launcher stay unchanged. Promotion needs two consecutive fixed-seed evaluations with at least 75% completions, at most 10% stall samples and at least 60% of airborne samples within 20% of the stage speed, plus 50,000 training transitions at that stage.

Post-launch rewards encourage forward progress at the stage speed and penalize stalls, underspeed, angular-rate excursions and abrupt commands. Hoop/crash/timeout rewards remain positive/negative as before. PPO scales rewards by 0.01, uses gamma 0.999, GAE lambda 0.98 and 512-step rollouts, excludes rail-constrained actions from actor updates, clips updates and monitors approximate KL. Observations include eight nearest obstacles, terrain clearance/preview and aircraft state, with running normalization saved inside each checkpoint.

`evaluations.jsonl` records deterministic fixed-seed tests every five updates, independent of the training environments. `metrics.jsonl` records stage, crashes, gates, loss, KL and actual new transitions per second. The browser/Blender view is **live policy evaluation during training**, not a baked replay and not part of the PPO batch. It refreshes weights between flights and runs at real time while training runs unpaced. Its Pause button pauses both training and the live flight. Put a `STOP` file in the run folder to stop and save.

The older `fwrl.training` entry point remains available for legacy experiments; older sections below describe those workflows and historical validation, not the current curriculum defaults.


**Native Linux developers:** follow [the Linux setup guide](docs/linux.md). Core training and the live browser viewer work without Windows, WSL, ROS or a GPU. Start with `bash scripts/setup_linux.sh`; CPU/CUDA selection is automatic.

Latest airframe: reference-inspired long fuselage, 2.106 m swept wings and conventional fixed tail attached directly to the body. The model retains elevon control. Reference sea-level 1g stall speed is **45 mph**, using 0.31 m² wing area; launch speed is **65 mph**. Stall is modeled through angle of attack, lift loss and increased drag, not an arbitrary speed cutoff. Aircraft dimensions and coefficients are design assumptions, not measurements from the reference image.

The source includes the Blender scene generator and a live Windows browser viewer. Generated `.blend` files, local virtual environments and training checkpoints are excluded from Git; rebuild the scene using the setup commands below.


## Elevons, stalls and hoop deadlines

The active Mountain Gauntlet model now uses a quaternion rigid-body state with translational velocity, rotational inertia, body angular rates, left/right elevon deflections and throttle. The policy directly commands both elevons and the motor. Common elevon deflection produces pitch moments; differential deflection produces roll moments. Surface deflection, servo slew and motor response are limited. There is no autopilot or pretrained controller.

The reference **sea-level, 1g stall speed is 45 mph**, using the supplied speed to infer CLmax for an assumed 5 kg mass and 0.31 m² wing area. Lift depends on angle of attack; beyond the assumed 15° critical angle lift falls and drag rises. Stall speed increases with load and altitude. A brief zoom climb can still occur, but static thrust is limited to 45 N versus 49.05 N weight, so sustained vertical powered flight is unavailable. These are assumed aerodynamic coefficients, not measured airframe validation. The older model allowed 100 N thrust and direct lift commands, which caused the unrealistic behavior.

The launch rail is inclined **20°**, releases at **65 mph**, and is aligned horizontally with an **8 m radius first hoop roughly 94 m horizontally from the launch start**, lowered 13 m below the extended launcher axis. Reaching it requires sustained airborne control. The remaining mountain hoops retain their harder layout. Passing each hoop gives **+100**, a crash gives **−150**, and missing the next hoop deadline gives **−150**; completion adds **+500**. Small progress/speed terms also apply. The first gate allows 20 seconds including launch; subsequent deadlines allow twice the travel time at 50 mph, plus a turn margin and climb allowance (approximately 176–381 s). The overall episode limit covers the sum of these budgets. Each valid crossing resets the next-gate timer. Failures reset the complete aircraft/servo state and timers to rest on the launcher. This penalty prevents farming a positive return by repeatedly taking only the easy first gate.

The live Windows viewer shows quaternion attitude, moving elevons, angle of attack/stall indication, time left and current reward. New sessions start from a random network. An explicit `--resume` can retain a compatible elevon checkpoint when changing deadlines or visuals; older policies with incompatible controls and observations are rejected. Physics integrates at 100 Hz, with 20 Hz outer steps; live display can slow if computation cannot keep pace.

The browser and Blender share a dark flying-wing mesh inspired by the supplied photograph: long swept wings, a tapered fuselage, a directly attached conventional tail, rear pusher propeller and separate elevons. The viewer’s **Aircraft view** button shows it close up. This visual change does not infer new aerodynamic coefficients from the photograph.

Equations/reference behavior: [NASA lift equation](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/), [NASA stall explanation](https://www.grc.nasa.gov/www/k-12/Aero2000/studweb/2-3-2i2.html), [FAA flight controls](https://www.faa.gov/regulationspolicies/handbooksmanuals/aviation/phak/chapter-6-flight-controls).


## Live learning from zero — Mountain Gauntlet

The default app environment is now **Mountain Gauntlet**, an open 20-hoop course with reversing turns, long climbs, descents and mountain terrain. Gates range from roughly 110 to 1,030 m altitude. The 50 mph launch rail and 200 mph objective remain.

Run `./Start-Highspeed.ps1`, or select **Run simulation** / **Start training** in the updated Flight Lab app. Blender opens and starts a new randomly initialized PPO policy. Its viewport follows the **current training aircraft**, receiving live positions at 20 Hz. There is no baked animation, demonstration trajectory, loaded checkpoint, waypoint controller, altitude hold, speed hold, or learned residual around an expert. Previous guided checkpoints remain on disk but are never loaded by this workflow.

The network now controls left and right elevons and throttle through the rigid-body model described above. It observes vehicle state, wind, upcoming gates and terrain-clearance samples. Rewards describe progress, gate passage, collisions and the speed objective; they do not supply actions. Early crashes and repeated launches are expected, and successful mountain-course flight has **not** been demonstrated.

The viewer shows one of 64 simultaneous training environments at 1× simulation speed, including automatic launcher resets. PPO updates may briefly hold the view; the HUD says when the policy is updating. Use the **Flight Lab** sidebar in Blender to pause/resume or stop and save. Space in the viewport toggles pause. Closing the Linux Blender viewer requests a checkpoint stop. Each new Start creates a fresh random policy; pause/resume keeps learning in the same session. The default budget is 10 million transitions (roughly a few hours at real-time pacing), with checkpoints after every update.

Each session writes its own atomic current-state JSON and log under `runs/live/`; checkpoints and metrics are in `runs/mountain-zero-*/`. Directly opening `worlds/mountain_gauntlet.blend` shows the unanimated scene; use the launcher for live training, or run its embedded `Watch Learning.py` text and click Start learning from zero. Training remains on CUDA in WSL; Blender renders the latest state as it happens.



A local desktop workbench for fixed-wing **waypoint navigation and obstacle-avoidance research**. It includes CSV trajectory analysis, a force-based fixed-wing simulator, ROS 2 and MATLAB reference controllers, a shared Blender world, and batched NVIDIA GPU reinforcement learning.

## Start the application

From this folder in PowerShell:

```powershell
.\Start-FlightLab.ps1
```

Alternatively run `.\.venv\Scripts\python.exe -m fwrl.app`. The light desktop interface has an **Environment**, **Flight analysis**, and **Training** sidebar. Use **Run simulation** in Environment, then **Analyze simulation** in Flight analysis. **Load examples** imports the included recordings. Training runs in Ubuntu-24.04 WSL2 while the GUI stays on Windows. Closing during a job requests a training checkpoint stop, if applicable, and waits for the job to finish before closing.

**Shuffle course** generates and saves a new seed under `runs/worlds/`. **Save world** exports the selected layout, and **Build Blender scene** creates a matching `.blend` and PNG in `runs/world-build/`. The seed makes both the varied hoop layout and vegetation placement repeatable. Trees are kept clear of a corridor around the route. To reopen a specific layout, use `python -m fwrl.app --world path/to/world.json` in the Windows virtual environment.

## Flight input and interpretation

Each CSV is one continuous flight. Required columns:

```csv
t_s,x_m,y_m,z_m,roll_deg
0,0,0,80,0
1,22,0,80,0
2,44,0,80,0
```

`roll_deg` is optional. Time is seconds; positions are local east/north/up in metres. GPS latitude/longitude, NED logs, PX4/ArduPilot logs and ROS bags require conversion before import. No such adapters are implemented yet. Duplicate/decreasing timestamps, NaNs, missing values and fewer than three samples are rejected. Separate flights must not be concatenated.

Classification is **duration weighted**, including across files. Defaults: turn rate at most 3°/s, vertical speed at most 0.5 m/s, and measured absolute roll at most 5° if available. Groundspeed under 5 m/s and gaps above 5 s are unknown, as are intervals whose turn estimate depends on those gaps. Unknown time remains in the denominator. Derivatives are interval-based without smoothing; configure thresholds for sensor noise and sampling rate.

Reports contain straight/left/right × level/climb/descent segments, banked straight-level segments, coverage, and loiter/spiral candidates for sustained turns exceeding 300°. These are geometric candidates, not proven flight intent. Racetrack and figure-eight recognition are not implemented; the included figure-eight example is classified into its primitive segments. Position-only results cannot establish wings-level attitude. Very sparse sampling can alias turns.

```powershell
.\.venv\Scripts\python.exe -m fwrl.cli analyze data/examples --output report.json
.\.venv\Scripts\python.exe -m fwrl.cli demo --output runs/baseline.csv
.\.venv\Scripts\python.exe -m pytest -q
```

## WSL2 and ROS 2

The selected combination is Ubuntu 24.04 / ROS 2 Jazzy / MATLAB R2026a. The local MATLAB executable reports **R2026a Update 4**, with licensed ROS Toolbox, although the supplied release description was “2026.2”.

First-time setup (from this project folder in Ubuntu):

```bash
bash scripts/setup_wsl.sh
bash scripts/build_ros.sh
bash scripts/run_ros.sh
```

ROS build outputs live in `~/.local/share/fwrl/ros2_ws` because colcon's generated setup hooks fail with spaces in the OneDrive path. `scripts/run_ros.sh` sources this prefix and adds the source project to `PYTHONPATH`. Override `FWRL_ROS_BUILD_ROOT` if needed, keeping its path free of spaces. `bash scripts/test_ros.sh` verifies controller messaging and the stale-command pause.

The setup script installs ROS, Blender, and a separate Linux virtual environment with a CUDA 13.0 PyTorch wheel. It does not install a Linux NVIDIA kernel driver; WSL uses the Windows host driver. The Windows `.venv` and Linux `~/.local/share/fwrl/venv` are independent. `scripts/wsl_python.sh` selects the Linux runtime; set `FWRL_VENV` to override its location. Initial provisioning uses the WSL root account; project scripts also support a normal Linux user with sudo. For larger datasets and runs, move a copy into the Linux filesystem to avoid OneDrive sync and `/mnt/c` I/O overhead, then create its virtual environment there.

| Topic | ROS type | Contract |
|---|---|---|
| `/fwrl/odometry` | `nav_msgs/Odometry` | `map` ENU pose, `base_link` FLU; forward airspeed in child-frame twist |
| `/fwrl/waypoint` | `geometry_msgs/PointStamped` | Static route waypoint in `map`, metres |
| `/fwrl/command` | `geometry_msgs/TwistStamped` | `fwrl_setpoint`: angular.x = desired left-bank rad, angular.y = climb angle rad, linear.x = speed m/s |
| `/fwrl/status` | `std_msgs/String` | JSON run state and reached-waypoint count |
| `/fwrl/analysis` | `std_msgs/String` | JSON batch report from analyzer node |

The command message is an explicitly documented simulation setpoint encoding, **not a physical velocity twist**. Do not connect it to an actuator interface. Simulation runs at nominal 20 Hz; physics pauses if commands are absent for 0.5 s. Timesteps are fixed and the ROS headers use wall clock; this is not a `/clock`-synchronized simulator. Domain ID defaults to 42. Run only one controller at a time. Analyzer usage after sourcing ROS and workspace setup:

```bash
ros2 run fwrl_ros analyzer --ros-args -p input_dir:="$PWD/data/examples"
```

## MATLAB controller

Start ROS with the Python controller disabled:

```bash
bash scripts/run_ros.sh python_controller:=false
```

In MATLAB, with this project as current folder:

```matlab
addpath('matlab');
fwrl_check;
fwrl_controller(600, 42);
```

`fwrl_check` verifies local ROS Toolbox node creation. That alone does not prove cross-OS DDS discovery. `fwrl_integration_check` additionally verifies that MATLAB commands move the WSL simulator. Windows/WSL networking and firewall rules must permit discovery on domain 42. If nodes do not discover each other, inspect `ros2 topic list` in Ubuntu with `ROS_DOMAIN_ID=42` and `ros2('topic','list','DomainID',42)` in MATLAB. Explicit Fast DDS peers may be required; do not disable the firewall globally. Integration test status is recorded in `VALIDATION.md`.

The MATLAB and Python controllers use the same bounded heading/altitude waypoint guidance. They command idealized inner-loop setpoints, not elevons or throttle. Controller synthesis, gain identification and aircraft aerodynamic validation remain separate work.

## GPU reinforcement learning

The trainer runs **both vectorized simulation and PPO neural networks on CUDA**. It learns bounded residuals around the reference waypoint guidance. Observations include waypoint displacement, heading, speed, bank/climb angle, wind, and all obstacle boxes; it is not a vision policy. Rewards favor route progress and waypoint completion, penalize collisions and control effort. Episodes end at course completion, collision, bounds violation, or time limit. Truncations bootstrap value estimates from the pre-reset state. The default high-speed course uses the force-based model described below; legacy worlds retain the original lag model.

```bash
bash scripts/wsl_python.sh -m fwrl.training --device cuda --envs 128 --steps 1000000 --output runs/navigation-001
bash scripts/wsl_python.sh -m fwrl.evaluate runs/navigation-001/policy.pt --device cuda --episodes 32
```

Every output folder must be new. Checkpoints include policy, optimizer, configuration and transition count. `metrics.jsonl` records progress. Create a `STOP` file in the run folder for a graceful checkpoint stop. Training rounds up to a whole rollout. CPU mode is available for debugging. Evaluation uses distinct wind seeds and exports a flight CSV for the analysis tab. A short smoke run verifies execution, not learned reliability. Checkpoint resumption and ROS deployment of learned policies are not implemented.

## Previous guided 200 mph course (retained for comparison)

The older `training_airfield.json` world is a roughly 7 km course with nine upright hoops approximately 750 m apart. The aircraft starts at rest on a ground-mounted inclined 22.352 m launch rail. Constant rail acceleration releases it at **50 mph (22.352 m/s) after 2 seconds**. The target airborne speed is **200 mph (89.408 m/s)**. The former meadow is preserved in `worlds/meadow_legacy.json`. Shuffle course preserves the selected simulation mode.

Physics is a force-based 3-DOF model using mass, altitude-dependent air density, lift saturation (CLmax), a quadratic drag polar, gravity, finite thrust/power, load limits, and bank-rate limits. Low-speed lift deficiency causes descent. The bank/lift autopilot remains idealized: this is **not calibrated six-degree-of-freedom aerodynamics** and does not model rotational inertia, sideslip, gust transients, structural failure, or control surfaces. The 5 kg/1.651 m aircraft's wing area and aerodynamic/propulsion coefficients are explicit assumptions in the world JSON; real flight validation requires measured coefficients and propulsion data. Force equations follow [NASA lift](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/lift-equation/) and [NASA drag](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/).

Hoops score only when crossed forward through the clear aperture in order. Swept collision tests include every hoop torus, terrain, trees and world bounds, conservatively accounting for aircraft size and distance between samples. GPU episodes reset to rest on the same launcher after a crash, completion or time limit. PPO learns small residual corrections around waypoint guidance and rewards speed accuracy at hoop crossings. This is guided learning, not learning all flight control from scratch.

To inspect the older guided demonstration, open `worlds/training_airfield.blend` and press **Space** to play the baked evaluated flight from its chase camera. The aircraft and launcher are at frame 1; the final frame resets to the launcher. To use **live physics with automatic crash resets**, open the embedded **Live Simulation.py** in Blender's Text Editor and run it with **Alt-P**, then press Space. This deliberately requires running the script once rather than weakening Blender's script security. Return to frame 1 to restart. The live mode uses the embedded trained policy when available, with zero wind; the evaluated replay includes its held-out wind. Scene custom properties show speed, next hoop and reset count. The reference controller is used when building without exported policy weights. Python source must remain available at the project path (the default `.blend` location also supports Windows path discovery).

```bash
bash scripts/wsl_python.sh -m fwrl.training --device cuda --envs 64 --steps 1048576 --output runs/new-highspeed-run
bash scripts/wsl_python.sh -m fwrl.evaluate runs/new-highspeed-run/policy.pt --device cuda --episodes 32
bash scripts/wsl_python.sh scripts/export_blender_policy.py runs/new-highspeed-run/policy.pt
blender --background --python blender/build_world.py -- --world worlds/training_airfield.json --output worlds/training_airfield.blend --flight runs/new-highspeed-run/blender_flight.json --policy-weights runs/new-highspeed-run/blender_policy.json
```

Blender displays simulations generated by the same numerical model; Blender rigid-body gravity is not applied a second time. Training executes in PyTorch/CUDA outside Blender. The app's Run simulation runs a finite reference-controller episode; training and Blender live mode provide automatic resets. The legacy meadow, ROS and MATLAB reference adapters retain their original idealized model and are not high-speed policy deployments.

## References

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [MathWorks ROS Toolbox supported distributions](https://www.mathworks.com/help/ros/gs/ros-system-requirements.html)
- [MATLAB ROS 2 publisher API](https://www.mathworks.com/help/ros/ref/ros2publisher.html)
- [PyTorch installation selector](https://pytorch.org/get-started/locally/)
- [PyTorch Blackwell support](https://pytorch.org/blog/pytorch-2-7/)


The live viewer includes sun shadows, atmospheric sky/haze and slope-dependent grass, rock and snow shading. The airframe has canted winglets, visible servos, horns and pushrods; elevons and linkages follow actual surface telemetry, with independent left/right angle readouts. Winglets are visual geometry; aerodynamic coefficients remain the existing assumed model.

Latest tuning: first hoop lowered 13 m below the original launcher-axis position; swept leading and trailing wing edges with narrower chord. The provisional 12S (44.4 V nominal) setup uses 45 N static thrust, 3,000 W useful propulsive power and a 0.12 s motor response. These are estimates, not values inferred from cell count or a calibrated motor/propeller map. Battery discharge and voltage sag are not simulated. Compatible prior learning is explicitly transferred when physics changes.

Tail revision: fuselage widened 16% with tapered nose retained. Twin booms clear the rear propeller and support fixed horizontal/vertical stabilizers. The elevons remain the active controls. Estimated pitch/yaw restoring and damping coefficients are increased; pitch/yaw inertia and the conservative collision radius are enlarged. These are reduced-order stability assumptions, not measured tail aerodynamics.

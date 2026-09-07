# Validation record — 2026-09-06

## Meadow and interface update

- Replaced the paved airfield with rolling terrain, **105 procedural trees**, **310 foliage patches**, grass tufts and **9 upright, numbered hoops**. Course and vegetation generation are reproducible by seed.
- The default meadow baseline completes all **9 waypoints in 126.75 simulated seconds**, without collision.
- **17 WSL tests pass**, covering deterministic generation, gate orientation, terrain relief/collisions, tree clearance, and agreement between NumPy/Torch terrain calculations, as well as existing analysis and simulation checks.
- CUDA regression run: **1,024 transitions** saved successfully in `runs/meadow-cuda-smoke-001`. Evaluation completed **4/4 meadow flights**, with no collisions/timeouts. This remains a simple-course smoke test, not a generalization claim.
- Blender `.blend` and PNG regenerated and visually inspected: upright route-aligned hoops, vegetation and a textured heightfield. Terrain is shared with CPU/CUDA collision checks; Blender's terrain mesh samples the analytic surface on a 220×220 grid.
- Light desktop redesign covers all three pages, navigation, rounded controls/cards, metrics, plots, inputs, tables, activity logs and status. Inspected the actual Windows app and exercised all three pages. A compact-window check exposed cramped plots at 780 px tall; minimum height is now 860 px to keep charts legible.
- Corrected Mplot3D draw order so airborne hoop markers remain visible above terrain; the preview intentionally prioritizes course legibility, while Blender handles full depth occlusion.
- Shuffle saves a new JSON under `runs/worlds`; imports, simulation, Blender builds and training share the selected world. A saved layout can be reopened with `--world`.

## Original airfield baseline

Verified on this Windows machine:

- Installed Ubuntu 24.04.4 under WSL2, alongside the existing Docker distribution.
- Windows NVIDIA driver 616.64; RTX 5070 Ti, 16,303 MiB VRAM, visible inside WSL.
- MATLAB R2026a Update 4 (26.1.0.3312084), ROS Toolbox 26.1 installed and license test passed.
- ROS 2 Jazzy package built successfully in the Linux filesystem.
- Windows tests: **10 passed, 2 skipped** (PyTorch-specific tests run separately in WSL).
- Desktop GUI constructed, loaded example flights, plotted results, and exited normally.
- Reference controller completed all 4 waypoints in **118.6 simulated seconds** without collision.
- ROS messaging test: simulated aircraft moved **44.0 m**, then correctly paused after commands became stale.
- Windows MATLAB received WSL ROS odometry and commanded the aircraft through DDS: **68.2 m** movement in the bounded integration test, with the Python controller disabled. No network configuration changes were needed.
- Blender 4.0.2 generated and saved `worlds/training_airfield.blend`; overview PNG rendered and visually inspected. Denoising disabled because this Ubuntu Blender package lacks OpenImageDenoiser.
- WSL tests: **12 passed**, including consistency between NumPy and Torch integration and distinct termination/time-limit handling.
- PyTorch **2.14.0+cu130** ran the CUDA simulator and PPO optimizer on the **RTX 5070 Ti**. The Linux runtime is `/root/.local/share/fwrl/venv`.
- GPU smoke training: **32,768 transitions**, finite losses, checkpoint saved in `runs/cuda-smoke-001/policy.pt`; about **22,112 transitions/s** at completion. This is an observed short-run measurement, not a sustained benchmark.
- Deterministic checkpoint evaluation with wind seed 1001: **16/16 course completions, 0 collisions, 0 timeouts**. This is the same simple world with different wind seeds and a policy built around baseline guidance; it does not demonstrate generalization to unseen obstacles or improvement over the baseline.
- `runs/cuda-smoke-001/evaluation_flight.csv` can be loaded into the app; evaluation metrics and training configuration are stored beside it. Dependency snapshots are in `configs/requirements-windows-lock.txt` and `configs/requirements-wsl-lock.txt`.

An earlier CUDA installation into `.venv-wsl` was cancelled because extraction through OneDrive was slow. Automatic approval review blocked cleanup of that abandoned folder. It remains unused and is ignored by Git; scripts use the native Linux runtime above.

This establishes basic functionality, not real-aircraft fidelity, controller flightworthiness, or navigation reliability on unseen worlds. The camera is a Blender asset; live image transport, vision policy training, aerodynamic control-surface dynamics, and learned-policy deployment over ROS are not implemented.


## High-speed launch/hoop update — 2026-09-06

- Linux suite: 21 passed; Windows suite: 15 passed, 6 PyTorch-dependent tests skipped.
- Tested rail start at rest, 22.352 m/s release after 2 s, force-limited low-speed descent, NumPy/PyTorch airborne parity, forward/backward gate crossing, fast torus strikes, ground collision, and launcher reset.
- CUDA PPO: 1,048,576 transitions, 64 environments; checkpoint `runs/highspeed-200mph-002/policy.pt`.
- Held-out evaluation seed 1001: 32/32 complete nine hoops, 0 crashes/timeouts. Mean gate speed 199.93 mph; minimum 197.27 mph. This is a single course and wind distribution, not evidence of real-aircraft reliability or learning from scratch.
- Blender 4.0.2 generated `worlds/training_airfield.blend` with the evaluated trajectory, embedded inference weights, chase camera, launch rails, and live simulation text.
- Headless Blender integration test passed: embedded script executes, aircraft releases at 50 mph, forced ground crash resets position/speed/next hoop to the launcher.
- The aerodynamic model is force-based 3DOF with assumed coefficients and an abstract bank/lift autopilot. It is not validated six-DOF flight dynamics. Legacy ROS/MATLAB adapters are unchanged.


## Live Mountain Gauntlet from zero — 2026-09-06

- Replaced the default app simulation workflow with live CUDA training streamed to Blender at 20 Hz. No animation or policy checkpoint is loaded by the new session launcher.
- Mountain Gauntlet: 20 gates, five rugged mountain groups, reversing turns, climbs and descents. Terrain and collisions share the analytic mountain function.
- Randomly initialized PPO directly controls roll rate, lift coefficient and throttle. No waypoint, altitude, speed or lift-trim controller is invoked in direct-control mode. Tests explicitly forbid the guidance call and verify zero lift/no throttle do not hold flight.
- Linux: 26 tests passed. Windows: 18 passed, 8 PyTorch tests skipped.
- Headless Blender/CUDA integration passed: live telemetry changes aircraft position, pause holds the trainer tick, resume advances it, stop saves a checkpoint. The integration-test policy is not loaded into the user session.
- Earlier 32/32 completion results apply only to the older guided course. No successful completion is claimed for this harder, unguided task. Early crashes are expected.
- Physics remains an abstract 3DOF aerodynamic model, not validated surface-control/6DOF dynamics.


## Elevon/stall/deadline correction — 2026-09-06

- Linux suite: 31 passed. Includes 50 mph reference stall relationship, sub-stall lift deficiency, post-stall lift loss and drag increase, full-throttle vertical deceleration/descent, elevon mixing and servo rate bounds, NumPy/Torch equivalence, first gate positive reward, deadline negative terminal/reset and ground-crash negative reward/reset.
- Neutral-control launch passes first gate at about 2.5 seconds after starting at rest; this is geometric launcher assistance, not a policy demonstration or loaded controller.
- CUDA smoke training of the new random policy ran 8,192 transitions with finite losses and observed gate passages/crashes. This does not establish course mastery.
- Old guided/direct-lift results and checkpoints do not validate the new six-DOF elevon model. All coefficients except the user-specified stall-speed target remain assumptions.


## Longer gate budgets and reference airframe — 2026-09-06

- Linux suite: 32 passed after deadline and checkpoint-resume changes.
- Blender world rebuilt successfully with the shared flying-wing mesh; the live Windows viewer was visually inspected in Aircraft view.
- Retained 49,152 transitions from the compatible elevon checkpoint and preserved the paused state. The first-gate telemetry shows the new 20-second budget.

## Conventional reference airframe and 45 mph stall

- 33 tests pass with the updated 45 mph reference stall relationship and larger wing.
- Existing coverage verifies below-stall lift deficiency, post-stall lift loss/drag rise, inability to sustain vertical powered climb, servo response, gate/crash rewards and resets.
- Long fuselage, directly attached tail, longer swept wings and aft propeller share one mesh generator between Blender and the browser.

## Native Linux compatibility

- Ubuntu Linux under WSL: 33 tests passed. Native shell wrapper ran CPU PPO for 16 transitions and saved telemetry plus its companion world file.
- With CUDA hidden, `--device auto` completed a second 16-transition CPU run.
- Native Linux viewer returned HTTP 200 for page, scene and state on an isolated port.
- Windows: 23 tests passed, 10 skipped because that environment has no PyTorch.
- Physical Linux desktop and other distributions were not exercised locally; Ubuntu CPU CI is included.

## Curriculum restart

- 38 tests passed, including fixed observation dimensions across stages, no guidance calls, rail-action masking, normalized checkpoint round trips, flight reward ordering and evaluation isolation.
- CPU and CUDA curriculum smoke runs completed with finite losses, checkpoint saves and independent fixed-seed evaluations. Smoke policies are not used by the fresh run.
- The new user run started at zero transitions with seed 73, 64 CUDA environments and 512-step rollouts. The live display is explicitly labeled policy evaluation; it never supplies training data or demonstrations.
- Initial updates had finite losses. These checks establish operation, not flight mastery.

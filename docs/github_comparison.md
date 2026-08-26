# GitHub Main Comparison

Checked against `https://github.com/Zikangz/SO101_control` branch `main` at commit `c75ac4f88752e4f6336caf39bce06c15793ac028` on 2026-08-26.

## Summary

- Local code is a ROS Noetic hardware/control branch centered on SO101 desk and hardware validation.
- Remote `main` is broader: it includes MuJoCo/RL training utilities, `so101_tracking`, additional notes, and vendored third-party trees.
- This branch keeps the ROS1 description package, Noetic bridge, hardware launch/configs, calibration helpers, safety poses, IK/FK, trajectory tools, aerial-state/gating nodes, compensation observers, and repeatability diagnostics.
- Local logs, local virtual/conda environments, ROS build outputs, archives, and third-party working copies are intentionally excluded from Git.

## Local-Only Control/Deployment Additions

- `requirements-noetic.txt` for Python 3.8 / ROS Noetic dependency pinning.
- `scripts/bootstrap_noetic.sh`, `scripts/so101_common.sh`, and portable `scripts/so101_env.sh` for quick setup on another machine.
- Hardware run wrappers: `run_ros_hardware_bridge.sh`, `run_ros_hardware_bridge_smooth.sh`.
- Calibration utilities: `run_seeed_*`, `check_so101_calibration.py`, `export_lerobot_calibration_to_ros.py`, `apply_so101_calibration_to_ros.sh`.
- Aerial integration helpers: `aerial_state_bridge.launch`, `aerial_velocity_gate.launch`, `so101_aerial_velocity_gate.py`, `check_aerial_mavros_so101.sh`.
- Compensation/diagnostic tools: backlash fitting, phase-z fitting, low-frequency bias observer, load compensation observer, precision suite, smoothness repeatability suite, and MuJoCo contact scan wrapper.
- Project docs under `docs/` covering Noetic setup, precision/aerial validation, architecture, troubleshooting, and current next steps.

## Remote-Only Areas Not Carried Into This Branch

- RL/MuJoCo simulation scripts such as `train_sac.py`, `train_pick_lift_sac.py`, `mujoco_planar_control_sim.py`, and evaluation scripts.
- `so101_tracking/` Gym-style environments.
- `tests/test_servo.py` from remote main.
- `mujoco_bridge.launch`.
- Remote notes: `NOTES_AND_USAGE.md`, `PICK_LIFT_AND_ACT_NOTES.md`, `ROS1_TRAJECTORY_DEBUG.md`.
- Remote vendored third-party files remain external/local in this branch rather than tracked directly.

## Changed Shared Files

- `README.md`: expanded from simulation/control notes into Noetic hardware quick-start and validation guide.
- `.gitignore`: expanded to exclude local environments, logs, archives, generated catkin outputs, and third-party checkouts.
- `ros1_ws/src/so101_ros1_bridge/CMakeLists.txt`: installs additional hardware, aerial, observer, trajectory, plotting, and compensation scripts.
- `hardware_bridge.launch`: adds real-hardware timing, PID, feedback-assist, backlash-compensation, aerial-state/gating, servo, and trajectory-action parameters.
- `mock_bridge.launch`: keeps mock control with optional kinematics, servo, and trajectory-action nodes.
- `so101_driver_node.py`: adds higher-rate command handling, telemetry, feedback-assist hooks, backlash compensation, timed trajectories, and richer status output.
- `so101_ee_sine_test.py`, `so101_sine_test.py`, `so101_analyze_csv.py`: expanded for validation, safer execution modes, richer CSV analysis, and repeatability workflows.
- `hardware.py`: keeps Feetech STS3215 backend, motor configuration, diagnostics, sync/individual writes, and portable third-party SDK discovery.
- Config YAMLs: updated safe joint limits, hardware calibration block, trajectory/servo parameters, and active/locked joint defaults.

## Readiness Judgment

- OK as the current ROS1/Noetic ground-control baseline for bench testing and conservative SO101 hardware operation.
- OK to push to GitHub after excluding local logs, `.env`, catkin build outputs, conda/venv folders, archives, and third-party working copies.
- Not final as an aerial flight controller. Aerial use still requires staged validation: mock, desk hardware, no-prop UAV, dummy-load power testing, emergency stop, MAVROS failsafe checks, and flight-envelope limits.

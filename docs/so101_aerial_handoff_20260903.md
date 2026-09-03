# SO101 Aerial Handoff - 2026-09-03

## Current conclusion

The SO101 follower now has a usable high-speed desktop baseline for future
aerial-manipulation work:

- Calibration has been regenerated after factory-middle setup and synced into
  `ros1_ws/src/so101_ros1_bridge/config/so101_simplified_4dof.yaml`.
- The current stable speed target is at least `f=0.1 Hz` for the reduced
  high-speed diagnostic path.
- The best high-speed trajectory shape is currently `xz_vertex_diamond`, not
  the original `xz_sine` at full `120 x 60 mm` size and not the edge-heavy
  `xz_edge_vertex8` path.
- Keep `SO101_LPF_ALPHA=1.0` for the current speed baseline unless visual
  jitter returns. The latest runs do not justify adding low-pass filtering.
- Conservative two-axis phase profiles have been generated and hardware tested.
  The latest diamond X/Z profile did not beat the constant Z-bias diamond
  baseline, so it is kept as an experiment rather than the recommended run.
- If validation reports `NOT SAFE TO RUN`, first command the arm to `ready` and
  validate with `--prep-pose ready`; the failing check can otherwise select an
  IK branch close to the wrist limit even when the real run would start from a
  safer branch.
- A same-size shape alternative is available as `xz_vertex_diamond`. It keeps
  `100 x 40 mm` at `f=0.1` but routes through top/right/bottom/left midpoints
  instead of combining X-edge and Z-extreme poses.
- `xz_vertex_diamond + z_bias=0.0083` is currently the best measured desktop
  result: mean `4.70 mm`, RMSE `5.04 mm`, max `8.38 mm`, with the same
  `100 x 40 mm` size at `f=0.1`.
- The remaining problem is mainly repeatable geometric error from gravity sag,
  structural compliance, backlash, and command/plant phase lag. It is not a
  random high-frequency vibration problem.

## Latest desktop metrics

All values below use `--ignore-start 10`.

| Run | Path | Mean EE error | RMSE | Max | Z high-pass RMS | Status |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083.csv` | edge-vertex 8, `100 x 40 mm`, `f=0.1`, constant Z bias | `6.49 mm` | `6.92 mm` | `13.05 mm` | `0.519 mm` | Original constant-bias baseline |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v1.csv` | same path, phase Z profile v1 | `6.17 mm` | `6.45 mm` | `10.62 mm` | `0.633 mm` | Slightly better geometry, slightly higher high-pass |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2.csv` | same path, phase Z profile v2 | `5.76 mm` | `6.04 mm` | `9.16 mm` | `0.572 mm` | Best original learned run |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v3.csv` | same path, phase Z profile v3 | `6.43 mm` | `6.81 mm` | `10.52 mm` | `0.588 mm` | Regressed; do not use as baseline |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat_20260903_203926.csv` | same path, repeated phase Z profile v2 | `5.69 mm` | `5.97 mm` | `9.51 mm` | `0.599 mm` | Current recommended desktop baseline |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_final_20260903_205926.csv` | same path, repeated phase Z profile v2 | `5.74 mm` | `6.03 mm` | `10.31 mm` | `0.606 mm` | Stable enough for next compensation step |
| `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_repeat_20260903_204105.csv` | same path, repeated constant Z bias | `6.27 mm` | `6.66 mm` | `11.60 mm` | `0.534 mm` | Slightly smoother, less accurate geometry |
| `/tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808.csv` | vertex diamond, `100 x 40 mm`, `f=0.1`, constant Z bias | `4.70 mm` | `5.04 mm` | `8.38 mm` | `0.573 mm` | Current best measured geometry |
| `/tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747.csv` | vertex diamond, `100 x 40 mm`, `f=0.1`, X/Z phase profile v1 safe | `5.67 mm` | `5.97 mm` | `9.85 mm` | `0.577 mm` | Regressed vs constant Z bias; do not use as baseline |
| `/tmp/so101_ee_xz_zigzag_f100_x040_z018_zbias0083.csv` | zigzag, `80 x 36 mm`, `f=0.1`, constant Z bias | `7.52 mm` | `8.35 mm` | `15.83 mm` | `0.612 mm` | Usable, less accurate |
| `/tmp/so101_ee_xz_sine_f100_x060_z030_zbias0083.csv` | original XZ sine, `120 x 60 mm`, `f=0.1`, constant Z bias | `8.87 mm` | `9.33 mm` | `15.76 mm` | `0.902 mm` | More Z excitation |
| `/tmp/so101_ee_sine.csv` | latest default-name edge run, no Z bias/profile | `10.71 mm` | `11.47 mm` | `19.68 mm` | `0.572 mm` | Do not use as baseline |

Interpretation: phase-Z compensation improved the original edge path through
v2, v2 repeated cleanly, and v3 regressed. The same-size diamond path with a
constant Z bias then beat the edge-path family. The diamond X/Z profile v1 safe
run regressed, so keep `xz_vertex_diamond + z_bias=0.0083` as the current
desktop baseline. Do not continue blind profile fitting without a new residual
diagnosis. The default-name
`/tmp/so101_ee_sine.csv` appears to be an uncompensated run and should not
replace the baseline.

## Saved artifacts on this machine

Trajectory CSVs:

Plot images are copied to the easy-to-find project directory `so101_plots/`.
The two key diamond CSVs are also preserved in
`calibration/phase_compensation/` for Git-based handoff.

- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083.csv`
- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v1.csv`
- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2.csv`
- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat_20260903_203926.csv`
- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_repeat_20260903_204105.csv`
- `/tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v3.csv`
- `/tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808.csv`
- `/tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747.csv`
- `calibration/phase_compensation/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808.csv`
- `calibration/phase_compensation/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747.csv`
- `/tmp/so101_ee_xz_zigzag_f100_x040_z018_zbias0083.csv`
- `/tmp/so101_ee_xz_sine_f100_x060_z030_zbias0083.csv`
- `/tmp/so101_ee_xz_8_fast_f040_zbias0083_current_calib.csv`
- `/tmp/so101_ee_xz_8_fast_f050_zbias0083_current_calib.csv`
- `/tmp/so101_ee_xz_8_fast_f055_zbias0083_current_calib.csv`
- `/tmp/so101_ee_xz_8_fast_f060_zbias0083_current_calib.csv`

Compensation profiles:

- `/tmp/so101_phase_z_edge_vertex8_f100_x050_z020.json`
- `/tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v2.json`
- `/tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v3.json`
- `/tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1.json`
- `/tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_safe.json`
- `/tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_ultrasafe.json`
- `/tmp/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_micro.json`
- `/tmp/so101_phase_xz_vertex_diamond_f100_x050_z020_v1.json`
- `/tmp/so101_phase_xz_vertex_diamond_f100_x050_z020_v1_safe.json`
- `calibration/phase_compensation/so101_phase_xz_edge_vertex8_f100_x050_z020_v1.json`
- `calibration/phase_compensation/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_safe.json`
- `calibration/phase_compensation/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_ultrasafe.json`
- `calibration/phase_compensation/so101_phase_xz_edge_vertex8_f100_x050_z020_v1_micro.json`
- `calibration/phase_compensation/so101_phase_xz_vertex_diamond_f100_x050_z020_v1.json`
- `calibration/phase_compensation/so101_phase_xz_vertex_diamond_f100_x050_z020_v1_safe.json`
- `/tmp/so101_phase_z_zigzag_f100_x040_z018.json`
- `/tmp/so101_phase_z_sine_f100_x060_z030.json`

Trajectory images:

- `so101_plots/so101_live_trajectory.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat_20260903_203926_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat_20260903_203926_joint_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_repeat_20260903_204105_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_repeat_20260903_204105_joint_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_zbias0083_joint_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v1_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v1_joint_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_joint_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v3_ee_tracking.png`
- `so101_plots/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v3_joint_tracking.png`
- `so101_plots/so101_ee_xz_zigzag_f100_x040_z018_zbias0083_ee_tracking.png`
- `so101_plots/so101_ee_xz_zigzag_f100_x040_z018_zbias0083_joint_tracking.png`
- `so101_plots/so101_ee_xz_sine_f100_x060_z030_zbias0083_ee_tracking.png`
- `so101_plots/so101_ee_xz_sine_f100_x060_z030_zbias0083_joint_tracking.png`
- `so101_plots/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808_ee_tracking.png`
- `so101_plots/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_20260903_220808_joint_tracking.png`
- `so101_plots/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747_ee_tracking.png`
- `so101_plots/so101_ee_xz_vertex_diamond_f100_x050_z020_phasexz_v1_safe_20260903_222747_joint_tracking.png`

## Current high-speed commands

Start the no-PID-overwrite bridge:

```bash
cd /home/zzk/ZZK/SO101
export SO101_ROOT="$PWD"
export PORT=/dev/ttyACM0
export ROS_HOME=/tmp/so101_ros_home
export ROS_LOG_DIR=/tmp/so101_ros_logs
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"

SO101_COMMAND_RATE_HZ=180 \
SO101_FEEDBACK_READ_RATE_HZ=25 \
SO101_LPF_ALPHA=1.0 \
SO101_SERVO_SPEED=500 \
SO101_SERVO_ACCELERATION=25 \
/home/zzk/ZZK/SO101/scripts/run_ros_hardware_bridge_no_pid.sh "$PORT"
```

Run the current desktop baseline:

```bash
cd /home/zzk/ZZK/SO101
export SO101_ROOT="$PWD"
export ROS_HOME=/tmp/so101_ros_home
export ROS_LOG_DIR=/tmp/so101_ros_logs
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source /home/zzk/ZZK/SO101/ros1_ws/devel/setup.bash

rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --execution-mode joint_trajectory \
  --pattern xz_vertex_diamond \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.050 \
  --z-amplitude 0.020 \
  --frequency 0.100 \
  --duration 30 \
  --rate 180 \
  --ramp-duration 8 \
  --move-to-center-duration 10 \
  --center-joint-tolerance 0.03 \
  --center-max-start-error 0.04 \
  --joint-limit-margin 0.080 \
  --z-feedforward-bias 0.0083 \
  --csv /tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_repeat.csv

rosrun so101_ros1_bridge so101_analyze_csv.py \
  --ignore-start 10 \
  /tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_repeat.csv
```

Run the old edge-path learned phase-Z version for comparison:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --execution-mode joint_trajectory \
  --pattern xz_edge_vertex8 \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.050 \
  --z-amplitude 0.020 \
  --frequency 0.100 \
  --duration 30 \
  --rate 180 \
  --ramp-duration 8 \
  --move-to-center-duration 10 \
  --center-joint-tolerance 0.03 \
  --center-max-start-error 0.04 \
  --joint-limit-margin 0.080 \
  --z-feedforward-bias 0 \
  --phase-z-compensation-profile /tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v2.json \
  --csv /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2.csv

rosrun so101_ros1_bridge so101_analyze_csv.py \
  --ignore-start 10 \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2.csv
```

Repeat the old edge-path learned phase-Z version, v2:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --execution-mode joint_trajectory \
  --pattern xz_edge_vertex8 \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.050 \
  --z-amplitude 0.020 \
  --frequency 0.100 \
  --duration 30 \
  --rate 180 \
  --ramp-duration 8 \
  --move-to-center-duration 10 \
  --center-joint-tolerance 0.03 \
  --center-max-start-error 0.04 \
  --joint-limit-margin 0.080 \
  --z-feedforward-bias 0 \
  --phase-z-compensation-profile /tmp/so101_phase_z_edge_vertex8_f100_x050_z020_v2.json \
  --csv /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat.csv

rosrun so101_ros1_bridge so101_analyze_csv.py \
  --ignore-start 10 \
  /tmp/so101_ee_xz_edge_vertex8_f100_x050_z020_phasez_v2_repeat.csv
```

## How to reduce geometry error without adding shake

Use this order. Do not jump straight to high-gain feedback.

1. Keep the smoother path geometry: use `xz_vertex_diamond`, not full-amplitude
   `xz_sine`, for the high-speed desktop baseline.
2. Use bounded periodic feedforward/iterative learning compensation only after
   checking that it improves both RMSE and max error. The latest diamond X/Z
   profile regressed, so the constant Z-bias diamond run remains the baseline.
3. Repeat the same path 2-3 times and compare repeatability. Only learn from
   repeatable error; do not learn from random vibration.
4. If residual error is still phase-like, add X/Z phase-lead compensation next.
   This should be small and bounded. It compensates command/plant lag without
   injecting high-frequency feedback.
5. If residual error changes with arm pose, payload, or UAV tilt, replace the
   fixed phase profile with a state-dependent model: `delta_ee = f(q, qdot,
   target_phase, payload, base_attitude)`.
6. Tune servo PID only after the feedforward path is understood. Keep `I=0`
   initially. Increase P/D cautiously on shoulder and elbow while watching
   current, temperature, and high-pass error.

Candidate algorithms, in practical order:

- Iterative Learning Control / periodic phase feedforward for repeated paths.
- Bounded inverse-error feedforward in task space, starting with Z only.
- Phase-lead compensation for smooth periodic tracking lag.
- Gravity/load compensation as a function of joint angle and UAV attitude.
- Low-gain outer-loop correction only after latency is characterized.
- Model predictive control or whole-body MPC in simulation, with arm setpoints
  still bounded before hardware deployment.

## Whole-body control migration plan

Target architecture:

```text
Simulation policy / planner
  -> whole-body safety supervisor
  -> UAV high-level setpoints through PX4 Offboard / ROS 2 DDS
  -> SO101 bounded joint or Cartesian setpoints
  -> hardware-specific drivers
```

Keep the arm driver interface narrow:

```text
Observation:
  arm joint position/velocity, commanded joint target, EE pose, servo current,
  voltage, temperature, fault flags

Action:
  active-joint target or delta, gripper command, optional Cartesian EE target

Safety envelope:
  joint limits, workspace limits, speed limits, command timeout, freeze, relax,
  estop, UAV-state gating
```

Recommended platform split:

- Windows laptop: documentation, plotting, log review, Isaac Sim/Lab only if it
  has a suitable NVIDIA GPU and RAM.
- Server: Isaac Lab training, large parallel RL, domain randomization, policy
  evaluation.
- Jetson Xavier NX: deployment, logging, ROS 2/PX4 bridge, low-rate inference,
  not large-scale Isaac Lab training.
- Current Ubuntu/ROS1 machine: capture final hardware evidence and export
  calibration/logs before it becomes unavailable.

Simulation order:

1. Export/verify SO101 URDF and meshes.
2. Build MuJoCo arm-only model first; match joint limits, locked joints,
   approximate servo response, latency, and calibration offsets.
3. Reproduce the desktop `xz_edge_vertex8 f=0.1` result in MuJoCo using the same
   action interface and logs.
4. Add UAV base as a moving base with measured mount transform.
5. Add PX4/Gazebo or Isaac/Pegasus-style vehicle dynamics separately.
6. Combine into whole-body simulation only after the arm-only sim reproduces the
   real tracking error trends.
7. Train policy to output high-level setpoints, not raw motor commands.

## What to do today

Before losing access to this machine, finish these items:

1. Save the hardware baseline package:

   ```bash
   cd /home/zzk/ZZK/SO101
   scripts/package_so101_handoff.sh
   ```

2. Run one repeat of the current best baseline and one repeat of phase-Z v2.
   Keep both CSVs and the live PNG.
3. Record a short phone video of the physical arm for each run, named with the
   CSV basename.
4. Record static dimensions: arm base pose relative to planned UAV body frame,
   gripper/payload mass, cable routing, power supply voltage, and mounting
   orientation.
5. Export this repo, the handoff tarball, and `so101_plots/` to external
   storage or the server.
6. On the next machine, do not start from ROS1-specific launch files. Start by
   porting the source model, calibration, and narrow arm command/state API.

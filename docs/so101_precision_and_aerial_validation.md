# SO101 Precision And Aerial Validation

This file records the desk-test workflow before the SO101 arm is allowed into an aerial manipulation loop.

## What The Current Controller Does

- ROS commands are position targets in radians, plus a normalized gripper command.
- The ROS node applies joint limits, velocity limits, command stale handling, and safe named poses.
- The STS3215 servos still run their internal position control. The ROS node writes Feetech `P_Coefficient`, `I_Coefficient`, and `D_Coefficient` registers on startup from `config/so101_simplified_4dof.yaml`.
- `/so101/servo_status` publishes raw encoder position, velocity, load, voltage, temperature, current, and moving status when the Feetech backend is running.
- Timed joint trajectories can now be sampled with either `linear` or velocity-continuous `cubic` interpolation in the driver. The smoother hardware launch uses `cubic` plus command LPF for visible slow Cartesian paths.

## Smooth Trajectory Profile

For the current shaking investigation, start the bridge with the smooth profile:

```bash
cd $SO101_ROOT
scripts/run_ros_hardware_bridge_smooth.sh /dev/ttyACM0
```

Default smooth parameters:

```text
command_rate_hz:          120
trajectory_interpolation: cubic
command_lpf_alpha:        0.55
servo_speed:              500
servo_acceleration:       25
servo_maximum_accel:      254
follow_joint_trajectory:  true
action_resample_hz:       80
```

Parameter meaning:

- `command_rate_hz`: how often the ROS driver samples and writes joint position commands.
- `trajectory_interpolation`: `linear` is piecewise linear; `cubic` is velocity-continuous Hermite interpolation across dense waypoints.
- `command_lpf_alpha`: first-order command low-pass filter. `1.0` disables filtering; lower values are smoother but add lag.
- `servo_speed`: STS3215 maximum speed sent with `WritePosEx`/`SyncWritePosEx`.
- `servo_acceleration`: STS3215 per-command acceleration sent with each position target.
- `servo_maximum_acceleration`: STS3215 maximum acceleration register, matching the LeRobot SO follower setup when set to `254`.
- `action_resample_hz`: optional resampling rate inside the `FollowJointTrajectory` action wrapper. It is only used by action clients, not by direct `/so101/command_joint_positions` tests.

Override any smooth parameter with environment variables:

```bash
SO101_LPF_ALPHA=0.45 SO101_COMMAND_RATE_HZ=150 scripts/run_ros_hardware_bridge_smooth.sh /dev/ttyACM0
```

The current analysis of `/tmp/so101_ee_xz_sine_planned_120w_60h_retry.csv` shows IK is not the limiting factor. The mean Z error is about `-0.014 m`, so use a first-pass static compensation of:

```bash
--z-feedforward-bias 0.014
```

This value must be confirmed by repeatability tests before it is treated as calibration.

Run the repeatability test in a second terminal:

```bash
cd $SO101_ROOT
scripts/run_so101_ee_smoothness_repeatability.sh
```

Outputs are saved under:

```text
$SO101_ROOT/logs/smoothness/<YYYYmmdd_HHMMSS>/
```

The analyzer prints:

- Cartesian tracking error and suggested `--z-feedforward-bias`.
- Joint tracking error in radians.
- A 0.5 s high-pass residual proxy for visible shaking.
- Planned, commanded, and measured joint high-pass residuals, so you can distinguish IK/path noise from driver/output or hardware tracking noise.

Re-analyze any previous CSV while ignoring the first 3 s startup transient:

```bash
rosrun so101_ros1_bridge so101_analyze_csv.py --ignore-start 3.0 /tmp/so101_ee_xz_sine_planned_120w_60h_retry.csv
```

## FollowJointTrajectory Status

`so101_follow_joint_trajectory_server.py` is now available and can be started by the smooth launch. It exposes:

```text
/so101/follow_joint_trajectory
```

It is a compatibility wrapper for MoveIt/ros_control-style clients. It validates joint names and timing, optionally resamples sparse action trajectories, forwards accepted trajectories to `/so101/command_joint_positions`, and reports action feedback from `/so101/joint_states`.

MoveIt1 is not enabled yet. The next MoveIt step is to generate a Noetic MoveIt config from the URDF, point its controller YAML at `/so101/follow_joint_trajectory`, and keep the current driver as the hardware executor.

## Live Servo Watch

Run this in a separate terminal while another terminal is testing one joint:

```bash
source /opt/ros/noetic/setup.bash
source $SO101_ROOT/ros1_ws/devel/setup.bash
rosrun so101_ros1_bridge so101_servo_watch.py shoulder_lift --rate 2
```

Use `all` instead of a joint name to show all servos.


## Coordinate Frame Notes

`/so101/end_effector_pose` is published in `base_link`. In this URDF, `base_link` is fixed to the SO101 base assembly. ROS uses metres and a right-handed frame:

```text
+X: roughly forward from the base through the arm working direction
+Y: lateral/side direction
+Z: upward from the base
origin: base_link origin in the base assembly, before the shoulder joints
```

With the default aerial-safe setup, `shoulder_pan` and `wrist_roll` are locked at `0.0`. The arm therefore behaves mostly like a planar serial chain, so the meaningful Cartesian trajectory test is in a fixed-Y XZ plane. Do not request large Y motion unless `shoulder_pan` is deliberately unlocked and revalidated.

## Cartesian Trajectory Preflight

Always preflight large Cartesian trajectories before commanding hardware:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --validate-only \
  --pattern figure8 \
  --x-amplitude 0.030 \
  --z-amplitude 0.024 \
  --frequency 0.04 \
  --duration 90 \
  --rate 10 \
  --csv $SO101_ROOT/logs/precision/validate_ee_figure8_30x24mm.csv
```

Current mock/IK preflight results using the conservative joint box:

```text
20mm / 16mm: accepted 180/180
30mm / 24mm: accepted 120/120
40mm / 32mm: failed, accepted 37/120
120mm / 100mm: failed, accepted 11/180
```

So 120 mm should not be run on hardware with the current locked-joint, conservative-limit setup. Start real hardware at 12/10 mm, then 20/16 mm, then 30/24 mm only if current, temperature, and tracking remain acceptable.

## Batch Precision Test

Start the bridge first:

```bash
cd $SO101_ROOT
scripts/run_ros_hardware_bridge.sh /dev/ttyACM0
```

Then run the full desk precision suite in a new terminal:

```bash
cd $SO101_ROOT
scripts/run_so101_precision_suite.sh
```

Outputs are saved under:

```text
$SO101_ROOT/logs/precision/<YYYYmmdd_HHMMSS>/
```

Each action gets a separate CSV:

- `01_hold_ready_30s.csv`
- `02_repeatability_safe_poses.csv`
- `3_sine_shoulder_lift.csv`
- `4_sine_elbow_flex.csv`
- `5_sine_wrist_flex.csv`
- `6_sine_gripper.csv`
- `7_ee_figure8_xz.csv`

For a more visible Cartesian loop, run:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --pattern figure8 \
  --x-amplitude 0.020 \
  --z-amplitude 0.016 \
  --frequency 0.04 \
  --duration 90 \
  --rate 10 \
  --csv $SO101_ROOT/logs/precision/ee_figure8_xz.csv
```

The analysis report is:

```text
$SO101_ROOT/logs/precision/<YYYYmmdd_HHMMSS>/analysis.txt
```

Re-run analysis on the latest result:

```bash
scripts/analyze_so101_precision_suite.sh
```


## Conservative Flight-Safe Joint Box

The default 4-DOF control config now uses the first MuJoCo-screened conservative joint box:

```text
shoulder_lift: [-0.6, 0.6]
elbow_flex:    [-0.8, 0.9]
wrist_flex:    [-0.6, 0.6]
shoulder_pan:  locked at 0.0
wrist_roll:    locked at 0.0
gripper:       task command / normally 0.5 during collision scan
```

This is not a complete proof of safety; it is the current desk-test box with zero random MuJoCo contacts in a 3000-sample screen. Wider ranges need path-level collision checks before use on a UAV.

## Rosbag Recording

Arm-only bag:

```bash
scripts/record_so101_arm_bag.sh
```

Aerial integration bag template:

```bash
scripts/record_aerial_manipulation_bag.sh
```

Start the combined state bridge when MAVROS and SO101 are both running:

```bash
roslaunch so101_ros1_bridge aerial_state_bridge.launch
rostopic echo -n 1 /aerial_manipulation/state
scripts/check_aerial_mavros_so101.sh
```

## MuJoCo Self-Contact Scan

Install MuJoCo into the Python environment you want to use for simulation, then run:

```bash
python3 -m pip install mujoco
scripts/run_so101_mujoco_contact_scan.sh
```

Outputs are saved under:

```text
$SO101_ROOT/logs/mujoco_contact/<YYYYmmdd_HHMMSS>/
```

This scan does not prove the arm is safe in all cases. It is a fast screen for common self-contact under the downloaded SO101 MuJoCo model.

## Suggested Acceptance Gates

- No serial/backend errors in `/so101/status`.
- Servo voltage stays near the expected 12 V follower supply during loaded moves.
- Servo temperature remains stable during repeated tests.
- Hold drift at a fixed pose should be small and monotonic drift should not grow over the 30 s window.
- Sine tracking should show no sustained oscillation and no large phase lag at the selected slow test frequency.
- MuJoCo scan should not report repeated contacts between moving arm links inside the planned workspace.

# SO101 ROS1 Trajectory Debug Notes

This note records the current diagnosis for the ROS1/Noetic SO101 bridge.

## Main Diagnosis

The most likely cause of visible Z offset and end-effector jitter is the online
Cartesian streaming path:

1. `so101_ee_sine_test.py --execution-mode cartesian_stream` publishes many
   `/so101/cartesian_target` poses.
2. `so101_kinematics_node.py` solves IK for each pose independently.
3. Each solved pose is sent as a single-point `JointTrajectory`.
4. `so101_driver_node.py` treats every single-point message as a new target and
   restarts its minimum-jerk interpolation.

This creates phase lag. On XZ trajectories the lag appears as a persistent Z
offset, and high-rate target resets can make the endpoint jitter.

## Preferred First Test

Use the preplanned joint trajectory path first. It solves IK for the whole path,
keeps one continuous IK branch, and sends one multi-point trajectory to the
driver:

```bash
cd /home/bot/research/so101_mujoco_tracking/ros1_ws
source devel/setup.bash

roslaunch so101_ros1_bridge hardware_bridge.launch \
  port:=/dev/ttyACM0 \
  trajectory_interpolation:=cubic
```

In another terminal:

```bash
cd /home/bot/research/so101_mujoco_tracking/ros1_ws
source devel/setup.bash

rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --execution-mode joint_trajectory \
  --pattern xz_sine \
  --x-amplitude 0.01 \
  --z-amplitude 0.01 \
  --frequency 0.02 \
  --duration 30 \
  --rate 20 \
  --csv /tmp/so101_joint_traj.csv
```

Analyze the result:

```bash
python3 src/so101_ros1_bridge/scripts/so101_analyze_csv.py \
  --ignore-start 5 \
  /tmp/so101_joint_traj.csv
```

## Precheck Before Moving Hardware

Validate IK and joint-limit margins without commanding the arm:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --validate-only \
  --pattern xz_sine \
  --x-amplitude 0.01 \
  --z-amplitude 0.01 \
  --frequency 0.02 \
  --duration 30 \
  --rate 20 \
  --csv /tmp/so101_precheck.csv
```

If this fails, reduce amplitude or move the center point away from limits. Keep
the planar 3 arm DOF + 1 gripper DOF configuration; do not unlock
`shoulder_pan` or `wrist_roll` for this hardware setup.

## Current Code Changes

- Online IK now uses multi-start only at the first target by default, then seeds
  subsequent solves from the last accepted IK solution. This reduces IK branch
  switching.
- The launch files now default to `so101_planar_3dof_gripper.yaml`, which means
  shoulder_lift, elbow_flex, wrist_flex, and gripper are commandable while
  shoulder_pan and wrist_roll remain locked.
- The launch files and configs default to `trajectory_interpolation: cubic` for
  smoother timed trajectories.
- The reduced legacy `so101_simplified_3dof.yaml` still exists for diagnosis,
  but it locks `wrist_flex` and is not the preferred config for this planar
  3 arm DOF + gripper setup.

## Configuration Guidance

- Prefer `so101_planar_3dof_gripper.yaml` for this arm: it preserves planar
  motion while keeping all 3 planar arm joints available to IK.
- Use only XZ-plane endpoint targets when `shoulder_pan` is locked; keep
  `y-amplitude` at 0 unless you intentionally unlock pan in a different robot.
- Start with small trajectories: 1 cm X amplitude, 1 cm Z amplitude, 0.02 Hz.
- Use `joint_trajectory` mode for controlled experiments.
- Use `cartesian_stream` only after the planned trajectory path works.

## Hardware Checks

Run a hold/repeatability test:

```bash
rosrun so101_ros1_bridge so101_precision_check.py \
  --sequence ready stow ready \
  --cycles 5 \
  --csv /tmp/so101_precision.csv

python3 src/so101_ros1_bridge/scripts/so101_analyze_csv.py /tmp/so101_precision.csv
```

Watch live servo telemetry:

```bash
rosrun so101_ros1_bridge so101_servo_watch.py all --rate 2
```

If Z error is mostly constant, inspect homing offsets, link/frame calibration,
gravity sag, and static load. If error grows with frequency or amplitude,
inspect servo speed/acceleration/current and trajectory smoothness.

## MuJoCo Testing

The ROS1 bridge now has `mock`, `feetech`, and `mujoco` backends. The mock
backend is a simple velocity-limited joint simulator; it does not model gravity,
contact, servo torque limits, backlash, or serial delay. The MuJoCo backend uses
the existing SO101 MJCF and the same `/so101/command_joint_positions` command
topic as hardware.

Start the MuJoCo backend:

```bash
cd /home/bot/research/so101_mujoco_tracking/ros1_ws
source devel/setup.bash

roslaunch so101_ros1_bridge mujoco_bridge.launch \
  mujoco_model_path:=/home/bot/research/so101_mujoco_tracking/assets/so101/scene.xml \
  trajectory_interpolation:=cubic
```

Then run the same planned trajectory test:

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --execution-mode joint_trajectory \
  --pattern xz_sine \
  --x-amplitude 0.01 \
  --z-amplitude 0.01 \
  --frequency 0.02 \
  --duration 30 \
  --rate 20 \
  --csv /tmp/so101_mujoco_joint_traj.csv
```

A MuJoCo test can reproduce:

- actuator lag from position gains and force limits
- gravity sag if the model and actuator limits are realistic
- oscillation from aggressive setpoints or low damping
- jitter caused by repeatedly resetting single-point targets

MuJoCo will not reproduce real servo backlash, encoder quantization, bus delay,
or gear hysteresis unless those effects are explicitly modeled.

If ROS Noetic's Python cannot import `mujoco`, install MuJoCo into that Python
environment first, or run the non-ROS MuJoCo scripts in the project virtualenv.

# Aerial SO101 Control Architecture

Updated: 2026-07-28 CST

## Role of This Code

This repository is the SO101 arm-side traditional-control stack for Ubuntu 20.04 + ROS Noetic. It should provide a stable, bounded, inspectable actuator interface before any UAV coupling or RL policy is deployed.

It is responsible for:

- reading SO101 joint state from the Feetech follower backend
- enforcing joint limits, locked joints, velocity limits, command timeout, freeze, relax, and estop
- publishing `/so101/joint_states`, `/so101/status`, `/so101/end_effector_pose`, and `/so101/kinematics_status`
- accepting joint deltas, absolute joint targets, gripper commands, and small Cartesian targets
- running repeatable safe pose scripts: stow, ready, grasp, release, return

It is not responsible for:

- direct PX4 motor control
- online RL training on real hardware
- bypassing PX4 failsafe logic
- flying with the SO101 mounted before no-prop, power, center-of-mass, vibration, and failsafe tests are complete

## Independent Control Boundary

Use two independent low-level controllers:

```text
PX4/MAVROS
  owns UAV attitude, position, velocity, arming, mode, and failsafe

SO101 ROS1 bridge
  owns arm joint control, FK/IK, gripper, arm freeze/relax/estop
```

A later whole-body supervisor coordinates them:

```text
policy or operator command
  -> whole-body safety supervisor
  -> PX4/MAVROS high-level setpoint
  -> SO101 joint or Cartesian command
```

The supervisor must be allowed to block arm motion when the UAV is not safe, and block offboard motion when the arm is outside allowed configuration.

## Frame Calibration

`UAV body -> SO101 base_link` is a fixed 6-DOF TF transform: translation plus rotation from the drone body frame to the arm base frame.

A CAD/3D model is a valid first estimate if the mounting geometry is rigid and dimensions are known. It should still be verified on hardware because printed mounts, screw holes, cable strain, and assembly tolerances can shift the arm base by millimetres and degrees.

Once this fixed transform is known, the arm end-effector pose can be transformed into the UAV body frame:

```text
T_uav_ee = T_uav_so101_base * T_so101_base_ee
```

The first term is calibration. The second term is FK from `/so101/joint_states`.

## Workspace Limits

Joint limits protect individual servos. Workspace limits protect the system-level volume where the end effector is allowed to move.

The current Noetic bridge now rejects IK targets outside a simple axis-aligned box in SO101 `base_link`:

```text
x: [0.07, 0.45]
y: [-0.25, 0.25]
z: [0.00, 0.38]
```

This is not full self-collision checking. Mechanical contact between arm links, the gripper, wiring, the drone frame, or payload fixtures still needs separate validation. The MuJoCo assets in `third_party/SO-ARM100-main/Simulation/SO101` are useful for this, but collision meshes and contact pairs must be checked before trusting the simulation as a safety filter.

## Current Hold And Command Strategy

The bridge default command loop is 100 Hz. The smooth test launcher uses 120 Hz, cubic timed-trajectory interpolation, and a command LPF. Both modes use `stale_action: hold_target`.

This means command timeout no longer makes the driver repeatedly freeze to the latest measured joint position. If gravity pulls the arm down a little, the driver keeps commanding the last target instead of accepting the sagged position as the new target.

This is still position control through the STS3215 internal servo controller. It is not an external torque controller and does not model gravity compensation. For aerial use, treat arm commands as bounded position targets with explicit freeze/relax/estop states.

Use `/so101/status` to verify:

```text
"stale": true
"stale_action": "hold_target"
"holding_last_target": true
```

Explicit `/so101/freeze` still captures the current pose as the new hold target. Explicit `/so101/relax` disables torque when configured.

## Precision Validation

Use the installed precision checker after starting the hardware bridge:

```bash
rosrun so101_ros1_bridge so101_precision_check.py \
  --sequence ready stow ready \
  --cycles 5 \
  --duration 2.5 \
  --settle 1.0 \
  --sample-window 5.0 \
  --sample-rate 10 \
  --csv /tmp/so101_precision.csv
```

This measures encoder-level hold drift and repeatability. It does not replace external measurement with a ruler, camera, AprilTag, or depth camera.

## MAVROS Status

`third_party/mavros` is currently a ROS2 branch/package set (`ament_cmake`, branch `ros2`). It is useful as source reference, but it is not directly buildable inside the current ROS Noetic catkin workspace.

For this Ubuntu 20.04 + ROS Noetic system, use distro MAVROS packages or a ROS1 branch when the UAV integration layer is added:

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```

The expected Noetic topics for the first supervisor are:

```text
/mavros/state
/mavros/local_position/pose
/mavros/local_position/velocity_local
/mavros/imu/data
/mavros/setpoint_position/local
/mavros/setpoint_velocity/cmd_vel_unstamped
```

## First Learning Interface

For the first RL or imitation-learning version, do not output raw PWM or raw servo packets.

Recommended action:

```text
[
  drone_position_or_velocity_setpoint_delta,
  yaw_or_yaw_rate_setpoint,
  arm_active_joint_delta,
  gripper_open_close
]
```

Recommended observation:

```text
[
  UAV pose, velocity, attitude, body rates,
  SO101 joint positions,
  SO101 joint command or target positions,
  SO101 end-effector pose in UAV body frame,
  target pose in UAV body frame,
  target pose relative to end effector,
  safety flags and command-valid mask
]
```

Recommended logging topics for datasets:

```text
/mavros/state
/mavros/local_position/pose
/mavros/local_position/velocity_local
/mavros/imu/data
/so101/joint_states
/so101/status
/so101/servo_status
/so101/end_effector_pose
/so101/kinematics_status
/so101/command_joint_positions
/so101/cartesian_target
/tf
```

Use the arm-only recorder before UAV integration:

```bash
scripts/record_so101_arm_bag.sh
```

Use the aerial recorder once MAVROS is running:

```bash
scripts/record_aerial_manipulation_bag.sh
```

## Dynamic Tracking Validation

After static repeatability is acceptable, run single-joint sine tracking:

```bash
rosrun so101_ros1_bridge so101_sine_test.py \
  --joint shoulder_lift \
  --amplitude 0.08 \
  --frequency 0.05 \
  --duration 60 \
  --rate 20 \
  --csv /tmp/so101_sine_shoulder_lift.csv

rosrun so101_ros1_bridge so101_analyze_csv.py /tmp/so101_sine_shoulder_lift.csv
```

Run this for `shoulder_lift`, `elbow_flex`, and `wrist_flex`. Large mean error suggests bias or gravity/load issues. Large RMSE or max absolute error suggests insufficient bandwidth, command limits, or too aggressive trajectory frequency. Rising current or temperature in `/so101/servo_status` suggests overload or excessive gains.

## MuJoCo Contact Scan

Install MuJoCo in the Python environment used for offline analysis:

```bash
python3 -m pip install mujoco
```

Then scan random configurations with the simplified aerial locks:

```bash
scripts/scan_so101_mujoco_self_collision.py \
  --samples 2000 \
  --locked shoulder_pan=0.0 wrist_roll=0.0 gripper=0.5 \
  --csv /tmp/so101_mujoco_contact_scan.csv
```

Treat this as a candidate-contact finder, not final safety proof. Any contact pair found by MuJoCo should be checked against the real arm and then converted into workspace or joint-combination limits.

## Immediate Next Code Layer

After hardware desk validation of the new FK/IK and safe sequences:

1. Verify workspace limits against hardware and MuJoCo self-contact cases.
2. Add a calibrated static transform `uav_body -> so101/base_link`.
3. Publish end-effector pose in UAV body frame.
4. Add a whole-body supervisor that gates arm commands based on UAV state and gates UAV offboard commands based on arm state.
5. Add rosbag logging profiles for training data collection.

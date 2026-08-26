# SO101 Arm Status and Aerial Manipulation Next Steps

Updated: 2026-07-27 CST

## Current Arm-Control Status

The SO101 traditional-control path is functionally brought up for desk testing:

- Motor setup and calibration were completed and exported to the ROS1 bridge config.
- The Noetic ROS1 bridge starts with the Feetech hardware backend.
- The bridge publishes `/so101/joint_states` and `/so101/status`.
- The bridge accepts `/so101/home`, `/so101/command_joint_deltas`, `/so101/command_joint_positions`, `/so101/gripper_command`, `/so101/freeze`, `/so101/relax`, and `/so101/estop`.
- The FK/IK layer now publishes `/so101/end_effector_pose` and accepts `/so101/cartesian_target`.
- The safety-sequence script now supports `stow`, `ready`, `grasp`, `release`, `return`, and `cycle`.
- A hardware smoke test connected successfully and received `status`, `joint_states`, `home`, and `delta` commands.

This means the basic mechanical-arm control interface is complete enough for cautious desktop experiments. It is not yet complete enough for drone-mounted operation.

## Current Simplified Control Mode

The first working mode is a simplified 4-DOF arm:

```text
active joints:
  shoulder_lift
  elbow_flex
  wrist_flex
  gripper

locked joints:
  shoulder_pan = 0.0
  wrist_roll = 0.0
```

This is intentional for first-phase safety. Unlocking `shoulder_pan` or `wrist_roll` should wait until repeatability, limits, and emergency-stop behavior are verified.

## Remaining Arm Work Before Aerial Use

1. Repeatability and safety validation
   - Run repeated home, freeze, relax, estop, and small-delta tests.
   - Record joint state logs for commanded vs. measured motion.
   - Confirm no motor overheats, stalls, or hits mechanical hard stops.

2. Stable device handling
   - Add a udev rule or use `/dev/serial/by-id/...` instead of relying only on `/dev/ttyACM0`.
   - Keep a startup preflight check for port existence, permissions, and port occupancy.

3. Kinematics layer
   - FK and position-only IK are implemented for the URDF `base_link -> gripper_frame_link` chain.
   - Next: add workspace keep-out limits and optional orientation tracking after repeatability is verified.
   - Next: define and publish the fixed transform from UAV body frame to SO101 `base_link`.

4. Higher-level arm control
   - Scripted safety poses are implemented: stow, ready, reach, grasp, release, return.
   - Add command smoothing and workspace constraints in Cartesian space.
   - Add gripper-specific open/close presets and force/current checks if available.

5. Drone integration
   - Define the arm base transform relative to the UAV body frame.
   - Connect arm state into the PX4/MAVROS offboard controller.
   - Add a supervisor that blocks arm motion when the UAV is not in a safe mode.
   - Tie arm estop/freeze into the overall UAV failsafe flow.

6. Flight-readiness testing
   - Bench tests with the drone unpowered or props removed.
   - Power integrity tests with the 12V follower supply and UAV electronics together.
   - Vibration, cable strain-relief, EMI, and center-of-mass checks.
   - No-prop SITL/HITL-style tests before any real flight attempt.

## Role of Imported Open-Source References

- `third_party/Seeed_RoboController`
  - Primary source for Feetech/STS servo tools, scanning, middle calibration, and the local SDK used by this project.

- `third_party/seeed_lerobot` and `third_party/lerobot`
  - Reference for SO101/SO100 hardware setup, calibration conventions, teleoperation, data collection, and future learning workflows.
  - Not imported directly into Noetic runtime because current LeRobot versions expect newer Python than ROS Noetic's Python 3.8.

- `third_party/SO-ARM100-main`
  - Mechanical assembly, CAD, STL/STEP, and hardware reference for SO100/SO101-style arms.
  - Useful for checking joint orientation, mechanical limits, cable routing, and replacement parts.

- `third_party/so101-ros-physical-ai`
  - ROS2 reference for SO101 description, kinematics, MoveIt layout, inference, visualization, and dataset tooling.
  - Used as architecture and URDF/kinematics reference; not directly run as the Noetic control stack.

- `third_party/lerobot-ros`
  - Reference for ROS-to-LeRobot bridge patterns and teleoperator concepts.

- `third_party/mavros`
  - Future PX4 bridge for UAV state, offboard commands, failsafe/status, and whole aerial-manipulation integration.

## Recommended Next Implementation Step

The small kinematics and pose-command layer is now implemented on top of the verified joint-control bridge:

```text
/so101/joint_states
  -> FK / end-effector pose
  -> scripted pose commands
  -> safe Cartesian reach commands
  -> later PX4/MAVROS whole-body supervisor
```

The next code layer should be a whole-body supervisor that subscribes to UAV state, arm state, and safety state, then gates both PX4/MAVROS offboard commands and SO101 joint/Cartesian commands.

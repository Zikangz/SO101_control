# SO101 Noetic Control Plan

## Current Decision

Do not directly convert the ROS2 reference stack into ROS1. The ROS2 packages are useful references for URDF, joint names, MoveIt layout, and policy integration, but the current machine is Ubuntu 20.04 + ROS Noetic + Python 3.8.

The correct first step is a ROS1-native bridge:

```text
ROS1 command topics
  -> safety and mapping layer
  -> mock backend for development
  -> Feetech STS3215 backend for real SO101
  -> /so101/joint_states
```

## Imported References

- `third_party/SO-ARM100-main`: SO101 hardware, meshes, URDF/MJCF.
- `third_party/lerobot`: SO101 follower/leader API and Feetech control reference.
- `third_party/so101-ros-physical-ai`: ROS2 description and architecture reference.
- `third_party/lerobot-ros`: ROS2/LeRobot bridge reference.

## Phase 1 Scope

Implemented now:

- ROS1 catkin packages.
- SO101 robot description package.
- `so101_driver_node.py` with mock and Feetech backends.
- `so101_kinematics_node.py` with URDF-based FK and position-only IK.
- `so101_safe_sequence.py` for stow, ready, grasp, release, return, and cycle.
- JointState publisher.
- End-effector pose publisher.
- Cartesian target subscriber.
- JointTrajectory position command subscriber.
- Float64MultiArray delta command subscriber.
- Gripper command subscriber.
- Home, freeze, relax, and emergency stop topics.
- Locked joint mapping.
- Joint limit and velocity limit filtering.

Not implemented in this phase:

- PX4/MAVROS offboard manager.
- RL policy runner.
- MoveIt planning.
- Full orientation IK.
- UAV-body to arm-base calibration workflow.
- Online learning.
- Drone-mounted real-arm operation.

## Default Simplified Arm

The default 4DOF simplified arm is:

```text
active:
  shoulder_lift
  elbow_flex
  wrist_flex
  gripper

locked:
  shoulder_pan = 0.0
  wrist_roll = 0.0
```

This follows the project document: first prove SO101 traditional control, then later connect to UAV/PX4 and RL outer-loop logic.

## Required Hardware Validation

Before real motion:

1. Confirm SO101 power supply voltage matches motor variant.
2. Confirm motor LEDs and daisy-chain wiring.
3. Confirm port with `lerobot-find-port` or `ls /dev/ttyACM* /dev/ttyUSB*`.
4. Confirm no load, clear workspace, and reachable emergency stop.
5. Run the mock bridge first.
6. Run hardware bridge and only send small deltas.

## Topic Contract

Published:

```text
/so101/joint_states    sensor_msgs/JointState
/so101/status          std_msgs/String JSON
```

Subscribed:

```text
/so101/command_joint_positions   trajectory_msgs/JointTrajectory
/so101/command_joint_deltas      std_msgs/Float64MultiArray
/so101/gripper_command           std_msgs/Float32
/so101/estop                     std_msgs/Bool
/so101/home                      std_msgs/Empty
/so101/freeze                    std_msgs/Empty
/so101/relax                     std_msgs/Empty
```

## Next Step After This Phase

Once the bridge is verified with hardware:

1. Run repeated hardware validation for safe pose sequences and small relative IK offsets.
2. Add workspace keep-out limits and UAV-body to SO101-base static transform.
3. Add MAVROS SITL offboard manager.
4. Add the whole-body safety supervisor.

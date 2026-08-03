# Third-party source snapshots

This directory stores small source snapshots used for local, ROS-free
comparison experiments. They are kept in-tree so the examples in this project
can run without depending on separate sibling checkouts.

## Argo-Robot/controls

- Source: `https://github.com/Argo-Robot/controls.git`
- Reference commit observed locally: `bc82dcdf06f35184be9d216719174a015c92e395`
- Local snapshot: `third_party/Argo-Robot-controls-raw/`
- Usage in this project: `scripts/mujoco_planar_control_sim.py --controller argo_external`

The local Argo snapshot includes only the files needed for the SO101 MuJoCo and
URDF IK comparison path. The integration calls Argo's `URDF_Kinematics` and
matches Argo's `gripper_link` to MuJoCo body `gripper`, so use
`--ee-frame body_gripper` for fair comparisons.

## MoveIt

- Source: `https://github.com/moveit/moveit.git`
- Reference commit observed locally: `fdb79acafb24efd9e138b0d309fb2463186d4ab2`
- Local snapshot: `third_party/moveit/`
- Usage in this project: reference source for trajectory processing; runtime
  comparison uses Python `ruckig` via `--controller moveit_ruckig`.

The local Ubuntu 24.04 setup does not run a full ROS Noetic MoveIt stack.
Instead, this project uses the MoveIt trajectory-processing source as a
reference and implements a ROS-free Ruckig retiming baseline.

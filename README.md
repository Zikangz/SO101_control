# SO-101 MuJoCo Trajectory Tracking

This is the first-stage training sandbox for SO-101 endpoint trajectory tracking
and the second-stage state-based pick-and-lift task.

See `NOTES_AND_USAGE.md` for the local modification log, longer usage notes,
and the control/RL concept map.

See `PICK_LIFT_AND_ACT_NOTES.md` for the MuJoCo cube pick-and-lift task and
LeRobot ACT network troubleshooting notes.

## Source Assets

The SO-101 simulation assets were extracted from:

```text
/home/bot/research/lerobot/SO-ARM100-main.zip
```

The useful files are under:

```text
assets/so101/
  so101_new_calib.xml
  so101_new_calib.urdf
  scene.xml
  scene_pick_lift.xml
  assets/*.stl
```

`so101_new_calib.xml` is already a MuJoCo model with six position actuators and
the endpoint site `gripperframe`.

## Run Smoke Tests

```bash
cd /home/bot/research
source .venvs/lerobot/bin/activate

python so101_mujoco_tracking/scripts/smoke_model.py
python so101_mujoco_tracking/scripts/smoke_env.py
python so101_mujoco_tracking/scripts/smoke_pick_lift_env.py
```

## View and Control in MuJoCo

Open the SO-101 model in MuJoCo's official viewer:

```bash
python so101_mujoco_tracking/scripts/view_so101.py
```

Open the pick-and-lift scene with cube and target marker:

```bash
python so101_mujoco_tracking/scripts/view_so101.py --model pick_lift
```

You can also launch the viewer directly:

```bash
python -m mujoco.viewer --mjcf so101_mujoco_tracking/assets/so101/scene.xml
```

In the right-side viewer UI, open the actuator/control section and move the
position actuator sliders.

Keyboard-control individual joints:

```bash
python so101_mujoco_tracking/scripts/keyboard_control_so101.py
```

Controls:

- `1-6`: select actuator
- `q/a`: increase/decrease selected actuator
- `r`: reset home pose
- `p`: pause/resume
- `Esc`: exit

## Local MuJoCo Control Simulation

For Ubuntu 24.04 without ROS, use the pure Python MuJoCo control replay script.
It keeps the SO101 as a planar 3 arm DOF + 1 gripper DOF system:

- commandable arm joints: `shoulder_lift`, `elbow_flex`, `wrist_flex`
- locked spatial joints: `shoulder_pan`, `wrist_roll`
- gripper command: normalized `0.0` closed to `1.0` open

Run the recommended preplanned joint-trajectory controller for one full
figure-eight cycle:

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller joint_trajectory \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

Watch MuJoCo and the live target/endpoint/error plots together. Red is the
current target marker and green is the endpoint marker in the MuJoCo viewer:

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller joint_trajectory \
  --viewer \
  --viewer-real-time \
  --live-plot \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

Compare the three local controller baselines:

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_like argo_like \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

Compare the current controller against the downloaded MoveIt/Ruckig-style
retiming baseline and the downloaded Argo-Robot/controls IK baseline. Argo's
URDF `gripper_link` aligns with the MuJoCo body `gripper`, so this comparison
uses `--ee-frame body_gripper`:

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_ruckig argo_external \
  --ee-frame body_gripper \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

Scan the planar XZ workspace. `config` uses conservative hardware-style limits;
`mujoco` uses the original MuJoCo XML joint ranges for simulation exploration:

```bash
python so101_mujoco_tracking/scripts/scan_planar_workspace.py \
  --limit-profile config \
  --ee-frame body_gripper

python so101_mujoco_tracking/scripts/scan_planar_workspace.py \
  --limit-profile mujoco \
  --ee-frame body_gripper
```

Run a larger rear/backbend trajectory in simulation. This deliberately uses
MuJoCo limits and starts from the first IK target so the initial front-to-rear
move does not dominate the error:

```bash
python so101_mujoco_tracking/scripts/compare_planar_controllers.py \
  --controllers joint_trajectory moveit_ruckig argo_external \
  --ee-frame body_gripper \
  --limit-profile mujoco \
  --center -0.04 0 0.32 \
  --x-amplitude 0.06 \
  --z-amplitude 0.02 \
  --frequency 0.05 \
  --cycles 1 \
  --start-at-first-target \
  --plan-multistart-every-target \
  --output-dir /tmp/so101_backbend_compare_true_external
```

Run the shared real-time Cartesian servo law in MuJoCo. This is the local
simulation analogue of the ROS node used for hardware:

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller servo \
  --servo-input velocity \
  --cycles 1 \
  --frequency 0.1 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

Run the less preferred online single-target streaming mode for comparison:

```bash
python so101_mujoco_tracking/scripts/mujoco_planar_control_sim.py \
  --controller cartesian_stream \
  --cycles 1 \
  --frequency 0.05 \
  --x-amplitude 0.03 \
  --z-amplitude 0.03
```

The script saves CSV and trajectory plots under:

```text
so101_mujoco_tracking/outputs/mujoco_planar_control_sim/
```

Add `--record-video` to simulator, comparison, IK baseline, training, or
evaluation commands to export MP4 demos next to the CSV/PNG results. For a quick
reporting clip without opening the interactive viewer:

```bash
python so101_mujoco_tracking/scripts/view_so101.py \
  --model pick_lift \
  --duration 5 \
  --record-video
```

Example generated media paths:

```text
outputs/mujoco_planar_control_sim/joint_trajectory_tracking.png
outputs/planar_controller_comparison/controller_comparison.png
outputs/eval_pick_lift_v1/pick_lift_eval_episode0.png
outputs/view_so101/pick_lift_view.mp4
```

Current controller pipeline:

1. Generate an XZ-plane sinusoidal endpoint target.
2. Solve planar endpoint IK with damped least squares using MuJoCo's site/body
   Jacobian, only over `shoulder_lift`, `elbow_flex`, and `wrist_flex`.
3. In `joint_trajectory` mode, precompute the full joint path and execute it
   through `JointSafetyFilter.set_timed_trajectory()`.
4. In `moveit_like` mode, keep the same IK path and apply a lightweight
   MoveIt-style velocity-limited retiming before execution.
5. In `moveit_ruckig` mode, use Python Ruckig as a ROS-free local analogue of
   MoveIt's jerk-limited trajectory smoothing/time parameterization.
6. In `argo_like` mode, stream DLS IK joint targets directly to MuJoCo position
   actuators, matching the control style used by Argo's simple MuJoCo demos.
7. In `argo_external` mode, call the downloaded Argo-Robot/controls
   `URDF_Kinematics` implementation directly.
8. In `servo` mode, run the same resolved-rate Cartesian servo used by the
   ROS hardware path: Cartesian target/velocity -> DLS Jacobian velocity ->
   Ruckig or simple joint limiter -> streaming joint setpoints.
9. `JointSafetyFilter` applies joint limits, max velocity, and cubic Hermite
   interpolation between timed waypoints.
10. MuJoCo position actuators track the commanded joint targets while MuJoCo
   simulates gravity, damping, friction loss, actuator force limits, and contact.

## ROS1 Noetic Real-Time Servo

For Jetson Xavier NX on Ubuntu 20.04 / ROS Noetic / Python 3.8, use the ROS1
workspace under `ros1_ws/`. The hardware path intentionally uses Python 3.8
compatible code and keeps Ruckig optional.

Build on the Jetson:

```bash
cd ~/SO101_control/ros1_ws
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install numpy pyyaml "feetech-servo-sdk>=1.0.0,<2.0.0"
python3 -m pip install ruckig  # optional; falls back to SimpleJointLimiter if unavailable
catkin_make
source devel/setup.bash
```

Start the hardware bridge with the servo node enabled:

```bash
roslaunch so101_ros1_bridge hardware_bridge.launch \
  port:=/dev/ttyACM0 \
  with_servo:=true \
  command_rate_hz:=100 \
  servo_rate_hz:=100
```

Send a very small velocity test first:

```bash
rostopic pub -r 20 /so101/ee_velocity_cmd geometry_msgs/TwistStamped \
  "header: {frame_id: 'base_link'}
twist:
  linear: {x: 0.005, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}"
```

Stop streaming and hold:

```bash
rostopic pub /so101/servo_disable std_msgs/Empty "{}" --once
```

Watch diagnostics:

```bash
rostopic echo /so101/status
rostopic echo /so101/servo_status
rostopic echo /so101/cartesian_servo_status
```

`/so101/servo_status` is the low-level Feetech/MuJoCo backend telemetry.
`/so101/cartesian_servo_status` is the Cartesian servo controller state.

The desktop RL/MuJoCo modules under `so101_tracking/` and some standalone
scripts use newer Python typing and are intended for the local training
environment. They should not be mixed into the Jetson Noetic runtime unless you
backport their annotations or run them in a separate Python environment.

## Traditional IK Baseline

Before training RL for a trajectory task, run a classical damped-least-squares
IK baseline:

```bash
python so101_mujoco_tracking/scripts/ik_baseline.py
```

This saves:

```text
so101_mujoco_tracking/outputs/ik_baseline/ik_tracking.csv
so101_mujoco_tracking/outputs/ik_baseline/ik_tracking.png
```

Run a standalone randomized 3D IK tracking demo without any RL training:

```bash
python so101_mujoco_tracking/scripts/ik_random_tracking.py \
  --segments 8 \
  --steps-per-segment 80 \
  --seed 0
```

Watch that IK-only random tracking demo in MuJoCo:

```bash
python so101_mujoco_tracking/scripts/ik_random_tracking.py \
  --viewer \
  --real-time \
  --segments 8 \
  --steps-per-segment 80 \
  --seed 0
```

For a fair comparison, keep `--episode-steps`, `--frame-skip`, `--ik-iters`,
`--gain`, `--damping`, and `--max-dq` aligned with the SAC environment settings.
The IK baseline is a kinematic Jacobian controller that writes joint-position
targets into MuJoCo's position actuators; MuJoCo still simulates the arm
dynamics, actuator force limits, gravity, damping, friction loss, and collisions.

## Stage-1 Task

- Observation: joint positions, joint velocities, endpoint position, target position, trajectory phase.
- Action: five arm-joint position deltas. The gripper is held fixed.
- Target: a 3D circular/Lissajous-like trajectory.
- Reward: endpoint distance, action magnitude, action smoothness, joint velocity.

## SAC Training

Short smoke run:

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 1000 \
  --learning-starts 100 \
  --eval-freq 500 \
  --save-freq 500 \
  --run-name smoke
```

Longer first run:

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --run-name first_tracking_sac
```

Train while watching the MuJoCo motion:

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --control-mode ik_residual \
  --viewer \
  --viewer-real-time \
  --run-name first_tracking_sac_viewer
```

Use `--viewer` without `--viewer-real-time` if you want maximum training speed
and only need occasional visual feedback. Use `--viewer-speed 0.5` for slow
motion, or `--viewer-speed 2.0` for 2x playback while still throttling display.

Evaluate the latest trained model:

```bash
python so101_mujoco_tracking/scripts/eval_tracking.py
```

Evaluate generalization on randomized smooth 3D trajectories:

```bash
python so101_mujoco_tracking/scripts/eval_tracking.py \
  --episodes 10 \
  --trajectory-mode random \
  --output-dir so101_mujoco_tracking/outputs/eval_random_latest
```

Train on randomized smooth 3D trajectories:

```bash
python so101_mujoco_tracking/scripts/train_sac.py \
  --total-timesteps 200000 \
  --control-mode ik_residual \
  --trajectory-mode random \
  --run-name residual_random_tracking
```

Monitor training:

```bash
tensorboard --logdir so101_mujoco_tracking/outputs/sac
```

## Stage-2 Pick-and-Lift

The first grasping task is SO-101 cube pick-and-lift with state-based
observations.

- Observation: robot joint state, endpoint position, cube position/velocity,
  goal position, relative vectors, gripper state, grasp flag, episode phase.
- Action: default `ee_delta`, a 4D action `[dx, dy, dz, gripper]`.
- Control: DLS IK maps endpoint delta to arm joint position targets; MuJoCo
  simulates gravity, damping, friction, actuator force limits, floor/cube contact.
- Grasping: default `virtual_grasp=True` for a stable first RL task; use
  `--no-virtual-grasp` later for contact-only experiments.
- Training now supports scripted demo replay prefill and BC warm start, matching
  the useful part of the official HIL-SERL recipe for this custom SO-101 env.

Run the scripted smoke test:

```bash
python so101_mujoco_tracking/scripts/smoke_pick_lift_env.py
```

Train SAC with scripted demos and BC warm start:

```bash
python so101_mujoco_tracking/scripts/train_pick_lift_sac.py \
  --total-timesteps 300000 \
  --control-mode ee_delta \
  --demo-prefill-episodes 50 \
  --bc-pretrain-steps 2000 \
  --demo-noise 0.03 \
  --run-name pick_lift_demo_bc_sac_v1
```

Evaluate:

```bash
python so101_mujoco_tracking/scripts/eval_pick_lift.py \
  --episodes 20 \
  --output-dir so101_mujoco_tracking/outputs/eval_pick_lift_v1
```

Watch evaluation in MuJoCo:

```bash
python so101_mujoco_tracking/scripts/eval_pick_lift.py \
  --episodes 3 \
  --viewer \
  --real-time
```

## Recommended Stage-1 Protocol

1. Run the IK baseline and record mean/final tracking error.
2. Run SAC in `--control-mode ik_residual`; the policy learns a residual on top
   of the same DLS IK controller instead of solving the full reaching problem
   from scratch.
3. Compare `outputs/ik_baseline/ik_tracking.csv` with the SAC evaluation CSV.
4. Only switch to `--control-mode joint_delta` after the residual setup reaches
   centimeter-level error; direct joint-delta RL is intentionally harder.

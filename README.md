# SO101 ROS Noetic Control Stack

This workspace is for the first project phase: traditional SO101 arm control and debugging on Ubuntu 20.04 + ROS Noetic.

## Current Status

This branch is suitable as the ROS1/Noetic control baseline for desk testing, calibration, repeatability checks, and conservative hardware operation. It is not yet a final aerial-flight controller: keep aerial use behind mock, no-prop, dummy-load, power, and emergency-stop validation before mounting on a UAV.

## Quick Deploy

```bash
git clone https://github.com/Zikangz/SO101_control.git
cd SO101_control
export SO101_ROOT="$PWD"

# Requires Ubuntu 20.04 with ROS Noetic already installed.
scripts/bootstrap_noetic.sh
source ros1_ws/devel/setup.bash
roslaunch so101_ros1_bridge mock_bridge.launch
```

For hardware calibration helpers that depend on Seeed tools, run:

```bash
scripts/bootstrap_noetic.sh --with-third-party
```

The current implementation deliberately does not convert the upstream ROS2 stack wholesale. Instead, it provides a ROS1 bridge around the SO101 control concepts:

- `so101_description`: ROS1/catkin robot description package adapted from the ROS2 reference assets.
- `so101_ros1_bridge`: ROS1 nodes for JointState publication, joint position/delta commands, FK/IK, locked joints, velocity limits, home/freeze/relax, scripted safety poses, and emergency stop.
- `third_party/`: upstream references only. Do not edit these directly.

## Jetson Xavier NX Deploy

Target platform for the current handoff:

- Jetson Xavier NX with Ubuntu 20.04.
- ROS Noetic installed from the Ubuntu 20.04 ROS packages.
- Python 3.8 from the system ROS environment.
- SO101 follower connected as `/dev/ttyACM0` unless udev assigns another port.
- Separate matched servo power supply for the 12V follower arm. Do not power the servos from Jetson. Use a common ground between Jetson/PX4 and the servo supply before UAV integration.

This is a Jetson-deployable SO101 arm-side baseline, not a final aerial-flight whole-body controller. Use it first for bench testing, no-prop vehicle testing, logging, power validation, and later PX4/MAVROS integration.

### Option A: deploy from GitHub

Use the handoff branch pushed after the 2026-09-03 experiments:

```bash
cd ~
git clone -b so101-aerial-handoff-20260903 git@github.com:Zikangz/SO101_control.git SO101
cd ~/SO101
export SO101_ROOT="$PWD"
```

If SSH keys are not configured on the Jetson, use HTTPS instead:

```bash
cd ~
git clone -b so101-aerial-handoff-20260903 https://github.com/Zikangz/SO101_control.git SO101
cd ~/SO101
export SO101_ROOT="$PWD"
```

### Option B: deploy from the local tarball

Copy `so101_jetson_deploy_20260904.tar.gz` to the Jetson, then run:

```bash
cd ~
mkdir -p SO101
tar -xzf so101_jetson_deploy_20260904.tar.gz -C SO101
cd ~/SO101
export SO101_ROOT="$PWD"
```

Verify the package if the `.sha256` file is copied with it:

```bash
sha256sum -c so101_jetson_deploy_20260904.tar.gz.sha256
```

### Install dependencies and build

Run these commands in a normal terminal. Do not activate the LeRobot conda environment while building or running ROS Noetic nodes.

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
source /opt/ros/noetic/setup.bash

python3 -m pip install --user -U \
  "importlib-metadata>=6.8,<8" \
  "setuptools>=65,<70" \
  "wheel>=0.38,<0.46"

python3 -m pip install --user -r requirements-noetic.txt

cd ros1_ws
catkin_make
source devel/setup.bash
```

Optional one-command bootstrap after ROS Noetic is installed:

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
scripts/bootstrap_noetic.sh
source ros1_ws/devel/setup.bash
```

### Check the port and servos

Plug in the follower arm USB adapter and servo power, then run:

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
export PORT=/dev/ttyACM0

ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
scripts/so101_servo_status.py "$PORT" --scan
```

Expected state before enabling the bridge:

- IDs `1-6` should be visible.
- Voltage should match the follower supply, around 12V for the current arm.
- Temperature should be normal at idle.
- PID columns should be left at the servo's current EEPROM values. The current workflow does not rewrite P/I/D.

### Terminal A: start the hardware bridge without PID overwrite

This wrapper enables torque, then launches the hardware bridge with `configure_motors_on_connect:=false`. The servo internal position PID still works; the bridge simply does not overwrite PID registers.

```bash
cd ~/SO101
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
~/SO101/scripts/run_ros_hardware_bridge_no_pid.sh "$PORT"
```

Keep this terminal running.

### Terminal B: verify ROS state and lock the arm

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
export ROS_HOME=/tmp/so101_ros_home
export ROS_LOG_DIR=/tmp/so101_ros_logs
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source ~/SO101/ros1_ws/devel/setup.bash

rostopic list | grep /so101
rosrun so101_ros1_bridge so101_control_cli.py state
rosrun so101_ros1_bridge so101_control_cli.py named ready --duration 4.0
sleep 1
rosrun so101_ros1_bridge so101_control_cli.py state
rosrun so101_ros1_bridge so101_control_cli.py freeze
```

If the visual `ready` pose is clearly wrong, stop and recheck factory middle/calibration before running trajectories. Do not solve a bad middle pose by relaxing joint limits.

### Terminal C: optional live trajectory plot

Use this on Jetson only if a display or X forwarding is available. For headless Jetson runs, skip live plotting and use offline plotting after CSV capture.

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
export ROS_HOME=/tmp/so101_ros_home
export ROS_LOG_DIR=/tmp/so101_ros_logs
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source ~/SO101/ros1_ws/devel/setup.bash

mkdir -p ~/SO101/so101_plots
rosrun so101_ros1_bridge so101_plot_ee_trajectory.py \
  --live \
  --history-seconds 120 \
  --update-rate-hz 15 \
  --reset-on-trajectory \
  --save ~/SO101/so101_plots/so101_live_trajectory_jetson.png
```

### Terminal D: reproduce the current best desktop baseline

The current recommended baseline is `xz_vertex_diamond + z_bias=0.0083`, `f=0.1 Hz`, `100 x 40 mm`. Do not use the tested `phasexz_v1_safe` profile as the default, because it regressed versus this baseline.

```bash
cd ~/SO101
export SO101_ROOT="$PWD"
export ROS_HOME=/tmp/so101_ros_home
export ROS_LOG_DIR=/tmp/so101_ros_logs
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
source /opt/ros/noetic/setup.bash
source ~/SO101/ros1_ws/devel/setup.bash

rosrun so101_ros1_bridge so101_control_cli.py named ready --duration 4.0
sleep 1
rosrun so101_ros1_bridge so101_control_cli.py freeze

STAMP=$(date +%Y%m%d_%H%M%S)
CSV=/tmp/so101_ee_xz_vertex_diamond_f100_x050_z020_zbias0083_jetson_${STAMP}.csv

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
  --prep-pose ready \
  --prep-duration 4.0 \
  --move-to-center-duration 10 \
  --center-joint-tolerance 0.03 \
  --center-max-start-error 0.04 \
  --joint-limit-margin 0.080 \
  --z-feedforward-bias 0.0083 \
  --csv "$CSV"

if [ ! -s "$CSV" ]; then
  echo "ERROR: CSV was not generated: $CSV"
  echo "Check Terminal A and the so101_ee_sine_test.py output before retrying."
  exit 1
fi

rosrun so101_ros1_bridge so101_analyze_csv.py --ignore-start 10 "$CSV"

mkdir -p ~/SO101/so101_plots
rosrun so101_ros1_bridge so101_plot_ee_trajectory.py \
  --csv "$CSV" \
  --output-dir ~/SO101/so101_plots

echo "$CSV"
```

Reference desktop result for comparison: mean EE error `4.700 mm`, RMSE `5.037 mm`, max `8.381 mm`, Z high-pass RMS `0.573 mm`. A Jetson result does not need to match exactly on the first run, but large regression usually means port assignment, calibration, power, USB latency, or mechanical mounting changed.

### Jetson deployment checklist

Before using the Jetson setup near a UAV, confirm:

1. `scripts/so101_servo_status.py "$PORT" --scan` sees all six servos.
2. `ready` and `freeze` hold the expected physical pose.
3. The diamond baseline finishes and writes a CSV.
4. The CSV has no IK command failures or rejected execution samples.
5. Minimum servo voltage stays close to the desktop baseline and does not show repeated drops.
6. Emergency stop, freeze, and relax commands are tested while the arm is unloaded.
7. No-prop UAV bench tests pass before any propeller-on test.

Useful stop commands:

```bash
rosrun so101_ros1_bridge so101_control_cli.py freeze
rosrun so101_ros1_bridge so101_control_cli.py estop on
rosrun so101_ros1_bridge so101_control_cli.py relax
```

### Sync the handoff to a USB drive

The tested USB path on the development machine is `/media/zzk/Ventoy/ZZK/SO101`.
Run this from a normal local terminal, not from a restricted sandbox, because
the USB mount must be writable:

```bash
cd /home/zzk/ZZK/SO101
scripts/sync_so101_to_usb.sh /media/zzk/Ventoy/ZZK/SO101
```

The script writes two locations on the USB drive:

- `/media/zzk/Ventoy/ZZK/SO101`: deploy-relevant project tree.
- `/media/zzk/Ventoy/ZZK/SO101_JetsonDeploy_20260904`: small clean handoff folder with the Jetson tarball, checksum, README, command notes, and meeting report.

It also verifies `so101_jetson_deploy_20260904.tar.gz` with sha256 on the USB
drive. Do not format the development machine until that verification prints
`成功` or `OK` and the GitHub branch `so101-aerial-handoff-20260903` has the
latest commits you need.

## Why Not Directly Use Latest LeRobot Inside ROS Noetic?

The cloned LeRobot repository currently requires Python >= 3.12, while ROS Noetic on Ubuntu 20.04 uses Python 3.8. The bridge is therefore written as a Noetic-native adapter:

- `mock` backend for software and ROS graph testing without hardware.
- `feetech` backend for STS3215 hardware via `scservo_sdk` when installed.

LeRobot remains useful for calibration, teleoperation, dataset tools, and API reference, but it should not be imported directly into a Noetic node unless you pin/use a Python 3.8-compatible LeRobot version.

## Build

```bash
export SO101_ROOT="${SO101_ROOT:-$PWD}"
cd $SO101_ROOT/ros1_ws
catkin_make
source devel/setup.bash
```

## Run Mock Bridge

```bash
roslaunch so101_ros1_bridge mock_bridge.launch
```

Then in another terminal:

```bash
source $SO101_ROOT/ros1_ws/devel/setup.bash
rosrun so101_ros1_bridge so101_control_cli.py status
rosrun so101_ros1_bridge so101_control_cli.py state
rosrun so101_ros1_bridge so101_control_cli.py home
rosrun so101_ros1_bridge so101_control_cli.py delta 0.02 -0.02 0.01 0.0
rosrun so101_ros1_bridge so101_control_cli.py ee
rosrun so101_ros1_bridge so101_safe_sequence.py ready --duration 1.5
rosrun so101_ros1_bridge so101_precision_check.py --sequence ready --cycles 1 --sample-window 2.0
rosrun so101_ros1_bridge so101_control_cli.py estop on
rosrun so101_ros1_bridge so101_control_cli.py estop off
```

`delta` without `--full` uses active joint order:
`shoulder_lift`, `elbow_flex`, `wrist_flex`, `gripper`.

## FK / IK and Safe Poses

Both `mock_bridge.launch` and `hardware_bridge.launch` now start `so101_kinematics_node.py` by default when `with_kinematics:=true`.

Published:

```text
/so101/end_effector_pose   geometry_msgs/PoseStamped
/so101/kinematics_status   std_msgs/String JSON
/so101/servo_status        std_msgs/String JSON
```

Subscribed:

```text
/so101/cartesian_target    geometry_msgs/PoseStamped
```

Useful commands:

```bash
source $SO101_ROOT/ros1_ws/devel/setup.bash

# Read current FK result.
rosrun so101_ros1_bridge so101_control_cli.py ee
rosrun so101_ros1_bridge so101_control_cli.py kin-status

# Move to a small relative Cartesian offset. Keep offsets small first.
rosrun so101_ros1_bridge so101_control_cli.py cartesian -0.02 0.00 -0.02 --relative

# Safe named poses and sequences.
rosrun so101_ros1_bridge so101_control_cli.py named list
rosrun so101_ros1_bridge so101_safe_sequence.py stow
rosrun so101_ros1_bridge so101_safe_sequence.py ready
rosrun so101_ros1_bridge so101_safe_sequence.py grasp
rosrun so101_ros1_bridge so101_safe_sequence.py release
rosrun so101_ros1_bridge so101_safe_sequence.py return
```

The current IK is position-only and uses the simplified safe mode. `shoulder_pan` and `wrist_roll` remain locked unless the config is deliberately changed. Some Cartesian targets are unreachable in this mode; check `/so101/kinematics_status` after every IK command.

The driver runs at 30 Hz by default and uses `stale_action: hold_target`. After a command times out, `/so101/status` may show `"stale": true`; that is now only a status flag. The driver continues holding the last target unless you explicitly call `/so101/freeze`, `/so101/relax`, or `/so101/estop`.

Precision and drift test:

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

Analyze that CSV:

```bash
rosrun so101_ros1_bridge so101_analyze_csv.py /tmp/so101_precision.csv
```

Single-joint sine tracking test:

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

Offline end-effector trajectory validation. This does not require roscore, does not open `/dev/ttyACM0`, and does not move hardware. Use this first when changing trajectory size or center. This helper uses total peak-to-peak dimensions; for example, `--total-width 0.120` means `x_amp=0.060`.

```bash
python3 $SO101_ROOT/ros1_ws/src/so101_ros1_bridge/scripts/so101_validate_ee_trajectory.py \
  --total-width 0.120 \
  --total-height 0.080 \
  --samples 180 \
  --limits-mode both \
  --scan-center \
  --output-prefix $SO101_ROOT/logs/trajectory_validation/ee_120w_80h
```

For the current locked q1/q5 planar mode, do not use the former `(0.360,
0.000, 0.150)` center. The guarded center below keeps a usable margin for the
same 120 mm x 60 mm path:

```text
base_link center = x 0.380 m, y 0.000 m, z 0.160 m
```

This command uses no unverified Cartesian bias and requires at least 0.08 rad
from every active joint hard limit. It does not start ROS or move the arm.

```bash
python3 $SO101_ROOT/ros1_ws/src/so101_ros1_bridge/scripts/so101_validate_ee_trajectory.py \
  --pattern figure8_xz \
  --center 0.380 0.0 0.160 \
  --total-width 0.120 \
  --total-height 0.060 \
  --z-feedforward-bias 0.000 \
  --joint-limit-margin 0.080 \
  --samples 240 \
  --limits-mode both \
  --output-prefix $SO101_ROOT/logs/trajectory_validation/ee_120w_60h_center_038_016_guarded
```

End-effector XZ figure-eight tracking test through the IK node. Preflight first; `--validate-only` does not move hardware. Use `xz_sine` for the locked-q1 planar 8-shaped path: `x=x0+x_amp*sin(t)`, `z=z0+z_amp*sin(2t)`. The older online `figure8` pattern keeps a `0.5*z_amplitude` scale for its z term, so do not use it when you mean a precise XZ peak-to-peak height.

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --validate-only \
  --pattern xz_sine \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.060 \
  --z-amplitude 0.030 \
  --z-feedforward-bias 0.000 \
  --joint-limit-margin 0.080 \
  --frequency 0.03 \
  --duration 90 \
  --rate 60 \
  --csv /tmp/so101_validate_120w_60h_guarded.csv

rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --pattern xz_sine \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.060 \
  --z-amplitude 0.030 \
  --z-feedforward-bias 0.000 \
  --joint-limit-margin 0.080 \
  --frequency 0.03 \
  --duration 90 \
  --rate 60 \
  --ramp-duration 8 \
  --csv /tmp/so101_ee_120w_60h_guarded.csv

rosrun so101_ros1_bridge so101_analyze_csv.py /tmp/so101_ee_120w_60h_guarded.csv
```

If the same upright, unloaded desk trajectory still has a repeatable lower-arc
error after the no-bias baseline, fit a bounded phase-dependent profile from
that baseline. This is only valid for the exact center, amplitudes, frequency,
payload, and arm mounting orientation used to create it. It is never valid for
a flying, tilted, inverted, or accelerating UAV.

```bash
mkdir -p $SO101_ROOT/logs/compensation

rosrun so101_ros1_bridge so101_fit_phase_z_compensation.py \
  /tmp/so101_ee_120w_60h_no_bias.csv \
  --frequency 0.03 \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.060 \
  --z-amplitude 0.030 \
  --exclude-start 12 \
  --exclude-end 5 \
  --max-abs-bias 0.012 \
  --output $SO101_ROOT/logs/compensation/ee_120w_60h_003hz.json

rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --validate-only \
  --pattern xz_sine \
  --center 0.380 0.0 0.160 \
  --x-amplitude 0.060 \
  --z-amplitude 0.030 \
  --frequency 0.03 \
  --duration 90 \
  --rate 60 \
  --joint-limit-margin 0.080 \
  --phase-z-compensation-profile $SO101_ROOT/logs/compensation/ee_120w_60h_003hz.json
```

The profile is deliberately rejected if its trajectory metadata differs from
the requested test. Compare its resulting CSV against the no-bias baseline;
do not retain it if the lower-arc RMS or the peak Cartesian error gets worse.

Low-frequency feedforward compensation should be layered only after the
q2/q3 internal PID is stable. The current preferred order is:

```text
1. Tune STS3215 internal position PID until q2/q3 tracking is repeatable.
2. Fit a bench-only phase Z profile for one exact path, payload, and mounting.
3. For aerial use, replace the bench phase profile with a model that uses
   joint angles, velocity direction, current/load feedback, and effective
   gravity in the arm base frame.
4. Keep any real-time feedback correction low-bandwidth, bounded, and disabled
   when feedback is stale, voltage sags, or the arm is near a joint limit.
```

The STS3215 current/load registers are useful for observing load and detecting
repeatable bias, but they are not calibrated joint torque sensors. Do not run a
fast ROS outer PI loop from these values. For this position-controlled servo
chain, use them first for logging and offline feedforward fitting, then for a
small bounded position bias only after repeatability tests show the same error
pattern.

Plot the planned IK target, filtered command, and feedback FK after a run.
The offline plot writes PNG files beside the CSV and does not connect to the
hardware. Cartesian coordinates are displayed in `base_link` millimetres;
joint angles are radians.

```bash
source /opt/ros/noetic/setup.bash
source $SO101_ROOT/ros1_ws/devel/setup.bash

rosrun so101_ros1_bridge so101_plot_ee_trajectory.py \
  --csv $SO101_ROOT/logs/smoothness/20260730_202411/ee_xz_sine_120w_60h_cycle_1.csv
```

For a live display, start this in another desktop terminal before the motion
test. It only subscribes to `/so101/command_joint_positions`,
`/so101/commanded_joint_states`, and `/so101/joint_states`; it never commands
the arm.

```bash
source /opt/ros/noetic/setup.bash
source $SO101_ROOT/ros1_ws/devel/setup.bash

rosrun so101_ros1_bridge so101_plot_ee_trajectory.py \
  --live \
  --history-seconds 120 \
  --update-rate-hz 15 \
  --save /tmp/so101_live_trajectory.png
```

For `xz_sine`, `--x-amplitude` and `--z-amplitude` are half-amplitudes in
metres: `0.060` is 120 mm total X width and `0.030` is 60 mm total Z height.
`--frequency` is cycles per second. `--rate` is the number of planned IK
waypoints per second, not the servo speed. Keep `--rate` at 50 or higher for
the timed trajectory. `--ramp-duration` is the smooth amplitude ramp at the
start and should be increased for larger moves. `--joint-limit-margin` is a
pre-execution safety condition: a target can be geometrically reachable while
still being unsuitable for smooth physical execution if it is too close to a
joint limit.

Use this order for physical trials: first validate the geometry, then change
one quantity at a time. A Gerono XZ figure-eight is analytically smooth, but
its left and right extrema have a vertical tangent. A visible straight segment
there is therefore not an interpolation corner; in the previous trace it was
made much worse by q2/q4 saturation. The guarded center above prevents that
saturation. Use no unverified Z feedforward bias when testing a new center,
payload, temperature, or UAV attitude.

```bash
rosrun so101_ros1_bridge so101_ee_sine_test.py \
  --validate-only \
  --pattern xz_sine \
  --center 0.360 0.0 0.115 \
  --x-amplitude 0.070 \
  --z-amplitude 0.035 \
  --frequency 0.020 \
  --duration 90 \
  --rate 50 \
  --csv /tmp/so101_validate_140w_70h.csv
```

The ROS outer position assist is disabled by default. Do not enable its P or I
gain for this arm while diagnosing the current shoulder error: its feedback is
delayed by serial reads, and it previously introduced a bounded correction on
top of the planned q2 command. That correction can make a compliant shoulder
visibly oscillate. Use the STS3215 internal position loop for stiffness, after
measuring the static error at the center.

```bash
cd $SO101_ROOT
SO101_COMMAND_RATE_HZ=80 \
SO101_FEEDBACK_READ_RATE_HZ=25 \
SO101_LPF_ALPHA=1.0 \
SO101_FEEDBACK_POSITION_ASSIST_GAIN=0.0 \
SO101_FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN=0.0 \
scripts/run_ros_hardware_bridge_smooth.sh /dev/ttyACM0

# In another terminal. This records the q2 hold error and never starts the 8 path.
scripts/run_so101_center_tracking_diagnostic.sh
```

The new trajectory preflight rejects an enabled outer assist. The status JSON
must show `feedback_position_assist.enabled=false`; during a center move it
should also show `transport.last_nonempty_write_mode=sync_write_pos_ex`. If
the static CSV still reports q2 error near 0.05 rad after 12 to 15 seconds,
check the shoulder horn, link screws, 12 V supply under load, and calibration;
then compare one conservative STS3215 internal-P change for q2 only. Do not
raise the ROS I gain or joint limits to make the trajectory start.

The current stable bench-tested internal PID defaults are written into the
launch/config files and the smooth bridge wrapper:

```text
default servo PID:      P=16 I=0 D=32
q2 shoulder_lift PID:   P=32 I=0 D=32
q3 elbow_flex PID:      P=28 I=0 D=32
q4 wrist_flex PID:      P=16 I=0 D=32
```

With these defaults, start the bridge without manually passing PID environment
variables:

```bash
cd $SO101_ROOT
scripts/run_ros_hardware_bridge_smooth.sh /dev/ttyACM0
```

The q2=32/q3=28 setting repeated three times on the 120 mm by 60 mm XZ path
with about 10.22 mm mean EE error, 11.03 mm RMSE, and -9.46 mm mean Z bias.
If another arm build or payload needs retuning, override one joint at a time
from the shell and compare CSV outputs before changing the default files again.
Stop increasing P if the joint audibly buzzes, temperature rises quickly, supply
voltage sags, or the CSV high-pass error grows.

A same-size faster trial is `x=0.060`, `z=0.030`, `frequency=0.040`; it has
an estimated max planned joint speed of 0.181 rad/s, below the current 0.5 to
0.6 rad/s software limits. Do not make both changes at once while the shoulder
and elbow tracking error remains above 0.03 rad RMS.

Live telemetry for only the joint currently being tested:

```bash
rosrun so101_ros1_bridge so101_servo_watch.py shoulder_lift --rate 2
rosrun so101_ros1_bridge so101_servo_watch.py all --rate 1
```

Batch precision suite, with separate CSV files per test:

```bash
scripts/run_so101_precision_suite.sh
scripts/analyze_so101_precision_suite.sh
```

The default output directory is:

```text
$SO101_ROOT/logs/precision/<YYYYmmdd_HHMMSS>/
```

Arm-only rosbag recording:

```bash
scripts/record_so101_arm_bag.sh
```

Combined MAVROS + SO101 state topic for aerial integration:

```bash
roslaunch so101_ros1_bridge aerial_state_bridge.launch
rostopic echo -n 1 /aerial_manipulation/state
scripts/check_aerial_mavros_so101.sh
```

### UAV Gravity And Inertial Load Observation

`--z-feedforward-bias` is a bench-only Cartesian position pre-compensation. It
can compensate a repeatable offset at one arm posture, payload, flight attitude
and motion condition, but it is not torque gravity compensation. Do not reuse a
fixed Z bias after the UAV tilts, inverts, changes payload, or accelerates.

Real-time compensation is possible, but not as direct torque gravity control.
The safe implementation for this STS3215 position-mode arm is a slow feedforward
position bias model:

```text
inputs:  q2/q3/q4 position, planned velocity direction, current/load feedback,
         supply voltage, temperature, and effective gravity from MAVROS IMU
output:  small bounded joint-space or Cartesian Z command bias
rate:    low bandwidth only; never a high-gain outer PI loop
guards:  disable or decay to zero when feedback is stale, voltage is low,
         temperature is high, or joint-limit margin is small
```

For desktop tests, the existing phase profile tool is the first feedforward
step. For aerial manipulation, use `/so101/load_compensation_state` as an
additional model input instead of reusing the desktop phase profile.

The observer below converts the MAVROS IMU attitude into gravity expressed in
the arm base frame and reports the effective gravity `g - a`. It publishes only
`/so101/load_compensation_state` and never commands the arm. Its
`static_bias_allowed` flag is false when IMU data is stale, the tilt from the
upright calibration attitude exceeds 10 degrees, or filtered linear acceleration
exceeds 0.5 m/s^2.

```bash
roslaunch so101_ros1_bridge load_compensation_observer.launch \
  uav_to_arm_roll_rad:=0.0 \
  uav_to_arm_pitch_rad:=0.0 \
  uav_to_arm_yaw_rad:=0.0

rostopic echo /so101/load_compensation_state
```

The three `uav_to_arm_*` values must be replaced by the calibrated fixed rotation
from the arm base frame to the PX4/MAVROS body frame. Verify the convention on a
restrained vehicle: upright must report gravity near `z=-9.81` in arm coordinates;
an inverted vehicle must report approximately `z=+9.81` and
`static_bias_allowed=false`. The combined aerial state topic includes this
observer output when both nodes are running.

MuJoCo contact scan, if the `mujoco` Python package is installed:

```bash
python3 -m pip install mujoco
scripts/scan_so101_mujoco_self_collision.py \
  --samples 2000 \
  --csv /tmp/so101_mujoco_contact_scan.csv

# Or use the timestamped wrapper:
scripts/run_so101_mujoco_contact_scan.sh
```

## Run Hardware Bridge

Do not run the hardware bridge before motor ID setup and calibration are complete. For your current hardware:

- OS: Ubuntu 20.04 / ROS Noetic / Python 3.8 for ROS.
- SO101 follower port: `/dev/ttyACM0`.
- Follower power: 12V follower variant. Use only the matching 12V supply for this arm.
- LeRobot setup/calibration should be done in a separate Python environment, not inside the sourced ROS Noetic shell.

Install optional hardware dependencies in the ROS Python environment:

```bash
python3 -m pip install pyserial "feetech-servo-sdk>=1.0.0,<2.0.0"
```

Start with a safe port and hardware unplugged from load:

```bash
cd $SO101_ROOT
scripts/run_ros_hardware_bridge.sh /dev/ttyACM0
```

The first real-hardware sequence should be:

1. Verify power, common ground, and red motor LEDs.
2. Complete `lerobot-setup-motors` for the follower on `/dev/ttyACM0`.
3. Complete calibration with `scripts/run_seeed_lerobot_calibrate.sh /dev/ttyACM0 aerial_so101_follower`.
4. Check the calibration with `scripts/check_so101_calibration.py --limited-wrist-roll` before exporting it to ROS.
5. Start `mock_bridge.launch` and test ROS commands.
6. Start `hardware_bridge.launch` with the arm unloaded and clear of obstacles.
7. Echo `/so101/joint_states`.
8. Send `/so101/home`.
9. Test small deltas only.
10. Test `/so101/estop`.

## Notes

- Command topics use ROS standard messages only. No custom messages are required for the first phase.
- Arm joints use radians in ROS topics. `/so101/gripper_command` uses normalized `0.0` closed to `1.0` open.
- Main command topics: `/so101/command_joint_deltas`, `/so101/command_joint_positions`, `/so101/cartesian_target`, `/so101/gripper_command`, `/so101/home`, `/so101/freeze`, `/so101/relax`, `/so101/estop`.
- The default simplified arm locks `shoulder_pan` and `wrist_roll` at zero.
- Do not mount the arm on a drone until desk tests, no-prop tests, dummy load tests, power tests, and emergency stop tests are complete.

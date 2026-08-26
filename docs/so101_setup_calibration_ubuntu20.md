# SO101 Setup and Calibration on Ubuntu 20.04

This guide follows the Seeed SO-ARM100/SO101 LeRobot flow, adapted for this project:

- Target machine now and later: Ubuntu 20.04.
- Runtime robotics stack: ROS Noetic, Python 3.8.
- SO101 follower port: `/dev/ttyACM0`.
- SO101 follower power: 12V follower variant.

## Important Compatibility Rule

Keep two environments separate:

```text
LeRobot setup/calibration environment
  -> used for lerobot-find-port, lerobot-setup-motors, lerobot-calibrate

ROS Noetic environment
  -> used for roslaunch so101_ros1_bridge ...
```

Do not source ROS Noetic inside the LeRobot environment unless you know exactly why. Do not import current LeRobot directly inside ROS Noetic nodes; the current cloned LeRobot requires newer Python than Ubuntu 20.04's ROS Python.

## Recommended LeRobot Setup Environment

Use the Seeed tutorial's LeRobot source for hardware setup, because it is the vendor-tested path for this hardware:

```bash
cd $SO101_ROOT/third_party
git clone https://github.com/Seeed-Projects/lerobot.git seeed_lerobot
cd seeed_lerobot
```

Create a separate Python environment. Conda/miniforge is preferred on Jetson because it avoids disturbing system Python:

```bash
conda create -y -n lerobot-so101 python=3.10
conda activate lerobot-so101
pip install -e ".[feetech]"
```

If dependency resolution fails on Jetson Xavier NX, pin to the exact Seeed tutorial revision or use the tutorial's provided environment commands first.

## Port and Permissions

Your follower is `/dev/ttyACM0`.

Check:

```bash
ls -l /dev/ttyACM0
```

Temporary permission fix:

```bash
sudo chmod 666 /dev/ttyACM0
```

More permanent option:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in.

## Motor ID Setup

Only connect the one motor requested by the prompt during ID setup. Do not connect the whole daisy chain for this step unless the command explicitly asks for it.

```bash
conda activate lerobot-so101
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
```

Expected follower IDs:

```text
shoulder_pan   1
shoulder_lift  2
elbow_flex     3
wrist_flex     4
wrist_roll     5
gripper        6
```

## Calibration

Use one stable calibration id. Recommended:

```text
aerial_so101_follower
```

Run the project wrapper:

```bash
cd $SO101_ROOT
scripts/run_seeed_lerobot_calibrate.sh /dev/ttyACM0 aerial_so101_follower
```

Optional passive monitor in another terminal. This does not open the serial port;
it only reads the status file written by the calibration process:

```bash
watch -n 0.1 'cat /tmp/so101_current_motor_status.txt 2>/dev/null || echo waiting-for-calibration'
```

During calibration:

1. Start from the middle of the range.
2. For each range recording step: press Enter to start recording, move that joint through its usable range, then press Enter again to stop.
3. Do not force a joint into hard stops.
4. Keep the 12V power supply matched to this follower variant.
5. This self-assembled follower treats `wrist_roll` as mechanically limited by default; test it through its safe usable range only.

## Export Calibration for ROS Bridge

After calibration, run:

```bash
cd $SO101_ROOT
SO101_WRIST_ROLL_LIMITED=1 .conda-so101-noetic/bin/python scripts/check_so101_calibration.py --limited-wrist-roll calibration/aerial_so101_follower.json
scripts/apply_so101_calibration_to_ros.sh aerial_so101_follower
```

## ROS Noetic Bridge Test

Open a new terminal for ROS:

```bash
cd $SO101_ROOT/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch so101_ros1_bridge mock_bridge.launch
```

After mock passes, test hardware:

```bash
roslaunch so101_ros1_bridge hardware_bridge.launch port:=/dev/ttyACM0
```

Then:

```bash
rostopic echo /so101/joint_states
rostopic pub /so101/home std_msgs/Empty "{}" --once
rostopic pub /so101/command_joint_deltas std_msgs/Float64MultiArray "data: [0.0, 0.02, -0.02, 0.01, 0.0, 0.0]" --once
rostopic pub /so101/estop std_msgs/Bool "data: true" --once
```

## Current Open Items

Before I would recommend real movement, confirm:

1. Whether you have only follower, or follower + leader.
2. Whether the follower motors already have factory IDs or are completely unset.
3. Whether the 12V follower power supply can provide enough current for all six STS3215 motors.

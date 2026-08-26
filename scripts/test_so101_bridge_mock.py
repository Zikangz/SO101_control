#!/usr/bin/env python3
"""Small smoke test for the ROS1 mock bridge.

Run after:
  roslaunch so101_ros1_bridge mock_bridge.launch
"""

import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def wait_for_joint_state(timeout_s=5.0):
    msg = rospy.wait_for_message("/so101/joint_states", JointState, timeout=timeout_s)
    return dict(zip(msg.name, msg.position))


def main():
    rospy.init_node("test_so101_bridge_mock", anonymous=True)
    pub = rospy.Publisher("/so101/command_joint_deltas", Float64MultiArray, queue_size=1)
    time.sleep(0.5)
    before = wait_for_joint_state()
    pub.publish(Float64MultiArray(data=[0.0, 0.05, -0.05, 0.02, 0.0, 0.0]))
    time.sleep(1.0)
    after = wait_for_joint_state()
    print("Before:", before)
    print("After: ", after)
    if abs(after.get("shoulder_pan", 0.0)) > 1e-6:
        print("shoulder_pan should be locked at 0.0", file=sys.stderr)
        return 1
    if abs(after.get("wrist_roll", 0.0)) > 1e-6:
        print("wrist_roll should be locked at 0.0", file=sys.stderr)
        return 1
    if abs(after.get("shoulder_lift", 0.0) - before.get("shoulder_lift", 0.0)) < 1e-4:
        print("shoulder_lift did not move in mock backend", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

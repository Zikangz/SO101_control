#!/usr/bin/env python3
"""Command-line helper for safe SO101 ROS1 control."""

import argparse
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, String
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.poses import ACTIVE_JOINTS_4DOF, JOINT_ORDER, SAFE_POSES


ACTIVE_JOINTS = ACTIVE_JOINTS_4DOF
FULL_JOINTS = JOINT_ORDER


def parse_joint_values(items):
    values = {}
    for item in items:
        if "=" not in item:
            raise argparse.ArgumentTypeError("Expected name=value, got %r" % item)
        name, value = item.split("=", 1)
        if name not in FULL_JOINTS:
            raise argparse.ArgumentTypeError("Unknown joint %r" % name)
        values[name] = float(value)
    return values


def wait_for_subscribers(pub, timeout_s=2.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            return True
        time.sleep(0.02)
    return pub.get_num_connections() > 0


def publish_once(pub, msg):
    wait_for_subscribers(pub)
    pub.publish(msg)
    rospy.sleep(0.2)


def print_joint_state(timeout_s):
    msg = rospy.wait_for_message("/so101/joint_states", JointState, timeout=timeout_s)
    print("joint positions:")
    for name, pos in zip(msg.name, msg.position):
        unit = "norm" if name == "gripper" else "rad"
        print("  %-14s %.4f %s" % (name, pos, unit))


def print_status(timeout_s, topic="/so101/status"):
    msg = rospy.wait_for_message(topic, String, timeout=timeout_s)
    print(msg.data)


def command_home():
    pub = rospy.Publisher("/so101/home", Empty, queue_size=1, latch=True)
    publish_once(pub, Empty())


def command_freeze():
    pub = rospy.Publisher("/so101/freeze", Empty, queue_size=1, latch=True)
    publish_once(pub, Empty())


def command_relax():
    pub = rospy.Publisher("/so101/relax", Empty, queue_size=1, latch=True)
    publish_once(pub, Empty())


def command_estop(enabled):
    pub = rospy.Publisher("/so101/estop", Bool, queue_size=1, latch=True)
    publish_once(pub, Bool(data=enabled))


def command_deltas(args):
    joints = FULL_JOINTS if args.full else ACTIVE_JOINTS
    if len(args.values) != len(joints):
        raise SystemExit("Expected %d values for %s" % (len(joints), ", ".join(joints)))
    pub = rospy.Publisher("/so101/command_joint_deltas", Float64MultiArray, queue_size=1, latch=True)
    publish_once(pub, Float64MultiArray(data=list(args.values)))


def publish_joint_positions(positions, duration=1.0):
    msg = JointTrajectory()
    msg.joint_names = list(positions.keys())
    point = JointTrajectoryPoint()
    point.positions = [positions[name] for name in msg.joint_names]
    point.time_from_start = rospy.Duration(duration)
    msg.points = [point]
    pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1, latch=True)
    publish_once(pub, msg)


def command_pose(args):
    publish_joint_positions(parse_joint_values(args.assignments), args.duration)


def command_named(args):
    if args.name == "list":
        for name in sorted(SAFE_POSES):
            print("%-8s %s" % (name, SAFE_POSES[name]))
        return
    if args.name not in SAFE_POSES:
        raise SystemExit("Unknown pose %r. Use: %s" % (args.name, ", ".join(sorted(SAFE_POSES))))
    publish_joint_positions(SAFE_POSES[args.name], args.duration)


def command_gripper(value):
    pub = rospy.Publisher("/so101/gripper_command", Float32, queue_size=1, latch=True)
    publish_once(pub, Float32(data=max(0.0, min(1.0, float(value)))))


def command_cartesian(args):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = args.frame
    x, y, z = args.xyz
    if args.relative:
        current = rospy.wait_for_message("/so101/end_effector_pose", PoseStamped, timeout=args.timeout)
        pose.header.frame_id = current.header.frame_id or args.frame
        pose.pose.position.x = current.pose.position.x + x
        pose.pose.position.y = current.pose.position.y + y
        pose.pose.position.z = current.pose.position.z + z
        pose.pose.orientation = current.pose.orientation
    else:
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        q = quaternion_from_euler(args.rpy[0], args.rpy[1], args.rpy[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
    pub = rospy.Publisher("/so101/cartesian_target", PoseStamped, queue_size=1, latch=True)
    publish_once(pub, pose)


def build_parser():
    parser = argparse.ArgumentParser(description="SO101 ROS1 control helper")
    parser.add_argument("--timeout", type=float, default=5.0, help="Timeout for read commands")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("state", help="Print one /so101/joint_states sample")
    sub.add_parser("status", help="Print one /so101/status sample")
    sub.add_parser("kin-status", help="Print one /so101/kinematics_status sample")
    sub.add_parser("ee", help="Print one /so101/end_effector_pose sample")
    sub.add_parser("home", help="Move to configured home positions")
    sub.add_parser("freeze", help="Hold current positions")
    sub.add_parser("relax", help="Disable servo torque after freezing")

    estop = sub.add_parser("estop", help="Enable or clear emergency stop")
    estop.add_argument("value", choices=["on", "off"])

    delta = sub.add_parser("delta", help="Send joint deltas in radians; gripper is normalized")
    delta.add_argument("values", nargs="+", type=float)
    delta.add_argument("--full", action="store_true", help="Use full joint order instead of active joint order")

    pose = sub.add_parser("pose", help="Send absolute joint positions as name=value")
    pose.add_argument("assignments", nargs="+")
    pose.add_argument("--duration", type=float, default=1.0)

    named = sub.add_parser("named", help="Send a conservative named pose: list/stow/ready/reach/grasp/release/return")
    named.add_argument("name", choices=sorted(SAFE_POSES) + ["list"])
    named.add_argument("--duration", type=float, default=1.5)

    cart = sub.add_parser("cartesian", help="Send an IK target in base_link frame")
    cart.add_argument("xyz", nargs=3, type=float, help="x y z in metres; offsets when --relative is set")
    cart.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0], help="roll pitch yaw radians; IK is currently position-only")
    cart.add_argument("--frame", default="base_link")
    cart.add_argument("--relative", action="store_true", help="Treat xyz as offsets from /so101/end_effector_pose")

    gripper = sub.add_parser("gripper", help="Set gripper normalized command, 0 closed to 1 open")
    gripper.add_argument("value", type=float)
    return parser


def main():
    args = build_parser().parse_args()
    rospy.init_node("so101_control_cli", anonymous=True)
    try:
        if args.command == "state":
            print_joint_state(args.timeout)
        elif args.command == "status":
            print_status(args.timeout)
        elif args.command == "kin-status":
            print_status(args.timeout, "/so101/kinematics_status")
        elif args.command == "ee":
            msg = rospy.wait_for_message("/so101/end_effector_pose", PoseStamped, timeout=args.timeout)
            print(
                "ee pose frame=%s xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)"
                % (
                    msg.header.frame_id,
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                )
            )
        elif args.command == "home":
            command_home()
        elif args.command == "freeze":
            command_freeze()
        elif args.command == "relax":
            command_relax()
        elif args.command == "estop":
            command_estop(args.value == "on")
        elif args.command == "delta":
            command_deltas(args)
        elif args.command == "pose":
            command_pose(args)
        elif args.command == "named":
            command_named(args)
        elif args.command == "cartesian":
            command_cartesian(args)
        elif args.command == "gripper":
            command_gripper(args.value)
        return 0
    except rospy.ROSException as exc:
        print("ROS error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

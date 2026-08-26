#!/usr/bin/env python3
"""Run conservative named SO101 safety pose sequences."""

import argparse
import json
import os
import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.poses import SAFE_POSES, SAFE_SEQUENCES


ALIASES = {
    "park": "stow",
    "home": "ready",
    "prepare": "ready",
    "pick": "grasp",
    "open": "release",
    "back": "return",
}


def _status_or_none(timeout_s):
    try:
        msg = rospy.wait_for_message("/so101/status", String, timeout=timeout_s)
    except rospy.ROSException:
        return None
    try:
        return json.loads(msg.data)
    except ValueError:
        return {"raw": msg.data}


def _check_arm_ready(timeout_s):
    status = _status_or_none(timeout_s)
    if status is None:
        raise RuntimeError("No /so101/status received; start the bridge first")
    if status.get("estop"):
        raise RuntimeError("SO101 estop is active; clear it before moving")
    backend_error = status.get("last_backend_error") or ""
    if backend_error:
        raise RuntimeError("SO101 backend reports an error: %s" % backend_error)
    return status


def _current_positions(timeout_s):
    msg = rospy.wait_for_message("/so101/joint_states", JointState, timeout=timeout_s)
    return dict(zip(msg.name, msg.position))


def _publish_pose(pub, pose_name, pose, duration_s):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = list(pose.keys())
    point = JointTrajectoryPoint()
    point.positions = [float(pose[name]) for name in msg.joint_names]
    point.time_from_start = rospy.Duration(duration_s)
    msg.points = [point]
    rospy.loginfo("SO101 pose %-8s -> %s", pose_name, pose)
    pub.publish(msg)


def _wait_until_close(target, timeout_s, tolerance, republish_cb=None):
    deadline = time.time() + timeout_s
    last_republish = 0.0
    errors = {}
    while time.time() < deadline and not rospy.is_shutdown():
        if republish_cb is not None and time.time() - last_republish >= 0.8:
            republish_cb()
            last_republish = time.time()
        current = _current_positions(timeout_s=1.0)
        errors = {
            name: abs(float(current.get(name, 0.0)) - float(value))
            for name, value in target.items()
        }
        if all(error <= tolerance for error in errors.values()):
            return True, errors
    return False, errors


def run_sequence(args):
    sequence_name = ALIASES.get(args.sequence, args.sequence)
    if sequence_name not in SAFE_SEQUENCES:
        raise RuntimeError(
            "Unknown sequence %r. Use one of: %s"
            % (args.sequence, ", ".join(sorted(SAFE_SEQUENCES)))
        )

    steps = SAFE_SEQUENCES[sequence_name]
    print("SO101 sequence:", sequence_name, "->", " -> ".join(steps))
    for step in steps:
        print("  %-8s %s" % (step, SAFE_POSES[step]))
    if args.dry_run:
        return 0

    _check_arm_ready(args.timeout)
    _current_positions(args.timeout)
    pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1, latch=True)
    deadline = time.time() + args.timeout
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    for step in steps:
        pose = SAFE_POSES[step]

        def _republish(step=step, pose=pose):
            _publish_pose(pub, step, pose, args.duration)

        _republish()
        ok, errors = _wait_until_close(
            pose,
            timeout_s=max(args.timeout, args.duration + args.settle),
            tolerance=args.tolerance,
            republish_cb=None if args.no_republish else _republish,
        )
        if not ok:
            raise RuntimeError("Pose %s did not converge; errors=%s" % (step, errors))
        if args.hold > 0:
            rospy.sleep(args.hold)
    print("SO101 sequence complete:", sequence_name)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Run conservative SO101 safety pose sequences")
    parser.add_argument(
        "sequence",
        choices=sorted(set(SAFE_SEQUENCES) | set(ALIASES)),
        help="Sequence to execute: stow/ready/grasp/release/return/cycle",
    )
    parser.add_argument("--duration", type=float, default=2.5, help="Nominal duration per pose")
    parser.add_argument("--timeout", type=float, default=6.0, help="Timeout per status/read/pose")
    parser.add_argument("--tolerance", type=float, default=0.08, help="Joint convergence tolerance")
    parser.add_argument("--settle", type=float, default=1.0, help="Extra wait budget after duration")
    parser.add_argument("--hold", type=float, default=0.3, help="Pause after each reached pose")
    parser.add_argument("--dry-run", action="store_true", help="Print the sequence without commanding motion")
    parser.add_argument("--no-republish", action="store_true", help="Do not refresh targets while waiting")
    return parser


def main():
    rospy.init_node("so101_safe_sequence", anonymous=True)
    try:
        return run_sequence(build_parser().parse_args())
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 sequence error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

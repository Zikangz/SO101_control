#!/usr/bin/env python3
"""Measure SO101 encoder repeatability and hold drift through safe poses."""

import argparse
import csv
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.poses import SAFE_POSES


def _read_joints(timeout_s):
    msg = rospy.wait_for_message("/so101/joint_states", JointState, timeout=timeout_s)
    return dict(zip(msg.name, msg.position))


def _read_ee(timeout_s):
    msg = rospy.wait_for_message("/so101/end_effector_pose", PoseStamped, timeout=timeout_s)
    return {
        "ee_x": msg.pose.position.x,
        "ee_y": msg.pose.position.y,
        "ee_z": msg.pose.position.z,
    }


def _publish_pose(pub, pose, duration_s):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = list(pose.keys())
    point = JointTrajectoryPoint()
    point.positions = [float(pose[name]) for name in msg.joint_names]
    point.time_from_start = rospy.Duration(duration_s)
    msg.points = [point]
    pub.publish(msg)


def _wait_close(target, timeout_s, tolerance):
    deadline = time.time() + timeout_s
    errors = {}
    while time.time() < deadline and not rospy.is_shutdown():
        current = _read_joints(1.0)
        errors = {name: abs(float(current.get(name, 0.0)) - float(value)) for name, value in target.items()}
        if all(error <= tolerance for error in errors.values()):
            return True, current, errors
    return False, _read_joints(1.0), errors


def _sample(duration_s, rate_hz, timeout_s):
    rate = rospy.Rate(rate_hz)
    deadline = time.time() + duration_s
    rows = []
    while time.time() < deadline and not rospy.is_shutdown():
        row = {"t": time.time()}
        row.update(_read_joints(timeout_s))
        try:
            row.update(_read_ee(timeout_s))
        except rospy.ROSException:
            pass
        rows.append(row)
        rate.sleep()
    return rows


def _ranges(rows, keys):
    result = {}
    for key in keys:
        values = [float(row[key]) for row in rows if key in row]
        if values:
            result[key] = max(values) - min(values)
    return result


def _write_csv(path, rows):
    if not path or not rows:
        return
    keys = sorted({key for row in rows for key in row})
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def run(args):
    unknown = [name for name in args.sequence if name not in SAFE_POSES]
    if unknown:
        raise RuntimeError("Unknown pose(s): %s" % ", ".join(unknown))

    pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1, latch=True)
    deadline = time.time() + args.timeout
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    all_rows = []
    final_rows = []
    for cycle in range(args.cycles):
        for pose_name in args.sequence:
            pose = SAFE_POSES[pose_name]
            rospy.loginfo("cycle=%d pose=%s target=%s", cycle + 1, pose_name, pose)
            _publish_pose(pub, pose, args.duration)
            ok, _current, errors = _wait_close(
                pose,
                timeout_s=max(args.timeout, args.duration + args.settle),
                tolerance=args.tolerance,
            )
            if not ok:
                raise RuntimeError("Pose %s did not converge; errors=%s" % (pose_name, errors))
            rospy.sleep(args.settle)
            samples = _sample(args.sample_window, args.sample_rate, args.timeout)
            for row in samples:
                row["cycle"] = cycle + 1
                row["pose"] = pose_name
            all_rows.extend(samples)
            if samples:
                final_rows.append(samples[-1])

    keys = ["shoulder_lift", "elbow_flex", "wrist_flex", "gripper", "ee_x", "ee_y", "ee_z"]
    hold_ranges = _ranges(all_rows, keys)
    repeat_ranges = _ranges(final_rows, keys)
    _write_csv(args.csv, all_rows)

    print("SO101 precision check complete")
    print("samples:", len(all_rows))
    print("hold drift range:")
    for key, value in sorted(hold_ranges.items()):
        unit = "m" if key.startswith("ee_") else "rad/norm"
        print("  %-14s %.6f %s" % (key, value, unit))
    print("repeatability range:")
    for key, value in sorted(repeat_ranges.items()):
        unit = "m" if key.startswith("ee_") else "rad/norm"
        print("  %-14s %.6f %s" % (key, value, unit))
    if args.csv:
        print("csv:", args.csv)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="SO101 encoder repeatability and hold-drift check")
    parser.add_argument("--sequence", nargs="+", default=["ready", "stow", "ready"])
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--sample-window", type=float, default=3.0)
    parser.add_argument("--sample-rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--tolerance", type=float, default=0.08)
    parser.add_argument("--csv", default="")
    return parser


def main():
    rospy.init_node("so101_precision_check", anonymous=True)
    try:
        return run(build_parser().parse_args())
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 precision check error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

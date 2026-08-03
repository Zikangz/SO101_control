#!/usr/bin/env python3
"""Run a single-joint sine tracking test and save measured response to CSV."""

import argparse
import csv
import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.poses import JOINT_ORDER


def _latest_joint_state(timeout_s):
    msg = rospy.wait_for_message("/so101/joint_states", JointState, timeout=timeout_s)
    return dict(zip(msg.name, msg.position))


def _latest_ee(timeout_s):
    try:
        msg = rospy.wait_for_message("/so101/end_effector_pose", PoseStamped, timeout=timeout_s)
    except rospy.ROSException:
        return {}
    return {"ee_x": msg.pose.position.x, "ee_y": msg.pose.position.y, "ee_z": msg.pose.position.z}


class ServoStatusCache:
    def __init__(self):
        self.latest = {}
        rospy.Subscriber("/so101/servo_status", String, self._on_status, queue_size=1)

    def _on_status(self, msg):
        try:
            self.latest = json.loads(msg.data).get("joints", {})
        except ValueError:
            self.latest = {}

    def flatten_joint(self, joint):
        data = self.latest.get(joint, {})
        return {
            "servo_voltage_v": data.get("voltage_v", ""),
            "servo_temperature_c": data.get("temperature_c", ""),
            "servo_current_ma": data.get("current_ma", ""),
            "servo_load_raw": data.get("load_raw", ""),
            "servo_velocity_raw": data.get("velocity_raw", ""),
            "servo_moving": data.get("moving", ""),
        }


def _publish_target(pub, joint, target, duration_s):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = [joint]
    point = JointTrajectoryPoint()
    point.positions = [float(target)]
    point.time_from_start = rospy.Duration(duration_s)
    msg.points = [point]
    pub.publish(msg)


def _write_csv(path, rows):
    keys = sorted({key for row in rows for key in row})
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def run(args):
    if args.joint not in JOINT_ORDER:
        raise RuntimeError("Unknown joint %r; expected one of %s" % (args.joint, ", ".join(JOINT_ORDER)))
    if args.frequency <= 0.0:
        raise RuntimeError("--frequency must be > 0")
    if args.rate <= 0.0:
        raise RuntimeError("--rate must be > 0")

    current = _latest_joint_state(args.timeout)
    center = current.get(args.joint, 0.0) if args.center is None else args.center
    print(
        "SO101 sine test joint=%s center=%.5f amplitude=%.5f frequency=%.5fHz duration=%.2fs"
        % (args.joint, center, args.amplitude, args.frequency, args.duration)
    )

    cache = ServoStatusCache()
    pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1, latch=True)
    deadline = time.time() + args.timeout
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    rows = []
    rate = rospy.Rate(args.rate)
    start = time.time()
    last_target = center
    while not rospy.is_shutdown():
        now = time.time()
        elapsed = now - start
        if elapsed > args.duration:
            break
        target = center + args.amplitude * math.sin(2.0 * math.pi * args.frequency * elapsed)
        _publish_target(pub, args.joint, target, 1.0 / args.rate)
        last_target = target

        measured = _latest_joint_state(args.timeout)
        row = {
            "t": now,
            "elapsed": elapsed,
            "joint": args.joint,
            "target": target,
            "measured": measured.get(args.joint, ""),
            "error": measured.get(args.joint, 0.0) - target if args.joint in measured else "",
        }
        for name in JOINT_ORDER:
            if name in measured:
                row[name] = measured[name]
        row.update(_latest_ee(args.timeout))
        row.update(cache.flatten_joint(args.joint))
        rows.append(row)
        rate.sleep()

    if args.return_center:
        _publish_target(pub, args.joint, center, args.return_duration)
    _write_csv(args.csv, rows)
    print("samples:", len(rows))
    print("last_target:", last_target)
    print("csv:", args.csv)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="SO101 single-joint sine tracking test")
    parser.add_argument("--joint", default="shoulder_lift")
    parser.add_argument("--center", type=float, default=None, help="Default: current joint position")
    parser.add_argument("--amplitude", type=float, default=0.12)
    parser.add_argument("--frequency", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--csv", default="/tmp/so101_sine.csv")
    parser.add_argument("--return-center", action="store_true", default=True)
    parser.add_argument("--no-return-center", dest="return_center", action="store_false")
    parser.add_argument("--return-duration", type=float, default=2.0)
    return parser


def main():
    rospy.init_node("so101_sine_test", anonymous=True)
    try:
        return run(build_parser().parse_args())
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 sine test error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

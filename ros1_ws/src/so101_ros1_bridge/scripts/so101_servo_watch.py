#!/usr/bin/env python3
"""Print live telemetry for one SO101 servo from /so101/servo_status."""

import argparse
import json
import os
import sys
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.poses import JOINT_ORDER


class TelemetryCache:
    def __init__(self):
        self.servo_status = {}
        self.joint_positions = {}
        self.last_servo_stamp = 0.0
        self.last_joint_stamp = 0.0
        rospy.Subscriber("/so101/servo_status", String, self._on_servo_status, queue_size=1)
        rospy.Subscriber("/so101/joint_states", JointState, self._on_joint_state, queue_size=1)

    def _on_servo_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        self.servo_status = payload.get("joints", {})
        self.last_servo_stamp = time.time()

    def _on_joint_state(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))
        self.last_joint_stamp = time.time()

    def ready(self, joint):
        return joint in self.servo_status or joint in self.joint_positions

    def row(self, joint):
        servo = self.servo_status.get(joint, {})
        pos = self.joint_positions.get(joint, "")
        return {
            "joint": joint,
            "position": pos,
            "raw_position": servo.get("raw_position", ""),
            "voltage_v": servo.get("voltage_v", ""),
            "temperature_c": servo.get("temperature_c", ""),
            "current_ma": servo.get("current_ma", ""),
            "load_raw": servo.get("load_raw", ""),
            "velocity_raw": servo.get("velocity_raw", ""),
            "moving": servo.get("moving", ""),
            "servo_age_s": time.time() - self.last_servo_stamp if self.last_servo_stamp else "",
        }


def _fmt(value, precision=4):
    if value == "":
        return "-"
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return ("%." + str(precision) + "f") % value
    return str(value)


def _print_row(row):
    print(
        "{joint:14s} pos={pos:>9s} raw={raw:>5s} V={volt:>5s} T={temp:>4s}C "
        "I={current:>8s}mA load={load:>7s} vel={vel:>7s} moving={moving:>1s} age={age:>5s}s".format(
            joint=row["joint"],
            pos=_fmt(row["position"], 5),
            raw=_fmt(row["raw_position"], 0),
            volt=_fmt(row["voltage_v"], 1),
            temp=_fmt(row["temperature_c"], 0),
            current=_fmt(row["current_ma"], 1),
            load=_fmt(row["load_raw"], 0),
            vel=_fmt(row["velocity_raw"], 0),
            moving=_fmt(row["moving"], 0),
            age=_fmt(row["servo_age_s"], 2),
        )
    )


def run(args):
    joints = JOINT_ORDER if args.joint == "all" else [args.joint]
    unknown = [name for name in joints if name not in JOINT_ORDER]
    if unknown:
        raise RuntimeError("Unknown joint(s): %s" % ", ".join(unknown))

    cache = TelemetryCache()
    deadline = time.time() + args.timeout
    while time.time() < deadline and not rospy.is_shutdown():
        if all(cache.ready(joint) for joint in joints):
            break
        rospy.sleep(0.05)
    else:
        raise RuntimeError("No /so101/servo_status or /so101/joint_states for %s" % ", ".join(joints))

    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        if not args.no_clear:
            sys.stdout.write("\033[2J\033[H")
        for joint in joints:
            _print_row(cache.row(joint))
        sys.stdout.flush()
        if args.once:
            break
        rate.sleep()
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Watch SO101 servo diagnostics for one joint")
    parser.add_argument("joint", choices=JOINT_ORDER + ["all"], help="Joint to print, or all")
    parser.add_argument("--rate", type=float, default=2.0, help="Terminal refresh rate")
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--once", action="store_true", help="Print one sample and exit")
    parser.add_argument("--no-clear", action="store_true", help="Do not clear terminal between updates")
    return parser


def main():
    rospy.init_node("so101_servo_watch", anonymous=True)
    try:
        return run(build_parser().parse_args())
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 servo watch error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

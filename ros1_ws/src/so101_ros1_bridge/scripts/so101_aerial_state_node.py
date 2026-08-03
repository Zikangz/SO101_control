#!/usr/bin/env python3
"""Publish one JSON status topic combining MAVROS and SO101 arm state."""

import json
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String


def _stamp_age(stamp):
    if stamp is None:
        return None
    return max(0.0, time.time() - stamp)


def _vec3(vec):
    return {"x": vec.x, "y": vec.y, "z": vec.z}


def _quat(q):
    return {"x": q.x, "y": q.y, "z": q.z, "w": q.w}


def _parse_json(data):
    try:
        return json.loads(data)
    except ValueError:
        return {"raw": data}


class AerialManipulationStateNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.fresh_timeout_s = float(rospy.get_param("~fresh_timeout_s", 1.0))
        self.out_topic = rospy.get_param("~out_topic", "/aerial_manipulation/state")

        self.mavros_state = None
        self.extended_state = None
        self.local_pose = None
        self.velocity = None
        self.imu = None
        self.battery = None
        self.arm_joints = None
        self.ee_pose = None
        self.arm_status = {}
        self.servo_status = {}
        self.load_compensation = {}
        self.stamps = {}

        rospy.Subscriber("/mavros/state", State, self._on_mavros_state, queue_size=1)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self._on_extended_state, queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_local_pose, queue_size=1)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._on_velocity, queue_size=1)
        rospy.Subscriber("/mavros/imu/data", Imu, self._on_imu, queue_size=1)
        rospy.Subscriber("/mavros/battery", BatteryState, self._on_battery, queue_size=1)
        rospy.Subscriber("/so101/joint_states", JointState, self._on_arm_joints, queue_size=1)
        rospy.Subscriber("/so101/end_effector_pose", PoseStamped, self._on_ee_pose, queue_size=1)
        rospy.Subscriber("/so101/status", String, self._on_arm_status, queue_size=1)
        rospy.Subscriber("/so101/servo_status", String, self._on_servo_status, queue_size=1)
        rospy.Subscriber("/so101/load_compensation_state", String, self._on_load_compensation, queue_size=1)

        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)

    def _set(self, name, value):
        with self.lock:
            setattr(self, name, value)
            self.stamps[name] = time.time()

    def _on_mavros_state(self, msg):
        self._set("mavros_state", msg)

    def _on_extended_state(self, msg):
        self._set("extended_state", msg)

    def _on_local_pose(self, msg):
        self._set("local_pose", msg)

    def _on_velocity(self, msg):
        self._set("velocity", msg)

    def _on_imu(self, msg):
        self._set("imu", msg)

    def _on_battery(self, msg):
        self._set("battery", msg)

    def _on_arm_joints(self, msg):
        self._set("arm_joints", msg)

    def _on_ee_pose(self, msg):
        self._set("ee_pose", msg)

    def _on_arm_status(self, msg):
        self._set("arm_status", _parse_json(msg.data))

    def _on_servo_status(self, msg):
        self._set("servo_status", _parse_json(msg.data))

    def _on_load_compensation(self, msg):
        self._set("load_compensation", _parse_json(msg.data))

    def _fresh(self, name):
        return _stamp_age(self.stamps.get(name)) is not None and _stamp_age(self.stamps.get(name)) <= self.fresh_timeout_s

    def _pose_payload(self, msg):
        if msg is None:
            return None
        return {
            "frame_id": msg.header.frame_id,
            "position": _vec3(msg.pose.position),
            "orientation": _quat(msg.pose.orientation),
        }

    def _snapshot(self):
        with self.lock:
            state = self.mavros_state
            ext = self.extended_state
            pose = self.local_pose
            vel = self.velocity
            imu = self.imu
            battery = self.battery
            arm_joints = self.arm_joints
            ee_pose = self.ee_pose
            arm_status = dict(self.arm_status)
            servo_status = dict(self.servo_status)
            load_compensation = dict(self.load_compensation)
            stamps = dict(self.stamps)

        arm_error = bool(arm_status.get("estop") or arm_status.get("last_backend_error"))
        mavros_connected = bool(state.connected) if state is not None else False
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "fresh": {
                name: (_stamp_age(stamps.get(name)) is not None and _stamp_age(stamps.get(name)) <= self.fresh_timeout_s)
                for name in [
                    "mavros_state",
                    "local_pose",
                    "velocity",
                    "imu",
                    "battery",
                    "arm_joints",
                    "ee_pose",
                    "arm_status",
                    "servo_status",
                    "load_compensation",
                ]
            },
            "ready": {
                "mavros_connected": mavros_connected,
                "uav_state_fresh": _stamp_age(stamps.get("mavros_state")) is not None
                and _stamp_age(stamps.get("mavros_state")) <= self.fresh_timeout_s,
                "arm_state_fresh": _stamp_age(stamps.get("arm_joints")) is not None
                and _stamp_age(stamps.get("arm_joints")) <= self.fresh_timeout_s,
                "arm_ok": not arm_error,
            },
            "mavros": {
                "connected": mavros_connected,
                "armed": bool(state.armed) if state is not None else False,
                "guided": bool(state.guided) if state is not None else False,
                "mode": state.mode if state is not None else "",
                "system_status": int(state.system_status) if state is not None else None,
                "landed_state": int(ext.landed_state) if ext is not None else None,
                "local_pose": self._pose_payload(pose),
                "velocity_local": {
                    "linear": _vec3(vel.twist.linear),
                    "angular": _vec3(vel.twist.angular),
                }
                if vel is not None
                else None,
                "imu": {
                    "orientation": _quat(imu.orientation),
                    "angular_velocity": _vec3(imu.angular_velocity),
                    "linear_acceleration": _vec3(imu.linear_acceleration),
                }
                if imu is not None
                else None,
                "battery": {
                    "voltage": battery.voltage,
                    "current": battery.current,
                    "percentage": battery.percentage,
                }
                if battery is not None
                else None,
            },
            "so101": {
                "joint_positions": dict(zip(arm_joints.name, arm_joints.position)) if arm_joints is not None else {},
                "end_effector_pose": self._pose_payload(ee_pose),
                "status": arm_status,
                "servo_status": servo_status,
                "load_compensation": load_compensation,
            },
        }
        return payload

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.pub.publish(String(data=json.dumps(self._snapshot(), sort_keys=True)))
            rate.sleep()


def main():
    rospy.init_node("so101_aerial_state_node")
    AerialManipulationStateNode().spin()


if __name__ == "__main__":
    main()

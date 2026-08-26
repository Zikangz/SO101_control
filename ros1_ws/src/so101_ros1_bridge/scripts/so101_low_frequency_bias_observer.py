#!/usr/bin/env python3
"""Observation-only low-frequency joint bias estimator for SO101.

This node deliberately does not command the arm.  It watches measured joint
state, filtered command state, servo telemetry, and the load compensation
observer, then publishes a bounded, slowly-varying command-bias estimate that a
future trajectory layer may consume after offline validation.
"""

import csv
import json
import math
import os
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


DEFAULT_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def _parse_json(text):
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return {}


def _clip(value, lo, hi):
    return max(float(lo), min(float(hi), float(value)))


def _flat_payload_value(payload, path, default=""):
    current = payload
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


class SO101LowFrequencyBiasObserver:
    def __init__(self):
        self.lock = threading.RLock()
        self.joints = list(rospy.get_param("~joints", DEFAULT_JOINTS))
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/so101/joint_states")
        self.commanded_topic = rospy.get_param("~commanded_joint_states_topic", "/so101/commanded_joint_states")
        self.servo_status_topic = rospy.get_param("~servo_status_topic", "/so101/servo_status")
        self.load_compensation_topic = rospy.get_param(
            "~load_compensation_topic", "/so101/load_compensation_state"
        )
        self.out_topic = rospy.get_param("~out_topic", "/so101/trajectory_bias_state")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.fresh_timeout_s = float(rospy.get_param("~fresh_timeout_s", 0.5))
        self.filter_tau_s = max(0.0, float(rospy.get_param("~filter_tau_s", 3.0)))
        self.deadband_rad = max(0.0, float(rospy.get_param("~deadband_rad", 0.002)))
        self.max_abs_bias_rad = max(0.0, float(rospy.get_param("~max_abs_bias_rad", 0.04)))
        self.max_bias_rate_rad_s = max(0.0, float(rospy.get_param("~max_bias_rate_rad_s", 0.004)))
        self.require_load_state_for_valid = bool(rospy.get_param("~require_load_state_for_valid", False))
        self.csv_path = rospy.get_param("~csv_path", "")
        self.append_csv = bool(rospy.get_param("~append_csv", False))

        self.measured = {}
        self.commanded = {}
        self.servo_status = {}
        self.load_compensation = {}
        self.last_measured_time = 0.0
        self.last_commanded_time = 0.0
        self.last_servo_status_time = 0.0
        self.last_load_time = 0.0
        self.filtered_error = {name: 0.0 for name in self.joints}
        self.bias = {name: 0.0 for name in self.joints}
        self.last_update_time = time.time()
        self.started_at = self.last_update_time
        self.csv_handle = None
        self.csv_writer = None

        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)
        rospy.Subscriber(self.joint_states_topic, JointState, self._on_joint_state, queue_size=20)
        rospy.Subscriber(self.commanded_topic, JointState, self._on_commanded_joint_state, queue_size=20)
        rospy.Subscriber(self.servo_status_topic, String, self._on_servo_status, queue_size=10)
        rospy.Subscriber(self.load_compensation_topic, String, self._on_load_compensation, queue_size=10)

        if self.csv_path:
            directory = os.path.dirname(os.path.abspath(self.csv_path))
            if directory:
                os.makedirs(directory, exist_ok=True)
            exists = os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0
            self.csv_handle = open(self.csv_path, "a" if self.append_csv else "w", newline="")
            self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=self._csv_fields())
            if not (self.append_csv and exists):
                self.csv_writer.writeheader()

    def _stamp_or_now(self, msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        return stamp if stamp > 0.0 else time.time()

    def _on_joint_state(self, msg):
        now = self._stamp_or_now(msg)
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.measured[name] = float(position)
            self.last_measured_time = now

    def _on_commanded_joint_state(self, msg):
        now = self._stamp_or_now(msg)
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.commanded[name] = float(position)
            self.last_commanded_time = now

    def _on_servo_status(self, msg):
        with self.lock:
            payload = _parse_json(msg.data)
            self.servo_status = payload.get("joints", payload)
            self.last_servo_status_time = time.time()

    def _on_load_compensation(self, msg):
        with self.lock:
            self.load_compensation = _parse_json(msg.data)
            self.last_load_time = time.time()

    def _fresh(self, now, stamp):
        return stamp > 0.0 and (now - stamp) <= self.fresh_timeout_s

    def _csv_fields(self):
        fields = [
            "stamp",
            "elapsed",
            "valid",
            "measured_fresh",
            "commanded_fresh",
            "servo_status_fresh",
            "load_state_fresh",
            "load_static_bias_allowed",
            "load_upright_tilt_deg",
            "load_linear_acceleration_norm_m_s2",
        ]
        for name in self.joints:
            fields.extend(
                [
                    name,
                    "commanded_" + name,
                    "tracking_error_" + name,
                    "filtered_tracking_error_" + name,
                    "suggested_command_bias_" + name,
                    "servo_" + name + "_current_ma",
                    "servo_" + name + "_load_raw",
                    "servo_" + name + "_moving",
                    "servo_" + name + "_velocity_raw",
                ]
            )
        return fields

    def _write_csv(self, payload):
        if not self.csv_writer:
            return
        row = {
            "stamp": payload["stamp"],
            "elapsed": payload["elapsed"],
            "valid": payload["valid"],
            "measured_fresh": payload["fresh"]["measured"],
            "commanded_fresh": payload["fresh"]["commanded"],
            "servo_status_fresh": payload["fresh"]["servo_status"],
            "load_state_fresh": payload["fresh"]["load_compensation"],
            "load_static_bias_allowed": payload["load_compensation"].get("static_bias_allowed", ""),
            "load_upright_tilt_deg": payload["load_compensation"].get("upright_tilt_deg", ""),
            "load_linear_acceleration_norm_m_s2": payload["load_compensation"].get(
                "linear_acceleration_norm_m_s2", ""
            ),
        }
        for name in self.joints:
            servo = self.servo_status.get(name, {}) if isinstance(self.servo_status, dict) else {}
            row[name] = payload["measured_rad"].get(name, "")
            row["commanded_" + name] = payload["commanded_rad"].get(name, "")
            row["tracking_error_" + name] = payload["tracking_error_rad"].get(name, "")
            row["filtered_tracking_error_" + name] = payload["filtered_tracking_error_rad"].get(name, "")
            row["suggested_command_bias_" + name] = payload["suggested_command_bias_rad"].get(name, "")
            row["servo_" + name + "_current_ma"] = servo.get("current_ma", "")
            row["servo_" + name + "_load_raw"] = servo.get("load_raw", "")
            row["servo_" + name + "_moving"] = servo.get("moving", "")
            row["servo_" + name + "_velocity_raw"] = servo.get("velocity_raw", "")
        self.csv_writer.writerow(row)
        self.csv_handle.flush()

    def _update(self):
        now = time.time()
        with self.lock:
            dt = max(0.0, min(1.0, now - self.last_update_time))
            self.last_update_time = now

            measured_fresh = self._fresh(now, self.last_measured_time)
            commanded_fresh = self._fresh(now, self.last_commanded_time)
            servo_fresh = self._fresh(now, self.last_servo_status_time)
            load_fresh = self._fresh(now, self.last_load_time)
            load_allowed = bool(self.load_compensation.get("static_bias_allowed", False)) and load_fresh
            valid = measured_fresh and commanded_fresh
            if self.require_load_state_for_valid:
                valid = valid and load_allowed

            alpha = 1.0 if self.filter_tau_s <= 0.0 else dt / (self.filter_tau_s + dt)
            measured = {name: float(self.measured.get(name, 0.0)) for name in self.joints}
            commanded = {name: float(self.commanded.get(name, 0.0)) for name in self.joints}
            raw_error = {}
            for name in self.joints:
                error = measured[name] - commanded[name]
                raw_error[name] = error
                if valid:
                    self.filtered_error[name] += alpha * (error - self.filtered_error[name])
                    target_bias = 0.0 if abs(self.filtered_error[name]) < self.deadband_rad else -self.filtered_error[name]
                    target_bias = _clip(target_bias, -self.max_abs_bias_rad, self.max_abs_bias_rad)
                    if self.max_bias_rate_rad_s > 0.0:
                        max_delta = self.max_bias_rate_rad_s * dt
                        target_bias = self.bias[name] + _clip(target_bias - self.bias[name], -max_delta, max_delta)
                    self.bias[name] = _clip(target_bias, -self.max_abs_bias_rad, self.max_abs_bias_rad)

            load_summary = {
                "fresh": load_fresh,
                "static_bias_allowed": bool(self.load_compensation.get("static_bias_allowed", False)),
                "upright_tilt_deg": self.load_compensation.get("upright_tilt_deg", ""),
                "linear_acceleration_norm_m_s2": self.load_compensation.get(
                    "linear_acceleration_norm_m_s2", ""
                ),
                "effective_gravity_arm_m_s2": self.load_compensation.get("effective_gravity_arm_m_s2", {}),
            }
            payload = {
                "stamp": now,
                "elapsed": now - self.started_at,
                "valid": bool(valid),
                "mode": "observe_only_no_command_output",
                "message": "publishing bounded low-frequency bias estimate only; no arm command is applied",
                "fresh": {
                    "measured": measured_fresh,
                    "commanded": commanded_fresh,
                    "servo_status": servo_fresh,
                    "load_compensation": load_fresh,
                },
                "gates": {
                    "require_load_state_for_valid": self.require_load_state_for_valid,
                    "max_abs_bias_rad": self.max_abs_bias_rad,
                    "max_bias_rate_rad_s": self.max_bias_rate_rad_s,
                    "deadband_rad": self.deadband_rad,
                    "filter_tau_s": self.filter_tau_s,
                },
                "measured_rad": measured,
                "commanded_rad": commanded,
                "tracking_error_rad": raw_error,
                "filtered_tracking_error_rad": dict(self.filtered_error),
                "suggested_command_bias_rad": dict(self.bias),
                "load_compensation": load_summary,
            }
            self._write_csv(payload)
            return payload

    def spin(self):
        rate = rospy.Rate(max(1.0, self.publish_rate_hz))
        while not rospy.is_shutdown():
            payload = self._update()
            self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))
            rate.sleep()

    def close(self):
        if self.csv_handle:
            self.csv_handle.close()
            self.csv_handle = None


def main():
    rospy.init_node("so101_low_frequency_bias_observer")
    node = SO101LowFrequencyBiasObserver()
    try:
        node.spin()
    finally:
        node.close()


if __name__ == "__main__":
    main()

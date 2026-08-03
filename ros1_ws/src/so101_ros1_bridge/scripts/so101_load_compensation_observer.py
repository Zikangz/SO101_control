#!/usr/bin/env python3
"""Observe gravity and inertial load in the SO101 arm base frame.

This node is deliberately observation-only. STS3215 is used here in position
mode, so it cannot accept a signed commanded torque for true model-based
gravity compensation. The published state is the input needed by a future
identified joint-bias compensator or a torque-capable actuator interface.
"""

import json
import math
import threading
import time

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_matrix, quaternion_matrix


GRAVITY_M_S2 = 9.80665


def _dot(left, right):
    return sum(float(a) * float(b) for a, b in zip(left, right))


def _norm(vector):
    return math.sqrt(_dot(vector, vector))


def _transpose_matvec(matrix, vector):
    return [sum(float(matrix[row][column]) * float(vector[row]) for row in range(3)) for column in range(3)]


def _matvec(matrix, vector):
    return [sum(float(matrix[row][column]) * float(vector[column]) for column in range(3)) for row in range(3)]


def _payload(vector):
    return {"x": float(vector[0]), "y": float(vector[1]), "z": float(vector[2])}


class SO101LoadCompensationObserver:
    def __init__(self):
        self.lock = threading.RLock()
        self.imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data")
        self.out_topic = rospy.get_param("~out_topic", "/so101/load_compensation_state")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.fresh_timeout_s = float(rospy.get_param("~fresh_timeout_s", 0.25))
        self.accel_filter_tau_s = float(rospy.get_param("~accel_filter_tau_s", 0.25))
        self.orientation_is_body_to_world = bool(rospy.get_param("~orientation_is_body_to_world", True))
        self.imu_acceleration_is_specific_force = bool(
            rospy.get_param("~imu_acceleration_is_specific_force", True)
        )
        self.max_static_bias_tilt_deg = float(rospy.get_param("~max_static_bias_tilt_deg", 10.0))
        self.max_static_bias_linear_accel_m_s2 = float(
            rospy.get_param("~max_static_bias_linear_accel_m_s2", 0.5)
        )

        rpy = rospy.get_param("~uav_to_arm_rpy", None)
        if not isinstance(rpy, (list, tuple)) or len(rpy) != 3:
            rpy = [
                float(rospy.get_param("~uav_to_arm_roll_rad", 0.0)),
                float(rospy.get_param("~uav_to_arm_pitch_rad", 0.0)),
                float(rospy.get_param("~uav_to_arm_yaw_rad", 0.0)),
            ]
        self.uav_to_arm_rpy = [float(value) for value in rpy]
        # R_BA maps an arm-base vector A into the UAV body frame B.
        self.rotation_body_from_arm = euler_matrix(*self.uav_to_arm_rpy)[:3, :3]

        self.filtered_linear_accel_body = None
        self.last_imu_time = None
        self.last_receive_time = None
        self.state = {"valid": False, "message": "waiting for MAVROS IMU"}

        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=10)

    def _on_imu(self, msg):
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        if _norm(quaternion) < 1e-6:
            rospy.logwarn_throttle(2.0, "SO101 load observer ignored IMU with zero orientation quaternion")
            return

        rotation_world_from_body = quaternion_matrix(quaternion)[:3, :3]
        if not self.orientation_is_body_to_world:
            rotation_world_from_body = rotation_world_from_body.T
        gravity_world = [0.0, 0.0, -GRAVITY_M_S2]
        gravity_body = _transpose_matvec(rotation_world_from_body, gravity_world)
        raw_accel_body = [
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        ]
        if self.imu_acceleration_is_specific_force:
            # sensor_msgs/Imu acceleration is normally specific force. Convert
            # it to vehicle linear acceleration before forming g - a.
            linear_accel_body = [raw_accel_body[idx] + gravity_body[idx] for idx in range(3)]
        else:
            linear_accel_body = raw_accel_body

        stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        now = time.time()
        if stamp <= 0.0:
            stamp = now
        with self.lock:
            if self.last_imu_time is None or self.filtered_linear_accel_body is None:
                filtered = list(linear_accel_body)
            else:
                dt = max(0.0, min(1.0, stamp - self.last_imu_time))
                alpha = 1.0 if self.accel_filter_tau_s <= 0.0 else dt / (self.accel_filter_tau_s + dt)
                filtered = [
                    self.filtered_linear_accel_body[idx]
                    + alpha * (linear_accel_body[idx] - self.filtered_linear_accel_body[idx])
                    for idx in range(3)
                ]
            self.filtered_linear_accel_body = filtered
            self.last_imu_time = stamp
            self.last_receive_time = now

            # In the accelerating arm-base frame, the links see g_eff = g - a.
            effective_gravity_body = [gravity_body[idx] - filtered[idx] for idx in range(3)]
            gravity_arm = _transpose_matvec(self.rotation_body_from_arm, gravity_body)
            linear_accel_arm = _transpose_matvec(self.rotation_body_from_arm, filtered)
            effective_gravity_arm = _transpose_matvec(self.rotation_body_from_arm, effective_gravity_body)
            upright_cosine = max(-1.0, min(1.0, -gravity_body[2] / GRAVITY_M_S2))
            tilt_deg = math.degrees(math.acos(upright_cosine))
            linear_accel_norm = _norm(filtered)
            static_bias_allowed = (
                tilt_deg <= self.max_static_bias_tilt_deg
                and linear_accel_norm <= self.max_static_bias_linear_accel_m_s2
            )
            self.state = {
                "valid": True,
                "message": "gravity/inertial load observed; no servo command is applied",
                "imu_frame_id": msg.header.frame_id,
                "orientation_is_body_to_world": self.orientation_is_body_to_world,
                "imu_acceleration_is_specific_force": self.imu_acceleration_is_specific_force,
                "uav_to_arm_rpy_rad": self.uav_to_arm_rpy,
                "gravity_body_m_s2": _payload(gravity_body),
                "gravity_arm_m_s2": _payload(gravity_arm),
                "linear_acceleration_body_m_s2": _payload(filtered),
                "linear_acceleration_arm_m_s2": _payload(linear_accel_arm),
                "effective_gravity_arm_m_s2": _payload(effective_gravity_arm),
                "upright_tilt_deg": tilt_deg,
                "linear_acceleration_norm_m_s2": linear_accel_norm,
                "static_bias_allowed": static_bias_allowed,
                "static_bias_gate": {
                    "max_tilt_deg": self.max_static_bias_tilt_deg,
                    "max_linear_acceleration_m_s2": self.max_static_bias_linear_accel_m_s2,
                },
                "stamp": stamp,
            }

    def spin(self):
        rate = rospy.Rate(max(1.0, self.publish_rate_hz))
        while not rospy.is_shutdown():
            with self.lock:
                payload = dict(self.state)
                age = None if self.last_receive_time is None else max(0.0, time.time() - self.last_receive_time)
            payload["imu_age_s"] = age
            payload["fresh"] = age is not None and age <= self.fresh_timeout_s
            if not payload["fresh"]:
                payload["static_bias_allowed"] = False
                payload["message"] = "MAVROS IMU is stale; static compensation must be disabled"
            self.pub.publish(String(data=json.dumps(payload, sort_keys=True)))
            rate.sleep()


def main():
    rospy.init_node("so101_load_compensation_observer")
    SO101LoadCompensationObserver().spin()


if __name__ == "__main__":
    main()

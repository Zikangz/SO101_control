#!/usr/bin/env python3
import json
import os
import sys
import threading
import time
import traceback

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.control import JointSafetyFilter
from so101_ros1_bridge.hardware import FeetechSO101Backend, MockSO101Backend, MujocoSO101Backend


class SO101DriverNode:
    def __init__(self):
        self.ns = "~"
        self.joint_order = rospy.get_param("~joint_order")
        self.active_joints = rospy.get_param("~active_joints", self.joint_order)
        self.locked_joints = rospy.get_param("~locked_joints", {})
        self.home_positions = rospy.get_param("~home_positions")
        self.limits = rospy.get_param("~limits")
        self.max_velocity = rospy.get_param("~max_velocity")
        self.command_rate_hz = float(rospy.get_param("~command_rate_hz", 50.0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.feedback_read_rate_hz = float(rospy.get_param("~feedback_read_rate_hz", self.publish_rate_hz))
        self.command_timeout_s = float(rospy.get_param("~command_timeout_s", 0.5))
        self.backend_name = rospy.get_param("~backend", "mock")
        self.mujoco_model_path = rospy.get_param("~mujoco_model_path", "")
        self.mujoco_substeps = int(rospy.get_param("~mujoco_substeps", 10))
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baudrate = int(rospy.get_param("~baudrate", 1000000))
        self.motor_ids = rospy.get_param("~motor_ids", {})
        self.hardware_calibration = rospy.get_param("~hardware_calibration", {})
        self.disable_torque_on_relax = bool(rospy.get_param("~disable_torque_on_relax", True))
        self.trajectory_hold_margin_s = float(rospy.get_param("~trajectory_hold_margin_s", 0.75))
        self.stale_action = rospy.get_param("~stale_action", "hold_target")
        self.servo_pid = rospy.get_param("~servo_pid", {})
        self.servo_speed = int(rospy.get_param("~servo_speed", 800))
        self.servo_acceleration = int(rospy.get_param("~servo_acceleration", 50))
        self.servo_maximum_acceleration = rospy.get_param("~servo_maximum_acceleration", None)
        if self.servo_maximum_acceleration in ("", "none", "None"):
            self.servo_maximum_acceleration = None
        elif self.servo_maximum_acceleration is not None:
            self.servo_maximum_acceleration = int(self.servo_maximum_acceleration)
        self.telemetry_rate_hz = float(rospy.get_param("~telemetry_rate_hz", 2.0))
        self.skip_duplicate_writes = bool(rospy.get_param("~skip_duplicate_writes", True))
        self.command_lpf_alpha = float(rospy.get_param("~command_lpf_alpha", 1.0))
        self.command_deadband_rad = float(rospy.get_param("~command_deadband_rad", 0.0))
        self.trajectory_interpolation = rospy.get_param("~trajectory_interpolation", "cubic")
        self.feedback_position_assist_gain = max(0.0, float(rospy.get_param("~feedback_position_assist_gain", 0.0)))
        self.feedback_position_assist_integral_gain = max(
            0.0, float(rospy.get_param("~feedback_position_assist_integral_gain", 0.0))
        )
        self.feedback_position_assist_max_offset_rad = max(
            0.0, float(rospy.get_param("~feedback_position_assist_max_offset_rad", 0.04))
        )
        self.feedback_position_assist_limit_margin_rad = max(
            0.0, float(rospy.get_param("~feedback_position_assist_limit_margin_rad", 0.03))
        )
        self.feedback_position_assist_alpha = max(
            0.0, min(1.0, float(rospy.get_param("~feedback_position_assist_alpha", 0.15)))
        )
        self.feedback_position_assist_max_feedback_age_s = max(
            0.0, float(rospy.get_param("~feedback_position_assist_max_feedback_age_s", 0.12))
        )
        self.feedback_position_assist_trajectory_only = bool(
            rospy.get_param("~feedback_position_assist_trajectory_only", True)
        )
        self.command_lpf_alpha = max(0.0, min(1.0, self.command_lpf_alpha))
        if self.stale_action not in ("hold_target", "freeze_current", "estop"):
            raise ValueError("Unsupported stale_action: %s" % self.stale_action)

        self.lock = threading.RLock()
        self.estop = False
        self.relaxed = False
        self.last_loop_time = time.time()
        self.command_hold_until = 0.0
        self.last_backend_error = ""
        self.last_telemetry_error = ""
        self.last_feedback_time = 0.0
        self.current_positions = dict(self.home_positions)
        self.output_positions = dict(self.home_positions)
        self.feedback_position_assist_offsets = {name: 0.0 for name in self.joint_order}
        self.feedback_position_assist_integral_offsets = {name: 0.0 for name in self.joint_order}
        self.feedback_position_assist_active = False

        self.filter = JointSafetyFilter(
            self.joint_order,
            self.active_joints,
            self.locked_joints,
            self.limits,
            self.max_velocity,
            self.home_positions,
            trajectory_interpolation=self.trajectory_interpolation,
        )
        self.backend = self._make_backend()

        self.joint_pub = rospy.Publisher("/so101/joint_states", JointState, queue_size=10)
        self.status_pub = rospy.Publisher("/so101/status", String, queue_size=10)
        self.servo_status_pub = rospy.Publisher("/so101/servo_status", String, queue_size=10)
        self.commanded_joint_pub = rospy.Publisher("/so101/commanded_joint_states", JointState, queue_size=10)
        self.target_joint_pub = rospy.Publisher("/so101/target_joint_states", JointState, queue_size=10)

        rospy.Subscriber("/so101/command_joint_positions", JointTrajectory, self._on_joint_trajectory, queue_size=1)
        rospy.Subscriber("/so101/command_joint_servo", JointState, self._on_joint_servo, queue_size=1)
        rospy.Subscriber("/so101/command_joint_deltas", Float64MultiArray, self._on_joint_deltas, queue_size=1)
        rospy.Subscriber("/so101/gripper_command", Float32, self._on_gripper_command, queue_size=1)
        rospy.Subscriber("/so101/estop", Bool, self._on_estop, queue_size=1)
        rospy.Subscriber("/so101/home", Empty, self._on_home, queue_size=1)
        rospy.Subscriber("/so101/freeze", Empty, self._on_freeze, queue_size=1)
        rospy.Subscriber("/so101/relax", Empty, self._on_relax, queue_size=1)

    def _hold_target_for(self, duration_s):
        duration_s = max(0.0, float(duration_s))
        self.command_hold_until = time.time() + max(self.command_timeout_s, duration_s)

    def _command_is_stale(self, now):
        if now <= self.command_hold_until:
            return False
        return self.filter.is_command_stale(self.command_timeout_s)

    def _make_backend(self):
        if self.backend_name == "mock":
            return MockSO101Backend(self.joint_order, self.home_positions, self.max_velocity)
        if self.backend_name == "mujoco":
            return MujocoSO101Backend(
                self.joint_order,
                self.home_positions,
                model_path=self.mujoco_model_path,
                substeps=self.mujoco_substeps,
            )
        if self.backend_name == "feetech":
            return FeetechSO101Backend(
                self.port,
                self.baudrate,
                self.joint_order,
                self.motor_ids,
                self.hardware_calibration,
                self.disable_torque_on_relax,
                servo_pid=self.servo_pid,
                servo_speed=self.servo_speed,
                servo_acceleration=self.servo_acceleration,
                servo_maximum_acceleration=self.servo_maximum_acceleration,
                skip_duplicate_writes=self.skip_duplicate_writes,
            )
        raise ValueError("Unsupported backend: %s" % self.backend_name)

    def connect(self):
        rospy.loginfo("Connecting SO101 backend '%s'", self.backend_name)
        self.backend.connect()
        self.current_positions = self.backend.read_positions()
        self.last_feedback_time = time.time()
        self.filter.freeze(self.current_positions)
        self.output_positions = dict(self.current_positions)
        self._reset_feedback_position_assist()
        rospy.loginfo("SO101 bridge connected with joints: %s", ", ".join(self.joint_order))

    def _reset_output_positions(self, positions):
        self.output_positions = {
            name: float(positions.get(name, self.output_positions.get(name, 0.0)))
            for name in self.joint_order
        }

    def _reset_feedback_position_assist(self):
        self.feedback_position_assist_offsets = {name: 0.0 for name in self.joint_order}
        self.feedback_position_assist_integral_offsets = {name: 0.0 for name in self.joint_order}
        self.feedback_position_assist_active = False

    def _apply_feedback_position_assist(self, desired, now, dt):
        """Apply a bounded outer PI correction around the servo's internal PID.

        STS3215 remains in its native position-control mode. This optional
        outer loop uses only its measured position. The integral term is
        disabled by default and has anti-windup plus a separate hard-limit
        margin because it is intended to cancel slow static deflection, not
        to force a joint into a mechanical stop.
        """
        dt = max(0.0, min(0.1, float(dt)))
        feedback_fresh = (now - self.last_feedback_time) <= self.feedback_position_assist_max_feedback_age_s
        active = self.feedback_position_assist_gain > 0.0 and feedback_fresh
        if self.feedback_position_assist_trajectory_only and not self.filter.trajectory_active():
            active = False

        assisted = dict(desired)
        alpha = self.feedback_position_assist_alpha
        for name in self.joint_order:
            previous = float(self.feedback_position_assist_offsets.get(name, 0.0))
            integral = float(self.feedback_position_assist_integral_offsets.get(name, 0.0))
            correction_target = 0.0
            if active and name in self.active_joints and name not in self.locked_joints and name != "gripper":
                error = float(desired.get(name, 0.0)) - float(self.current_positions.get(name, 0.0))
                proportional = self.feedback_position_assist_gain * error
                candidate_integral = integral + self.feedback_position_assist_integral_gain * error * dt
                raw_correction = proportional + candidate_integral
                correction_target = max(
                    -self.feedback_position_assist_max_offset_rad,
                    min(self.feedback_position_assist_max_offset_rad, raw_correction),
                )
                # Do not accumulate further when already saturated in the
                # same direction as the current tracking error.
                if (
                    abs(raw_correction) <= self.feedback_position_assist_max_offset_rad
                    or raw_correction * error < 0.0
                ):
                    integral = candidate_integral
            else:
                integral = integral + alpha * (0.0 - integral)
            self.feedback_position_assist_integral_offsets[name] = integral
            offset = previous + alpha * (correction_target - previous)
            self.feedback_position_assist_offsets[name] = offset

            target = float(desired.get(name, self.output_positions.get(name, 0.0)))
            lo, hi = self.limits.get(name, (-float("inf"), float("inf")))
            if name in self.active_joints and name not in self.locked_joints and name != "gripper":
                lo = min(float(hi), float(lo) + self.feedback_position_assist_limit_margin_rad)
                hi = max(float(lo), float(hi) - self.feedback_position_assist_limit_margin_rad)
            assisted[name] = max(float(lo), min(float(hi), target + offset))

        self.feedback_position_assist_active = bool(active)
        return assisted

    def _smooth_command(self, desired):
        if not self.output_positions:
            self._reset_output_positions(desired)
        output = {}
        alpha = self.command_lpf_alpha
        for name in self.joint_order:
            target = float(desired.get(name, self.output_positions.get(name, 0.0)))
            previous = float(self.output_positions.get(name, target))
            if alpha >= 0.999:
                value = target
            elif alpha <= 0.0:
                value = previous
            else:
                value = previous + alpha * (target - previous)
            deadband = 0.0 if name == "gripper" else self.command_deadband_rad
            if deadband > 0.0 and abs(value - previous) < deadband and abs(target - previous) < deadband:
                value = previous
            output[name] = value
        self.output_positions = output
        return dict(output)

    def _on_joint_trajectory(self, msg):
        if not msg.points:
            return
        names = list(msg.joint_names)
        with self.lock:
            if self.estop:
                return
            try:
                if len(msg.points) > 1:
                    timed_positions = []
                    for point in msg.points:
                        values = list(point.positions)
                        if len(names) != len(values):
                            rospy.logwarn("Ignoring trajectory command with mismatched names/positions")
                            return
                        timed_positions.append((float(point.time_from_start.to_sec()), dict(zip(names, values))))
                    self.filter.set_timed_trajectory(names, timed_positions)
                    duration_s = max(item[0] for item in timed_positions)
                else:
                    values = list(msg.points[0].positions)
                    if len(names) != len(values):
                        rospy.logwarn("Ignoring trajectory command with mismatched names/positions")
                        return
                    duration_s = float(msg.points[0].time_from_start.to_sec())
                    self.filter.set_target_positions(dict(zip(names, values)), duration_s=duration_s)
                self._hold_target_for(duration_s + self.trajectory_hold_margin_s)
                self.relaxed = False
            except ValueError as exc:
                rospy.logwarn("Rejected joint position command: %s", exc)

    def _on_joint_servo(self, msg):
        """High-rate streaming servo setpoints from so101_servo_node.

        Unlike single-point JointTrajectory messages, these update the safety
        filter's target without restarting the minimum-jerk profile, so the
        driver velocity-limits toward a continuously-moving target instead of
        re-easing on every frame (see JointSafetyFilter.set_servo_target).
        """
        if not msg.name or not msg.position:
            return
        if len(msg.name) != len(msg.position):
            rospy.logwarn_throttle(1.0, "Ignoring servo command with mismatched names/positions")
            return
        with self.lock:
            if self.estop:
                return
            try:
                self.filter.set_servo_target(dict(zip(msg.name, msg.position)))
                self._hold_target_for(self.command_timeout_s)
                self.relaxed = False
            except ValueError as exc:
                rospy.logwarn_throttle(1.0, "Rejected servo command: %s", exc)

    def _on_joint_deltas(self, msg):
        values = list(msg.data)
        if len(values) == len(self.joint_order):
            names = self.joint_order
        elif len(values) == len(self.active_joints):
            names = list(self.active_joints)
        else:
            rospy.logwarn(
                "Ignoring delta command length %d; expected %d full joints or %d active joints",
                len(values),
                len(self.joint_order),
                len(self.active_joints),
            )
            return
        with self.lock:
            if self.estop:
                return
            try:
                self.filter.apply_deltas(dict(zip(names, values)), duration_s=self.command_timeout_s)
                self._hold_target_for(self.command_timeout_s)
                self.relaxed = False
            except ValueError as exc:
                rospy.logwarn("Rejected joint delta command: %s", exc)

    def _on_gripper_command(self, msg):
        with self.lock:
            if self.estop:
                return
            self.filter.set_target_positions({"gripper": float(msg.data)}, duration_s=self.command_timeout_s)
            self._hold_target_for(self.command_timeout_s)
            self.relaxed = False

    def _on_estop(self, msg):
        with self.lock:
            self.estop = bool(msg.data)
            if self.estop:
                rospy.logwarn("SO101 emergency stop enabled; freezing arm")
                self.filter.freeze(self.current_positions)
                self._reset_output_positions(self.current_positions)
                self._reset_feedback_position_assist()
                self.command_hold_until = 0.0
            else:
                rospy.logwarn("SO101 emergency stop cleared")

    def _on_home(self, _msg):
        with self.lock:
            if self.estop:
                rospy.logwarn("Ignoring home while estop is active")
                return
            self.filter.home(duration_s=3.0)
            self._hold_target_for(3.0)
            self.relaxed = False

    def _on_freeze(self, _msg):
        with self.lock:
            self.filter.freeze(self.current_positions)
            self._reset_output_positions(self.current_positions)
            self._reset_feedback_position_assist()
            self.command_hold_until = 0.0

    def _on_relax(self, _msg):
        with self.lock:
            self.filter.freeze(self.current_positions)
            self._reset_output_positions(self.current_positions)
            self._reset_feedback_position_assist()
            self.command_hold_until = 0.0
            self.backend.relax()
            self.relaxed = True

    def _publish_joint_state(self, positions, publisher=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(self.joint_order)
        msg.position = [float(positions.get(name, 0.0)) for name in self.joint_order]
        msg.velocity = []
        msg.effort = []
        (publisher or self.joint_pub).publish(msg)

    def _publish_status(self, stale=None):
        now = time.time()
        if stale is None:
            stale = self._command_is_stale(now)
        status = {
            "backend": self.backend_name,
            "mujoco_model_path": self.mujoco_model_path if self.backend_name == "mujoco" else "",
            "mujoco_substeps": self.mujoco_substeps if self.backend_name == "mujoco" else "",
            "estop": self.estop,
            "relaxed": self.relaxed,
            "stale": bool(stale),
            "stale_action": self.stale_action,
            "holding_last_target": bool(stale and self.stale_action == "hold_target"),
            "command_hold_remaining_s": max(0.0, self.command_hold_until - now),
            "servo_pid": self.servo_pid,
            "servo_speed": self.servo_speed,
            "servo_acceleration": self.servo_acceleration,
            "servo_maximum_acceleration": self.servo_maximum_acceleration,
            "command_rate_hz": self.command_rate_hz,
            "publish_rate_hz": self.publish_rate_hz,
            "feedback_read_rate_hz": self.feedback_read_rate_hz,
            "command_lpf_alpha": self.command_lpf_alpha,
            "command_deadband_rad": self.command_deadband_rad,
            "trajectory_interpolation": self.trajectory_interpolation,
            "feedback_position_assist": {
                "enabled": self.feedback_position_assist_gain > 0.0,
                "active": self.feedback_position_assist_active,
                "gain": self.feedback_position_assist_gain,
                "integral_gain": self.feedback_position_assist_integral_gain,
                "max_offset_rad": self.feedback_position_assist_max_offset_rad,
                "limit_margin_rad": self.feedback_position_assist_limit_margin_rad,
                "alpha": self.feedback_position_assist_alpha,
                "feedback_age_s": max(0.0, now - self.last_feedback_time),
                "offsets_rad": self.feedback_position_assist_offsets,
                "integral_offsets_rad": self.feedback_position_assist_integral_offsets,
            },
            "skip_duplicate_writes": self.skip_duplicate_writes,
            "trajectory_active": self.filter.trajectory_active(),
            "desired_positions": self.filter.commanded,
            "commanded_positions": self.output_positions,
            "target_positions": self.filter.target,
            "tracking_error_rad": {
                name: float(self.current_positions.get(name, 0.0)) - float(self.output_positions.get(name, 0.0))
                for name in self.joint_order
            },
            "active_joints": list(self.active_joints),
            "locked_joints": self.locked_joints,
            "last_backend_error": self.last_backend_error,
            "last_telemetry_error": self.last_telemetry_error,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def _publish_servo_status(self):
        try:
            diagnostics = self.backend.read_diagnostics()
            self.last_telemetry_error = ""
            payload = {
                "stamp": rospy.Time.now().to_sec(),
                "backend": self.backend_name,
                "joints": diagnostics,
            }
            self.servo_status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        except Exception as exc:
            self.last_telemetry_error = str(exc)
            rospy.logwarn_throttle(2.0, "SO101 telemetry error: %s", exc)

    def spin(self):
        rate = rospy.Rate(self.command_rate_hz)
        publish_period = 1.0 / max(1.0, self.publish_rate_hz)
        feedback_period = 1.0 / max(1.0, self.feedback_read_rate_hz) if self.feedback_read_rate_hz > 0 else 0.0
        telemetry_period = 1.0 / max(0.1, self.telemetry_rate_hz) if self.telemetry_rate_hz > 0 else 0.0
        last_publish = 0.0
        last_feedback = 0.0
        last_telemetry = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            dt = now - self.last_loop_time
            self.last_loop_time = now
            with self.lock:
                stale = self._command_is_stale(now)
                try:
                    if feedback_period > 0.0 and now - last_feedback >= feedback_period:
                        self.current_positions = self.backend.read_positions()
                        last_feedback = now
                        self.last_feedback_time = now
                        self.last_backend_error = ""
                    stale = self._command_is_stale(now)
                    if stale and self.stale_action == "freeze_current":
                        self.filter.freeze(self.current_positions)
                        self._reset_output_positions(self.current_positions)
                        self._reset_feedback_position_assist()
                    elif stale and self.stale_action == "estop":
                        self.estop = True
                        self.filter.freeze(self.current_positions)
                        self._reset_output_positions(self.current_positions)
                        self._reset_feedback_position_assist()
                        rospy.logwarn_throttle(2.0, "SO101 command stale; estop enabled")
                    if not self.estop and not self.relaxed:
                        desired_command = self.filter.step(dt)
                        assisted_command = self._apply_feedback_position_assist(desired_command, now, dt)
                        output_command = self._smooth_command(assisted_command)
                        self.backend.write_positions(output_command)
                except Exception as exc:
                    self.last_backend_error = str(exc)
                    rospy.logerr_throttle(1.0, "SO101 backend error: %s", exc)
                    self.estop = True
                    self.filter.freeze(self.current_positions)
                    self._reset_output_positions(self.current_positions)
                    self._reset_feedback_position_assist()

                if now - last_publish >= publish_period:
                    self._publish_joint_state(self.current_positions)
                    self._publish_joint_state(self.output_positions, self.commanded_joint_pub)
                    self._publish_joint_state(self.filter.target, self.target_joint_pub)
                    self._publish_status(stale=stale)
                    last_publish = now
                if telemetry_period > 0 and now - last_telemetry >= telemetry_period:
                    self._publish_servo_status()
                    last_telemetry = now
            rate.sleep()


def main():
    rospy.init_node("so101_driver_node")
    node = None
    try:
        node = SO101DriverNode()
        node.connect()
        node.spin()
    except Exception:
        rospy.logfatal("SO101 driver failed:\n%s", traceback.format_exc())
        raise
    finally:
        if node is not None:
            try:
                node.backend.close()
            except Exception:
                rospy.logwarn("Ignoring backend close failure:\n%s", traceback.format_exc())


if __name__ == "__main__":
    main()

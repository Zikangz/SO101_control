#!/usr/bin/env python3
"""Flight-aware velocity gate for the SO101 Cartesian servo.

Upper layers should publish raw end-effector velocity commands to
``/so101/ee_velocity_cmd_raw``.  This node clamps, rate-limits and optionally
derates those commands from UAV and arm state before forwarding them to
``/so101/ee_velocity_cmd`` for ``so101_servo_node.py``.

The node is intentionally feed-forward: it never closes a high-bandwidth loop
on the low-rate joint feedback.  It only gates unsafe or stale commands and
keeps the command velocity continuous.
"""

import json
import math
import threading
import time
import traceback

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


def _param_bool(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _param_float(name, default):
    value = rospy.get_param(name, default)
    if value in (None, "", "none", "None"):
        return float(default)
    return float(value)


def _parse_json(data):
    try:
        return json.loads(data)
    except Exception:
        return {}


def _as_dict(value):
    return value if isinstance(value, dict) else {}


def _safe_float(value, default=0.0):
    if value in (None, "", "none", "None"):
        return None if default is None else float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None if default is None else float(default)


def _vec_norm(values):
    return math.sqrt(sum(_safe_float(v) * _safe_float(v) for v in values))


def _dict_vec_norm(payload):
    payload = _as_dict(payload)
    return _vec_norm([payload.get("x", 0.0), payload.get("y", 0.0), payload.get("z", 0.0)])


def _scale_above(value, nominal, maximum, minimum):
    value = abs(_safe_float(value))
    if _safe_float(nominal) <= 0.0 and _safe_float(maximum) <= 0.0:
        return 1.0
    nominal = max(0.0, _safe_float(nominal))
    maximum = max(nominal + 1e-9, _safe_float(maximum, nominal + 1.0))
    minimum = max(0.0, min(1.0, _safe_float(minimum, 1.0)))
    if value <= nominal:
        return 1.0
    if value >= maximum:
        return minimum
    frac = (value - nominal) / (maximum - nominal)
    return 1.0 + frac * (minimum - 1.0)


def _scale_below(value, warn, cutoff, minimum):
    warn = _safe_float(warn)
    cutoff = _safe_float(cutoff)
    minimum = max(0.0, min(1.0, _safe_float(minimum, 1.0)))
    if warn <= 0.0 or cutoff <= 0.0 or value is None:
        return 1.0
    value = _safe_float(value, warn)
    if value >= warn:
        return 1.0
    if value <= cutoff:
        return minimum
    frac = (warn - value) / max(1e-9, warn - cutoff)
    return 1.0 + frac * (minimum - 1.0)


def _quat_tilt_rad(q):
    q = _as_dict(q)
    x = _safe_float(q.get("x", 0.0))
    y = _safe_float(q.get("y", 0.0))
    z = _safe_float(q.get("z", 0.0))
    w = _safe_float(q.get("w", 1.0), 1.0)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # World Z component of the body Z axis for a body-to-world quaternion.
    r33 = 1.0 - 2.0 * (x * x + y * y)
    r33 = max(-1.0, min(1.0, r33))
    return math.acos(r33)


class AerialVelocityGateNode:
    def __init__(self):
        self.lock = threading.RLock()

        self.rate_hz = _param_float("~rate_hz", 100.0)
        self.raw_topic = rospy.get_param("~raw_velocity_topic", "/so101/ee_velocity_cmd_raw")
        self.output_topic = rospy.get_param("~output_velocity_topic", "/so101/ee_velocity_cmd")
        self.aerial_state_topic = rospy.get_param("~aerial_state_topic", "/aerial_manipulation/state")
        self.arm_status_topic = rospy.get_param("~arm_status_topic", "/so101/status")
        self.servo_status_topic = rospy.get_param("~servo_status_topic", "/so101/servo_status")
        self.status_topic = rospy.get_param("~status_topic", "/so101/aerial_velocity_gate_status")
        self.output_frame = rospy.get_param("~output_frame", "base_link")

        self.command_timeout_s = _param_float("~command_timeout_s", 0.20)
        self.aerial_state_timeout_s = _param_float("~aerial_state_timeout_s", 0.75)
        self.arm_status_timeout_s = _param_float("~arm_status_timeout_s", 0.75)
        self.servo_status_timeout_s = _param_float("~servo_status_timeout_s", 2.5)

        self.require_aerial_state = _param_bool("~require_aerial_state", False)
        self.require_mavros_connected = _param_bool("~require_mavros_connected", False)
        self.require_armed = _param_bool("~require_armed", False)
        self.require_guided = _param_bool("~require_guided", False)
        self.require_arm_state = _param_bool("~require_arm_state", True)
        self.require_arm_ok = _param_bool("~require_arm_ok", True)
        self.planar_xz_only = _param_bool("~planar_xz_only", True)

        self.max_speed = _param_float("~max_ee_speed_m_s", 0.10)
        self.max_accel = _param_float("~max_ee_accel_m_s2", 0.35)
        self.max_jerk = _param_float("~max_ee_jerk_m_s3", 1.20)
        self.min_scale = _param_float("~min_scale", 0.20)

        self.uav_nominal_speed = _param_float("~uav_nominal_speed_m_s", 1.0)
        self.uav_max_speed = _param_float("~uav_max_speed_m_s", 3.0)
        self.uav_speed_min_scale = _param_float("~uav_speed_min_scale", 0.35)
        self.uav_nominal_angular = _param_float("~uav_nominal_angular_rad_s", 0.8)
        self.uav_max_angular = _param_float("~uav_max_angular_rad_s", 2.5)
        self.uav_angular_min_scale = _param_float("~uav_angular_min_scale", 0.35)
        self.uav_nominal_dynamic_accel = _param_float("~uav_nominal_dynamic_accel_m_s2", 1.5)
        self.uav_max_dynamic_accel = _param_float("~uav_max_dynamic_accel_m_s2", 5.0)
        self.uav_accel_min_scale = _param_float("~uav_accel_min_scale", 0.30)
        self.uav_nominal_tilt_deg = _param_float("~uav_nominal_tilt_deg", 15.0)
        self.uav_max_tilt_deg = _param_float("~uav_max_tilt_deg", 35.0)
        self.uav_tilt_min_scale = _param_float("~uav_tilt_min_scale", 0.40)
        self.gravity_m_s2 = _param_float("~gravity_m_s2", 9.80665)

        self.tracking_error_nominal_rad = _param_float("~tracking_error_nominal_rad", 0.035)
        self.tracking_error_max_rad = _param_float("~tracking_error_max_rad", 0.090)
        self.tracking_error_min_scale = _param_float("~tracking_error_min_scale", 0.25)
        self.min_write_rate_hz = _param_float("~min_write_rate_hz", 60.0)
        self.write_rate_soft_margin_hz = _param_float("~write_rate_soft_margin_hz", 15.0)

        self.uav_battery_warn_v = _param_float("~uav_battery_warn_v", 0.0)
        self.uav_battery_cutoff_v = _param_float("~uav_battery_cutoff_v", 0.0)
        self.servo_voltage_warn_v = _param_float("~servo_voltage_warn_v", 10.5)
        self.servo_voltage_cutoff_v = _param_float("~servo_voltage_cutoff_v", 9.5)
        self.servo_temp_warn_c = _param_float("~servo_temp_warn_c", 65.0)
        self.servo_temp_cutoff_c = _param_float("~servo_temp_cutoff_c", 75.0)
        self.servo_current_warn_ma = _param_float("~servo_current_warn_ma", 0.0)
        self.servo_current_cutoff_ma = _param_float("~servo_current_cutoff_ma", 0.0)

        self.raw_velocity = [0.0, 0.0, 0.0]
        self.raw_stamp = 0.0
        self.aerial_state = {}
        self.aerial_stamp = 0.0
        self.arm_status = {}
        self.arm_status_stamp = 0.0
        self.servo_status = {}
        self.servo_status_stamp = 0.0
        self.output_velocity = [0.0, 0.0, 0.0]
        self.output_accel = [0.0, 0.0, 0.0]
        self.last_tick = time.time()
        self.last_status = {}

        self.pub = rospy.Publisher(self.output_topic, TwistStamped, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.raw_topic, TwistStamped, self._on_raw_velocity, queue_size=1)
        rospy.Subscriber(self.aerial_state_topic, String, self._on_aerial_state, queue_size=1)
        rospy.Subscriber(self.arm_status_topic, String, self._on_arm_status, queue_size=1)
        rospy.Subscriber(self.servo_status_topic, String, self._on_servo_status, queue_size=1)

    def _now(self):
        return rospy.Time.now().to_sec()

    def _on_raw_velocity(self, msg):
        with self.lock:
            self.raw_velocity = [
                float(msg.twist.linear.x),
                0.0 if self.planar_xz_only else float(msg.twist.linear.y),
                float(msg.twist.linear.z),
            ]
            self.raw_stamp = self._now()

    def _on_aerial_state(self, msg):
        with self.lock:
            self.aerial_state = _parse_json(msg.data)
            self.aerial_stamp = self._now()

    def _on_arm_status(self, msg):
        with self.lock:
            self.arm_status = _parse_json(msg.data)
            self.arm_status_stamp = self._now()

    def _on_servo_status(self, msg):
        with self.lock:
            self.servo_status = _parse_json(msg.data)
            self.servo_status_stamp = self._now()

    def _snapshot(self, now):
        with self.lock:
            return {
                "raw_velocity": list(self.raw_velocity),
                "raw_age_s": now - self.raw_stamp if self.raw_stamp > 0.0 else None,
                "aerial_state": dict(_as_dict(self.aerial_state)),
                "aerial_age_s": now - self.aerial_stamp if self.aerial_stamp > 0.0 else None,
                "arm_status": dict(_as_dict(self.arm_status)),
                "arm_status_age_s": now - self.arm_status_stamp if self.arm_status_stamp > 0.0 else None,
                "servo_status": dict(_as_dict(self.servo_status)),
                "servo_status_age_s": now - self.servo_status_stamp if self.servo_status_stamp > 0.0 else None,
            }

    def _arm_payload(self, snap):
        aerial = _as_dict(snap["aerial_state"])
        so101 = _as_dict(aerial.get("so101", {}))
        arm = dict(_as_dict(so101.get("status", {})))
        arm.update(_as_dict(snap["arm_status"]))
        return arm

    def _servo_payload(self, snap):
        aerial = _as_dict(snap["aerial_state"])
        so101 = _as_dict(aerial.get("so101", {}))
        servo = dict(_as_dict(so101.get("servo_status", {})))
        servo.update(_as_dict(snap["servo_status"]))
        return servo

    def _hard_gate_reasons(self, snap, now):
        reasons = []
        raw_age = snap["raw_age_s"]
        aerial_age = snap["aerial_age_s"]
        arm_age = snap["arm_status_age_s"]
        aerial = _as_dict(snap["aerial_state"])
        ready = _as_dict(aerial.get("ready", {}))
        mavros = _as_dict(aerial.get("mavros", {}))
        arm = self._arm_payload(snap)

        if raw_age is None or raw_age > self.command_timeout_s:
            reasons.append("raw_command_stale")
        if self.require_aerial_state and (aerial_age is None or aerial_age > self.aerial_state_timeout_s):
            reasons.append("aerial_state_stale")
        if self.require_mavros_connected and not bool(ready.get("mavros_connected", mavros.get("connected", False))):
            reasons.append("mavros_not_connected")
        if self.require_armed and not bool(mavros.get("armed", False)):
            reasons.append("uav_not_armed")
        if self.require_guided and not bool(mavros.get("guided", False)):
            reasons.append("uav_not_guided")
        arm_fresh = (arm_age is not None and arm_age <= self.arm_status_timeout_s) or bool(ready.get("arm_state_fresh", False))
        if self.require_arm_state and not arm_fresh:
            reasons.append("arm_state_stale")
        if self.require_arm_ok:
            arm_ok = bool(ready.get("arm_ok", True))
            arm_error = bool(arm.get("estop") or arm.get("relaxed") or arm.get("last_backend_error"))
            if not arm_ok or arm_error:
                reasons.append("arm_not_ok")
        return reasons

    def _scale_from_aerial_state(self, snap):
        aerial = _as_dict(snap["aerial_state"])
        mavros = _as_dict(aerial.get("mavros", {}))
        velocity = _as_dict(mavros.get("velocity_local", {}))
        imu = _as_dict(mavros.get("imu", {}))
        battery = _as_dict(mavros.get("battery", {}))
        components = {}

        linear_speed = _dict_vec_norm(velocity.get("linear", {}))
        components["uav_linear_speed"] = _scale_above(
            linear_speed, self.uav_nominal_speed, self.uav_max_speed, self.uav_speed_min_scale
        )

        angular = imu.get("angular_velocity") or velocity.get("angular", {})
        angular_speed = _dict_vec_norm(angular)
        components["uav_angular_speed"] = _scale_above(
            angular_speed, self.uav_nominal_angular, self.uav_max_angular, self.uav_angular_min_scale
        )

        linear_accel_payload = imu.get("linear_acceleration")
        if isinstance(linear_accel_payload, dict):
            accel_norm = _dict_vec_norm(linear_accel_payload)
            dynamic_accel = max(0.0, abs(accel_norm - self.gravity_m_s2))
            components["uav_dynamic_accel"] = _scale_above(
                dynamic_accel,
                self.uav_nominal_dynamic_accel,
                self.uav_max_dynamic_accel,
                self.uav_accel_min_scale,
            )
        else:
            accel_norm = None
            dynamic_accel = None
            components["uav_dynamic_accel"] = 1.0

        tilt_deg = math.degrees(_quat_tilt_rad(imu.get("orientation", {})))
        components["uav_tilt"] = _scale_above(
            tilt_deg, self.uav_nominal_tilt_deg, self.uav_max_tilt_deg, self.uav_tilt_min_scale
        )

        components["uav_battery"] = _scale_below(
            battery.get("voltage"), self.uav_battery_warn_v, self.uav_battery_cutoff_v, self.min_scale
        )

        metrics = {
            "uav_linear_speed_m_s": linear_speed,
            "uav_angular_speed_rad_s": angular_speed,
            "uav_dynamic_accel_m_s2": dynamic_accel,
            "uav_tilt_deg": tilt_deg,
            "uav_battery_v": battery.get("voltage"),
        }
        return components, metrics

    def _scale_from_arm_state(self, snap):
        arm = self._arm_payload(snap)
        components = {}
        tracking_error = 0.0
        errors = _as_dict(arm.get("tracking_error_rad", {}))
        if errors:
            for name, value in errors.items():
                if name != "gripper":
                    tracking_error = max(tracking_error, abs(_safe_float(value)))
        components["arm_tracking_error"] = _scale_above(
            tracking_error,
            self.tracking_error_nominal_rad,
            self.tracking_error_max_rad,
            self.tracking_error_min_scale,
        )

        write_rate = None
        rates = _as_dict(arm.get("effective_rates_hz", {}))
        write_rate = rates.get("write")
        if write_rate is None or self.min_write_rate_hz <= 0.0:
            components["arm_write_rate"] = 1.0
        else:
            soft = max(0.0, self.min_write_rate_hz - self.write_rate_soft_margin_hz)
            components["arm_write_rate"] = _scale_below(write_rate, self.min_write_rate_hz, soft, self.min_scale)

        servo = self._servo_payload(snap)
        joints = servo.get("joints", {}) if isinstance(servo, dict) else {}
        min_voltage = None
        max_temp = None
        max_current = None
        if isinstance(joints, dict):
            for diag in joints.values():
                diag = _as_dict(diag)
                voltage = diag.get("voltage_v")
                temp = diag.get("temperature_c")
                current = diag.get("current_ma")
                if voltage is not None:
                    voltage = _safe_float(voltage, None)
                    if voltage is not None:
                        min_voltage = voltage if min_voltage is None else min(min_voltage, voltage)
                if temp is not None:
                    temp = _safe_float(temp, None)
                    if temp is not None:
                        max_temp = temp if max_temp is None else max(max_temp, temp)
                if current is not None:
                    current = _safe_float(current, None)
                    if current is not None:
                        current = abs(current)
                        max_current = current if max_current is None else max(max_current, current)

        components["servo_voltage"] = _scale_below(
            min_voltage, self.servo_voltage_warn_v, self.servo_voltage_cutoff_v, self.min_scale
        )
        components["servo_temperature"] = _scale_above(
            max_temp or 0.0, self.servo_temp_warn_c, self.servo_temp_cutoff_c, self.min_scale
        )
        components["servo_current"] = _scale_above(
            max_current or 0.0, self.servo_current_warn_ma, self.servo_current_cutoff_ma, self.min_scale
        )

        metrics = {
            "arm_tracking_error_max_rad": tracking_error,
            "arm_write_rate_hz": write_rate,
            "servo_min_voltage_v": min_voltage,
            "servo_max_temperature_c": max_temp,
            "servo_max_current_ma": max_current,
        }
        return components, metrics

    def _compute_target(self, snap, now):
        reasons = self._hard_gate_reasons(snap, now)
        aerial_components, aerial_metrics = self._scale_from_aerial_state(snap)
        arm_components, arm_metrics = self._scale_from_arm_state(snap)
        components = {}
        components.update(aerial_components)
        components.update(arm_components)
        scale = min([1.0] + list(components.values()))
        scale = max(0.0, min(1.0, scale))
        if reasons:
            scale = 0.0
        elif scale > 0.0:
            scale = max(self.min_scale, scale)

        raw = list(snap["raw_velocity"])
        if self.planar_xz_only:
            raw[1] = 0.0
        max_speed = max(0.0, self.max_speed * scale)
        target = self._clamp_speed(raw, max_speed)
        metrics = {}
        metrics.update(aerial_metrics)
        metrics.update(arm_metrics)
        return target, scale, max_speed, reasons, components, metrics

    def _clamp_speed(self, velocity, max_speed):
        velocity = [float(v) for v in velocity]
        if self.planar_xz_only:
            velocity[1] = 0.0
            norm = math.sqrt(velocity[0] * velocity[0] + velocity[2] * velocity[2])
        else:
            norm = _vec_norm(velocity)
        max_speed = max(0.0, float(max_speed))
        if norm > max_speed and norm > 1e-9:
            scale = max_speed / norm
            velocity = [v * scale for v in velocity]
        return velocity

    def _step_limiter(self, target, dt, scale):
        dt = max(1e-4, min(0.1, float(dt)))
        accel_limit = max(0.0, self.max_accel * max(scale, self.min_scale if scale > 0.0 else 1.0))
        jerk_limit = max(0.0, self.max_jerk * max(scale, self.min_scale if scale > 0.0 else 1.0))
        if scale <= 0.0:
            accel_limit = max(self.max_accel, 1e-6)
            jerk_limit = max(self.max_jerk, 1e-6)

        out = list(self.output_velocity)
        acc = list(self.output_accel)
        for idx in range(3):
            desired_acc = (target[idx] - out[idx]) / dt
            desired_acc = max(-accel_limit, min(accel_limit, desired_acc))
            da = desired_acc - acc[idx]
            max_da = jerk_limit * dt
            if max_da > 0.0 and abs(da) > max_da:
                da = math.copysign(max_da, da)
            acc[idx] += da
            candidate = out[idx] + acc[idx] * dt
            if (target[idx] - out[idx]) * (target[idx] - candidate) <= 0.0:
                candidate = target[idx]
                acc[idx] = 0.0
            out[idx] = candidate
        self.output_velocity = self._clamp_speed(out, self.max_speed)
        if self.planar_xz_only:
            self.output_velocity[1] = 0.0
            acc[1] = 0.0
        self.output_accel = acc
        return list(self.output_velocity)

    def _publish_velocity(self, now, velocity):
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.from_sec(now)
        msg.header.frame_id = self.output_frame
        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])
        self.pub.publish(msg)

    def _publish_status(self, now, snap, target, output, scale, max_speed, reasons, components, metrics):
        payload = {
            "stamp": now,
            "active": bool(not reasons and _vec_norm(output) > 1e-5),
            "scale": float(scale),
            "max_speed_m_s": float(max_speed),
            "raw_velocity_m_s": [float(v) for v in snap["raw_velocity"]],
            "target_velocity_m_s": [float(v) for v in target],
            "output_velocity_m_s": [float(v) for v in output],
            "hard_gate_reasons": list(reasons),
            "scale_components": {k: float(v) for k, v in components.items()},
            "metrics": metrics,
            "ages_s": {
                "raw_command": snap["raw_age_s"],
                "aerial_state": snap["aerial_age_s"],
                "arm_status": snap["arm_status_age_s"],
                "servo_status": snap["servo_status_age_s"],
            },
            "limits": {
                "max_ee_speed_m_s": self.max_speed,
                "max_ee_accel_m_s2": self.max_accel,
                "max_ee_jerk_m_s3": self.max_jerk,
            },
        }
        self.last_status = payload
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = self._now()
            dt = now - self.last_tick
            self.last_tick = now
            try:
                snap = self._snapshot(now)
                target, scale, max_speed, reasons, components, metrics = self._compute_target(snap, now)
                output = self._step_limiter(target, dt, scale)
                self._publish_velocity(now, output)
                self._publish_status(now, snap, target, output, scale, max_speed, reasons, components, metrics)
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0,
                    "SO101 aerial velocity gate error; commanding zero velocity: %s\n%s",
                    exc,
                    traceback.format_exc(),
                )
                output = self._step_limiter([0.0, 0.0, 0.0], dt, 0.0)
                self._publish_velocity(now, output)
            rate.sleep()


def main():
    rospy.init_node("so101_aerial_velocity_gate")
    AerialVelocityGateNode().spin()


if __name__ == "__main__":
    main()

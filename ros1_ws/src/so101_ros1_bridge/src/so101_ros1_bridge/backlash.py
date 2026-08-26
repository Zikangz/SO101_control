import bisect
import copy
import json
import math
import os
import time


def load_backlash_profile(path):
    if not path:
        return {}
    expanded = os.path.expanduser(str(path))
    with open(expanded, "r") as handle:
        profile = json.load(handle)
    if not isinstance(profile, dict):
        raise ValueError("backlash compensation profile must be a JSON object")
    joints = profile.get("joints")
    if not isinstance(joints, dict) or not joints:
        raise ValueError("backlash compensation profile has no joints")
    return profile


def _clip(value, lo, hi):
    return max(float(lo), min(float(hi), float(value)))


def _finite_or_none(value):
    if value is None or value == "":
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


def _interpolate(xs, ys, x):
    if not xs or not ys:
        return 0.0
    if len(xs) != len(ys):
        raise ValueError("backlash profile breakpoints and bias arrays differ in length")
    x = float(x)
    if len(xs) == 1 or x <= xs[0]:
        return float(ys[0])
    if x >= xs[-1]:
        return float(ys[-1])
    idx = bisect.bisect_right(xs, x) - 1
    idx = max(0, min(idx, len(xs) - 2))
    x0 = float(xs[idx])
    x1 = float(xs[idx + 1])
    if x1 <= x0:
        return float(ys[idx])
    alpha = (x - x0) / (x1 - x0)
    return float(ys[idx]) + alpha * (float(ys[idx + 1]) - float(ys[idx]))


class DirectionalBacklashCompensator:
    """Bounded feed-forward compensation for direction-dependent joint error.

    This is intentionally conservative: it never estimates torque, never uses an
    integral term, and only adds a small, slew-limited position bias from an
    offline profile. It is meant for STS position servos where the only reliable
    closed-loop command interface is position.
    """

    def __init__(
        self,
        joint_order,
        limits,
        active_joints,
        locked_joints,
        profile=None,
        enabled=False,
        trajectory_only=True,
        max_abs_bias_rad=None,
        bias_slew_rad_s=None,
        velocity_threshold_rad_s=None,
        position_hysteresis_rad=None,
        limit_margin_rad=None,
        joints=None,
    ):
        self.joint_order = list(joint_order)
        self.limits = {name: tuple(value) for name, value in limits.items()}
        self.active_joints = set(active_joints)
        self.locked_joints = set(locked_joints or [])
        self.profile = copy.deepcopy(profile or {})
        self.enabled = bool(enabled)
        self.trajectory_only = bool(trajectory_only)
        self.velocity_threshold_rad_s = self._profile_float(
            "velocity_threshold_rad_s", velocity_threshold_rad_s, 0.006
        )
        self.position_hysteresis_rad = self._profile_float(
            "position_hysteresis_rad", position_hysteresis_rad, 0.003
        )
        self.bias_slew_rad_s = self._profile_float("bias_slew_rad_s", bias_slew_rad_s, 0.025)
        self.limit_margin_rad = self._profile_float("limit_margin_rad", limit_margin_rad, 0.04)
        self.max_abs_bias_rad = self._profile_float("max_abs_bias_rad", max_abs_bias_rad, 0.035)
        requested_joints = set(joints or [])
        raw_profiles = self.profile.get("joints", {}) if isinstance(self.profile, dict) else {}
        self.joint_profiles = {}
        for name, data in raw_profiles.items():
            if requested_joints and name not in requested_joints:
                continue
            if name not in self.joint_order:
                continue
            if name not in self.active_joints or name in self.locked_joints or name == "gripper":
                continue
            self.joint_profiles[name] = self._normalise_joint_profile(name, data)
        if self.enabled and not self.joint_profiles:
            raise ValueError("backlash compensation enabled but profile has no usable active joints")

        self.bias = {name: 0.0 for name in self.joint_order}
        self.direction = {name: 0 for name in self.joint_order}
        self.velocity = {name: 0.0 for name in self.joint_order}
        self.target_bias = {name: 0.0 for name in self.joint_order}
        self.last_switch_position = {name: None for name in self.joint_order}
        self.previous_desired = None
        self.previous_time = None
        self.active = False
        self.last_update_wall_time = time.time()

    def _profile_float(self, key, override, default):
        value = _finite_or_none(override)
        if value is None:
            value = _finite_or_none(self.profile.get(key) if isinstance(self.profile, dict) else None)
        if value is None:
            value = default
        return float(value)

    def _normalise_joint_profile(self, name, data):
        if not isinstance(data, dict):
            raise ValueError("backlash profile for %s must be an object" % name)
        breakpoints = [float(value) for value in data.get("breakpoints_rad", [])]
        positive = [float(value) for value in data.get("positive_bias_rad", [])]
        negative = [float(value) for value in data.get("negative_bias_rad", [])]
        if not breakpoints or len(breakpoints) != len(positive) or len(breakpoints) != len(negative):
            raise ValueError("backlash profile for %s has invalid table lengths" % name)
        combined = sorted(zip(breakpoints, positive, negative), key=lambda item: item[0])
        max_abs = _finite_or_none(data.get("max_abs_bias_rad"))
        if max_abs is None:
            max_abs = self.max_abs_bias_rad
        max_abs = max(0.0, min(float(max_abs), self.max_abs_bias_rad))
        return {
            "breakpoints_rad": [item[0] for item in combined],
            "positive_bias_rad": [_clip(item[1], -max_abs, max_abs) for item in combined],
            "negative_bias_rad": [_clip(item[2], -max_abs, max_abs) for item in combined],
            "max_abs_bias_rad": max_abs,
        }

    def reset(self, desired=None):
        for name in self.joint_order:
            self.bias[name] = 0.0
            self.direction[name] = 0
            self.velocity[name] = 0.0
            self.target_bias[name] = 0.0
            self.last_switch_position[name] = None
        self.previous_desired = copy.deepcopy(desired) if desired is not None else None
        self.previous_time = None
        self.active = False

    def _update_direction(self, name, position, velocity):
        if abs(velocity) < self.velocity_threshold_rad_s:
            return self.direction.get(name, 0)
        proposed = 1 if velocity > 0.0 else -1
        current = self.direction.get(name, 0)
        if proposed == current:
            return current
        last_switch = self.last_switch_position.get(name)
        if last_switch is not None and abs(float(position) - float(last_switch)) < self.position_hysteresis_rad:
            return current
        self.direction[name] = proposed
        self.last_switch_position[name] = float(position)
        return proposed

    def _desired_bias(self, name, position, direction):
        if direction == 0:
            return 0.0
        profile = self.joint_profiles[name]
        table_name = "positive_bias_rad" if direction > 0 else "negative_bias_rad"
        value = _interpolate(profile["breakpoints_rad"], profile[table_name], position)
        return _clip(value, -profile["max_abs_bias_rad"], profile["max_abs_bias_rad"])

    def apply(self, desired, now=None, trajectory_active=False):
        now = time.time() if now is None else float(now)
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = max(0.0, min(0.1, now - float(self.previous_time)))
        previous = self.previous_desired or desired
        output = copy.deepcopy(desired)
        command_active = bool(self.enabled and self.joint_profiles)
        if self.trajectory_only and not trajectory_active:
            command_active = False
        self.active = command_active

        for name in self.joint_order:
            position = float(desired.get(name, previous.get(name, 0.0)))
            prev_position = float(previous.get(name, position))
            velocity = 0.0 if dt <= 1e-6 else (position - prev_position) / dt
            self.velocity[name] = velocity
            if name not in self.joint_profiles:
                self.target_bias[name] = 0.0
                continue
            direction = self._update_direction(name, position, velocity) if command_active else self.direction.get(name, 0)
            target_bias = self._desired_bias(name, position, direction) if command_active else 0.0
            self.target_bias[name] = target_bias
            previous_bias = float(self.bias.get(name, 0.0))
            if dt <= 1e-6 or self.bias_slew_rad_s <= 0.0:
                next_bias = target_bias if dt <= 1e-6 else previous_bias
            else:
                max_delta = self.bias_slew_rad_s * dt
                next_bias = previous_bias + _clip(target_bias - previous_bias, -max_delta, max_delta)
            max_abs = self.joint_profiles[name]["max_abs_bias_rad"]
            next_bias = _clip(next_bias, -max_abs, max_abs)
            lo, hi = self.limits.get(name, (-math.inf, math.inf))
            lo = min(float(hi), float(lo) + self.limit_margin_rad)
            hi = max(float(lo), float(hi) - self.limit_margin_rad)
            output[name] = _clip(position + next_bias, lo, hi)
            self.bias[name] = float(output[name]) - position

        self.previous_desired = copy.deepcopy(desired)
        self.previous_time = now
        self.last_update_wall_time = time.time()
        return output

    def status(self):
        return {
            "enabled": bool(self.enabled),
            "active": bool(self.active),
            "trajectory_only": bool(self.trajectory_only),
            "profile_loaded": bool(self.joint_profiles),
            "profile_source": self.profile.get("source", "") if isinstance(self.profile, dict) else "",
            "joints": sorted(self.joint_profiles.keys()),
            "velocity_threshold_rad_s": self.velocity_threshold_rad_s,
            "position_hysteresis_rad": self.position_hysteresis_rad,
            "bias_slew_rad_s": self.bias_slew_rad_s,
            "limit_margin_rad": self.limit_margin_rad,
            "max_abs_bias_rad": self.max_abs_bias_rad,
            "direction": {name: self.direction.get(name, 0) for name in self.joint_profiles},
            "velocity_rad_s": {name: self.velocity.get(name, 0.0) for name in self.joint_profiles},
            "target_bias_rad": {name: self.target_bias.get(name, 0.0) for name in self.joint_profiles},
            "applied_bias_rad": {name: self.bias.get(name, 0.0) for name in self.joint_profiles},
        }

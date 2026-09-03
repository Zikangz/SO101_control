#!/usr/bin/env python3
"""Run a small Cartesian end-effector sine test through the SO101 IK node."""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.kinematics import SO101Kinematics
from so101_ros1_bridge.poses import JOINT_ORDER, SAFE_POSES


class RosCache:
    def __init__(self):
        self.lock = threading.RLock()
        self.ee = None
        self.joints = {}
        self.commanded_joints = {}
        self.servo_status = {}
        self.kinematics_status = {}
        self.driver_status = {}
        self.driver_status_time = 0.0
        rospy.Subscriber("/so101/end_effector_pose", PoseStamped, self._on_ee, queue_size=1)
        rospy.Subscriber("/so101/joint_states", JointState, self._on_joint_state, queue_size=1)
        rospy.Subscriber("/so101/commanded_joint_states", JointState, self._on_commanded_joint_state, queue_size=1)
        rospy.Subscriber("/so101/servo_status", String, self._on_servo_status, queue_size=1)
        rospy.Subscriber("/so101/kinematics_status", String, self._on_kinematics_status, queue_size=1)
        rospy.Subscriber("/so101/status", String, self._on_driver_status, queue_size=1)

    def _on_ee(self, msg):
        with self.lock:
            self.ee = msg

    def _on_joint_state(self, msg):
        with self.lock:
            self.joints = dict(zip(msg.name, msg.position))

    def _on_commanded_joint_state(self, msg):
        with self.lock:
            self.commanded_joints = dict(zip(msg.name, msg.position))

    def _on_servo_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {}
        with self.lock:
            self.servo_status = payload.get("joints", {})

    def _on_kinematics_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {"raw": msg.data}
        with self.lock:
            self.kinematics_status = payload

    def _on_driver_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            payload = {}
        with self.lock:
            self.driver_status = payload
            self.driver_status_time = time.time()

    def snapshot(self):
        with self.lock:
            return (
                self.ee,
                dict(self.joints),
                dict(self.commanded_joints),
                dict(self.servo_status),
                dict(self.kinematics_status),
            )

    def latest_driver_status(self):
        with self.lock:
            return dict(self.driver_status), float(self.driver_status_time)


def _pose_xyz(msg):
    return [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


def _latest_center(cache, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        ee, _, _, _, _ = cache.snapshot()
        if ee is not None:
            return ee
        rospy.sleep(0.05)
    raise RuntimeError("No /so101/end_effector_pose received; start hardware bridge with kinematics enabled")


def _wait_driver_status(cache, timeout_s):
    deadline = time.time() + float(timeout_s)
    while time.time() < deadline and not rospy.is_shutdown():
        status, received_at = cache.latest_driver_status()
        if status and received_at > 0.0:
            return status
        rospy.sleep(0.05)
    return {}


def _check_driver_preflight(cache, args):
    """Reject controller combinations that invalidate hardware tracking tests."""
    status = _wait_driver_status(cache, min(2.0, max(0.2, float(args.timeout))))
    if not status:
        raise RuntimeError("No /so101/status received; cannot verify the hardware controller configuration")

    assist = status.get("feedback_position_assist", {})
    if bool(assist.get("enabled", False)) and not args.allow_feedback_position_assist:
        raise RuntimeError(
            "The ROS feedback position assist (outer P/PI) is enabled. Disable it before any "
            "hardware tracking or center diagnostic: set SO101_FEEDBACK_POSITION_ASSIST_GAIN=0.0 "
            "and SO101_FEEDBACK_POSITION_ASSIST_INTEGRAL_GAIN=0.0 when starting the bridge. "
            "The delayed outer loop can create the shoulder correction and visible oscillation being measured."
        )
    return status


def _check_synchronous_writes(cache, args):
    status = _wait_driver_status(cache, 0.5)
    transport = status.get("transport", {}) if status else {}
    write_mode = transport.get("last_nonempty_write_mode", "")
    if write_mode == "individual_write_pos_ex" and not args.allow_sequential_writes:
        detail = transport.get("last_sync_write_error", "")
        raise RuntimeError(
            "The Feetech backend fell back to sequential per-servo writes%s. "
            "Do not start a synchronized Cartesian path until SyncWritePosEx is working; "
            "sequential writes add inter-joint skew and can appear as arm jitter."
            % (": " + str(detail) if detail else "")
        )


def _publish_target(pub, frame_id, xyz, orientation):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    msg.pose.orientation = orientation
    pub.publish(msg)


def _publish_joint_pose(pub, pose, duration_s):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = list(pose.keys())
    point = JointTrajectoryPoint()
    point.positions = [float(pose[name]) for name in msg.joint_names]
    point.time_from_start = rospy.Duration(duration_s)
    msg.points = [point]
    pub.publish(msg)


def _publish_joint_trajectory(pub, joint_names, timed_positions):
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = list(joint_names)
    for elapsed, positions in timed_positions:
        point = JointTrajectoryPoint()
        point.positions = [float(positions[name]) for name in msg.joint_names]
        point.time_from_start = rospy.Duration(float(elapsed))
        msg.points.append(point)
    pub.publish(msg)


def _wait_for_publisher_connection(pub, timeout_s, topic):
    deadline = time.time() + float(timeout_s)
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)
    if pub.get_num_connections() == 0:
        raise RuntimeError("No subscriber connected to %s; is the SO101 driver running?" % topic)


def _wait_joints_close(cache, target, timeout_s, tolerance):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        _, joints, _, _, _ = cache.snapshot()
        if joints:
            errors = [abs(float(joints.get(name, 0.0)) - float(value)) for name, value in target.items()]
            if all(error <= tolerance for error in errors):
                return True
        rospy.sleep(0.05)
    return False


def _joint_error_details(cache, target):
    _, joints, commanded_joints, _, _ = cache.snapshot()
    details = []
    max_abs_error = 0.0
    for name, target_value in target.items():
        measured = joints.get(name, None)
        commanded = commanded_joints.get(name, None)
        if measured is None:
            details.append((name, float(target_value), None, commanded, None))
            continue
        error = float(measured) - float(target_value)
        max_abs_error = max(max_abs_error, abs(error))
        details.append((name, float(target_value), float(measured), commanded, error))
    return max_abs_error, details


def _format_joint_error_details(details):
    lines = []
    for name, target, measured, commanded, error in details:
        measured_text = "missing" if measured is None else "%.4f" % measured
        commanded_text = "missing" if commanded is None else "%.4f" % float(commanded)
        error_text = "missing" if error is None else "%+.4f" % error
        lines.append(
            "    %-14s target=% .4f measured=%s commanded=%s error=%s"
            % (name, target, measured_text, commanded_text, error_text)
        )
    return "\n".join(lines)


def _wait_ee_close(cache, target_xyz, timeout_s, tolerance_m):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not rospy.is_shutdown():
        ee, _, _, _, _ = cache.snapshot()
        if ee is not None:
            xyz = _pose_xyz(ee)
            error = math.sqrt(sum((float(xyz[idx]) - float(target_xyz[idx])) ** 2 for idx in range(3)))
            if error <= tolerance_m:
                return True
        rospy.sleep(0.05)
    return False


def _hold_cartesian_target(pub, cache, frame_id, center, orientation, duration_s, rate_hz, tolerance_m):
    if duration_s <= 0.0:
        return
    print(
        "Moving to trajectory center=(%.4f, %.4f, %.4f) for %.2fs before sine"
        % (center[0], center[1], center[2], duration_s)
    )
    rate = rospy.Rate(max(1.0, float(rate_hz)))
    end_time = time.time() + float(duration_s)
    while time.time() < end_time and not rospy.is_shutdown():
        _publish_target(pub, frame_id, center, orientation)
        rate.sleep()
    ok = _wait_ee_close(cache, center, 1.0, tolerance_m)
    if not ok:
        ee, _, _, _, _ = cache.snapshot()
        if ee is not None:
            xyz = _pose_xyz(ee)
            error = math.sqrt(sum((float(xyz[idx]) - float(center[idx])) ** 2 for idx in range(3)))
            print(
                "Warning: EE is %.3fm from trajectory center after pre-positioning" % error,
                file=sys.stderr,
            )


def _prepare_pose(cache, args):
    if args.prep_pose == "none":
        return
    pose = SAFE_POSES[args.prep_pose]
    pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1, latch=True)
    deadline = time.time() + args.timeout
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)
    print("Preparing SO101 pose:", args.prep_pose, pose)
    _publish_joint_pose(pub, pose, args.prep_duration)
    ok = _wait_joints_close(cache, pose, max(args.timeout, args.prep_duration + 2.0), args.prep_tolerance)
    if not ok:
        print("Warning: prep pose did not fully converge before trajectory test", file=sys.stderr)
    rospy.sleep(args.settle)


def _target_offset(args, elapsed):
    w = 2.0 * math.pi * args.frequency
    x_amp = args.x_amplitude if args.x_amplitude is not None else args.amplitude
    y_amp = args.y_amplitude if args.y_amplitude is not None else 0.0
    z_amp = args.z_amplitude if args.z_amplitude is not None else args.amplitude

    def eased_segment(points):
        phase = (elapsed * args.frequency) % 1.0
        scaled = phase * len(points)
        idx = int(math.floor(scaled)) % len(points)
        frac = scaled - math.floor(scaled)
        # Half-cosine interpolation gives zero velocity at each vertex, which
        # makes cornered diagnostic paths less harsh on the low-cost servos.
        ease = 0.5 - 0.5 * math.cos(math.pi * frac)
        start = points[idx]
        end = points[(idx + 1) % len(points)]
        return [start[i] + (end[i] - start[i]) * ease for i in range(3)]

    if args.pattern == "axis":
        offset = [0.0, 0.0, 0.0]
        offset[{"x": 0, "y": 1, "z": 2}[args.axis]] = args.amplitude * math.sin(w * elapsed)
        return offset
    if args.pattern == "circle":
        return [x_amp * math.cos(w * elapsed), y_amp * math.sin(w * elapsed), z_amp * math.sin(w * elapsed)]
    if args.pattern == "xz_sine":
        return [x_amp * math.sin(w * elapsed), 0.0, z_amp * math.sin(2.0 * w * elapsed)]
    if args.pattern == "xz_edge_vertex8":
        return [x_amp * math.sin(w * elapsed), 0.0, z_amp * math.cos(2.0 * w * elapsed)]
    if args.pattern == "xz_vertex_diamond":
        return eased_segment(
            [
                [0.0, 0.0, z_amp],
                [x_amp, 0.0, 0.0],
                [0.0, 0.0, -z_amp],
                [-x_amp, 0.0, 0.0],
            ]
        )
    if args.pattern == "xz_zigzag":
        return eased_segment(
            [
                [-x_amp, 0.0, z_amp],
                [x_amp, 0.0, z_amp],
                [-x_amp, 0.0, -z_amp],
                [x_amp, 0.0, -z_amp],
            ]
        )
    if args.pattern == "figure8":
        return [x_amp * math.sin(w * elapsed), y_amp * math.sin(w * elapsed), z_amp * 0.5 * math.sin(2.0 * w * elapsed)]
    if args.pattern == "lissajous":
        return [
            x_amp * math.sin(w * elapsed),
            y_amp * math.sin(1.5 * w * elapsed + math.pi / 3.0),
            z_amp * math.sin(2.0 * w * elapsed + math.pi / 2.0),
        ]
    raise RuntimeError("Unsupported pattern: %s" % args.pattern)


def _phase_profile_value(coefficients, harmonics, phase):
    value = float(coefficients[0])
    for k in range(1, int(harmonics) + 1):
        value += float(coefficients[2 * k - 1]) * math.sin(k * phase)
        value += float(coefficients[2 * k]) * math.cos(k * phase)
    return value


def _z_feedforward_bias(args, elapsed, ramp_scale):
    profile = getattr(args, "phase_z_profile", None)
    if profile is None:
        return float(args.z_feedforward_bias)
    phase = 2.0 * math.pi * float(profile["frequency_hz"]) * float(elapsed)
    harmonics = int(profile["harmonics"])
    coefficients = profile["coefficients_m"]
    bias = _phase_profile_value(coefficients, harmonics, phase)
    return float(ramp_scale) * bias


def _feedforward_biases(args, elapsed, ramp_scale):
    profile = getattr(args, "phase_xz_profile", None)
    if profile is not None:
        phase = 2.0 * math.pi * float(profile["frequency_hz"]) * float(elapsed)
        harmonics = int(profile["harmonics"])
        x_bias = _phase_profile_value(profile["coefficients_x_m"], harmonics, phase)
        z_bias = _phase_profile_value(profile["coefficients_z_m"], harmonics, phase)
        return [float(ramp_scale) * x_bias, 0.0, float(ramp_scale) * z_bias]
    return [0.0, 0.0, _z_feedforward_bias(args, elapsed, ramp_scale)]


def _command_target(args, target, elapsed, ramp_scale):
    bias = _feedforward_biases(args, elapsed, ramp_scale)
    return [float(target[idx]) + bias[idx] for idx in range(3)]


def _configure_phase_z_profile(args, center):
    args.phase_z_profile = None
    path = getattr(args, "phase_z_compensation_profile", "")
    if not path:
        return
    if abs(float(args.z_feedforward_bias)) > 1e-9:
        raise RuntimeError("--phase-z-compensation-profile already contains the full Z bias; use --z-feedforward-bias 0")
    try:
        with open(path) as handle:
            profile = json.load(handle)
    except (OSError, ValueError) as exc:
        raise RuntimeError("Cannot load phase Z compensation profile %s: %s" % (path, exc))
    if profile.get("schema") != "so101_phase_z_compensation_v1":
        raise RuntimeError("Unsupported phase Z compensation profile schema")
    expected = {
        "pattern": args.pattern,
        "frequency_hz": float(args.frequency),
        "center_xyz_m": [float(value) for value in center],
        "x_amplitude_m": float(args.x_amplitude if args.x_amplitude is not None else args.amplitude),
        "z_amplitude_m": float(args.z_amplitude if args.z_amplitude is not None else args.amplitude),
    }
    if profile.get("pattern") != expected["pattern"]:
        raise RuntimeError("phase profile pattern does not match this test")
    for key in ("frequency_hz", "x_amplitude_m", "z_amplitude_m"):
        if abs(float(profile.get(key, float("nan"))) - expected[key]) > 1e-6:
            raise RuntimeError("phase profile %s does not match this test" % key)
    profile_center = profile.get("center_xyz_m", [])
    if len(profile_center) != 3 or max(abs(float(profile_center[idx]) - expected["center_xyz_m"][idx]) for idx in range(3)) > 1e-6:
        raise RuntimeError("phase profile center does not match this test")
    coefficients = profile.get("coefficients_m", [])
    harmonics = int(profile.get("harmonics", -1))
    if harmonics < 0 or len(coefficients) != 1 + 2 * harmonics:
        raise RuntimeError("phase profile coefficients are invalid")
    args.phase_z_profile = profile


def _configure_phase_xz_profile(args, center):
    args.phase_xz_profile = None
    path = getattr(args, "phase_xz_compensation_profile", "")
    if not path:
        return
    if abs(float(args.z_feedforward_bias)) > 1e-9:
        raise RuntimeError("--phase-xz-compensation-profile already contains the full Z bias; use --z-feedforward-bias 0")
    if getattr(args, "phase_z_compensation_profile", ""):
        raise RuntimeError("Use either --phase-z-compensation-profile or --phase-xz-compensation-profile, not both")
    try:
        with open(path) as handle:
            profile = json.load(handle)
    except (OSError, ValueError) as exc:
        raise RuntimeError("Cannot load phase X/Z compensation profile %s: %s" % (path, exc))
    if profile.get("schema") != "so101_phase_xz_compensation_v1":
        raise RuntimeError("Unsupported phase X/Z compensation profile schema")
    expected = {
        "pattern": args.pattern,
        "frequency_hz": float(args.frequency),
        "center_xyz_m": [float(value) for value in center],
        "x_amplitude_m": float(args.x_amplitude if args.x_amplitude is not None else args.amplitude),
        "z_amplitude_m": float(args.z_amplitude if args.z_amplitude is not None else args.amplitude),
    }
    if profile.get("pattern") != expected["pattern"]:
        raise RuntimeError("phase X/Z profile pattern does not match this test")
    for key in ("frequency_hz", "x_amplitude_m", "z_amplitude_m"):
        if abs(float(profile.get(key, float("nan"))) - expected[key]) > 1e-6:
            raise RuntimeError("phase X/Z profile %s does not match this test" % key)
    profile_center = profile.get("center_xyz_m", [])
    if len(profile_center) != 3 or max(abs(float(profile_center[idx]) - expected["center_xyz_m"][idx]) for idx in range(3)) > 1e-6:
        raise RuntimeError("phase X/Z profile center does not match this test")
    harmonics = int(profile.get("harmonics", -1))
    coefficients_x = profile.get("coefficients_x_m", [])
    coefficients_z = profile.get("coefficients_z_m", [])
    expected_count = 1 + 2 * harmonics
    if harmonics < 0 or len(coefficients_x) != expected_count or len(coefficients_z) != expected_count:
        raise RuntimeError("phase X/Z profile coefficients are invalid")
    args.phase_xz_profile = profile


def _configure_compensation_profiles(args, center):
    _configure_phase_z_profile(args, center)
    _configure_phase_xz_profile(args, center)


def _joint_limit_margins(solution, limits, joint_names):
    """Return per-joint and minimum signed distance to configured limits."""
    margins = {}
    for name in joint_names:
        if name not in solution or name not in limits:
            continue
        lo, hi = limits[name]
        value = float(solution[name])
        margins[name] = min(value - float(lo), float(hi) - value)
    return margins, min(margins.values()) if margins else float("inf")


def _ramp_scale(elapsed, ramp_duration):
    if ramp_duration <= 0.0:
        return 1.0
    u = max(0.0, min(1.0, float(elapsed) / float(ramp_duration)))
    return (3.0 * u * u) - (2.0 * u * u * u)


def _servo_summary(servo_status):
    currents = []
    loads = []
    temps = []
    volts = []
    per_joint = {}
    for joint, data in servo_status.items():
        for key, out in (
            ("current_ma", currents),
            ("load_raw", loads),
            ("temperature_c", temps),
            ("voltage_v", volts),
        ):
            value = data.get(key, "")
            if value != "":
                try:
                    out.append(float(value))
                except (TypeError, ValueError):
                    pass
        prefix = "servo_%s_" % str(joint)
        for source_key, output_key in (
            ("current_ma", "current_ma"),
            ("load_raw", "load_raw"),
            ("voltage_v", "voltage_v"),
            ("temperature_c", "temperature_c"),
            ("velocity_raw", "velocity_raw"),
            ("moving", "moving"),
        ):
            value = data.get(source_key, "")
            if value != "":
                per_joint[prefix + output_key] = value
    summary = {
        "servo_max_current_ma": max(currents) if currents else "",
        "servo_max_abs_load_raw": max([abs(value) for value in loads]) if loads else "",
        "servo_max_temperature_c": max(temps) if temps else "",
        "servo_min_voltage_v": min(volts) if volts else "",
    }
    summary.update(per_joint)
    return summary


def _write_csv(path, rows):
    if not path:
        return
    keys = sorted({key for row in rows for key in row})
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _param_any(names, default=None):
    for name in names:
        if rospy.has_param(name):
            return rospy.get_param(name)
    return default


def _load_kinematics_context(args):
    urdf = rospy.get_param("/robot_description", "")
    if not urdf:
        raise RuntimeError("Missing /robot_description; start the SO101 bridge with with_description:=true")

    limits = _param_any(["~limits", "/so101_kinematics_node/limits", "/so101_driver_node/limits"], {})
    home_positions = _param_any(
        ["~home_positions", "/so101_kinematics_node/home_positions", "/so101_driver_node/home_positions"],
        {},
    )
    locked_joints = _param_any(
        ["~locked_joints", "/so101_kinematics_node/locked_joints", "/so101_driver_node/locked_joints"],
        {},
    )
    active_joints = _param_any(
        ["~ik_active_joints", "/so101_kinematics_node/ik_active_joints", "/so101_kinematics_node/active_joints", "/so101_driver_node/active_joints"],
        ["shoulder_lift", "elbow_flex", "wrist_flex"],
    )
    ik_tolerance_m = float(
        _param_any(["~ik_tolerance_m", "/so101_kinematics_node/ik_tolerance_m"], 0.002)
    )
    ik_command_tolerance_m = float(
        _param_any(["~ik_command_tolerance_m", "/so101_kinematics_node/ik_command_tolerance_m"], 0.008)
    )
    ik_max_iters = int(_param_any(["~ik_max_iters", "/so101_kinematics_node/ik_max_iters"], 200))
    kin = SO101Kinematics.from_urdf(
        urdf,
        base_link=args.frame,
        tip_link=_param_any(["~tip_link", "/so101_kinematics_node/tip_link"], "gripper_frame_link"),
        limits_override=limits,
    )
    return kin, active_joints, locked_joints, limits, home_positions, ik_tolerance_m, ik_command_tolerance_m, ik_max_iters


def _complete_joint_seed(home_positions, locked_joints, pose=None, current_joints=None):
    seed = {}
    for name in JOINT_ORDER:
        if name in home_positions:
            seed[name] = float(home_positions[name])
        elif current_joints and name in current_joints:
            seed[name] = float(current_joints[name])
        else:
            seed[name] = 0.0
    for name, value in (pose or {}).items():
        seed[name] = float(value)
    for name, value in (locked_joints or {}).items():
        seed[name] = float(value)
    return seed


def _validate_trajectory(args, cache):
    (
        kin,
        active_joints,
        locked_joints,
        limits,
        home_positions,
        ik_tolerance_m,
        ik_command_tolerance_m,
        ik_max_iters,
    ) = _load_kinematics_context(args)

    _, current_joints, _, _, _ = cache.snapshot()
    prep = {} if args.prep_pose == "none" else SAFE_POSES[args.prep_pose]
    seed = _complete_joint_seed(home_positions, locked_joints, prep, current_joints)
    if args.center is None:
        center = list(kin.fk(seed)[:3, 3])
        center_source = "prep_pose:%s" % args.prep_pose
    else:
        center = list(args.center)
        center_source = "cli"
    _configure_compensation_profiles(args, center)
    center_bias_scale = 0.0 if (args.phase_z_profile is not None or getattr(args, "phase_xz_profile", None) is not None) else 1.0
    command_center = _command_target(args, center, 0.0, center_bias_scale)
    validation_seed_joints = current_joints
    if args.prep_pose != "none":
        validation_seed_joints = _complete_joint_seed(
            home_positions,
            locked_joints,
            SAFE_POSES[args.prep_pose],
            current_joints=None,
        )
    center_solution, _center_err = _solve_center_pose(
        kin,
        active_joints,
        locked_joints,
        home_positions,
        validation_seed_joints,
        command_center,
        args,
    )

    rows = []
    ik_accepted_count = 0
    execution_accepted_count = 0
    converged_count = 0
    max_err = 0.0
    max_commanded_step = 0.0
    joint_min = {}
    joint_max = {}
    joint_min_margin = {}
    limit_margin_failures = []
    last_solution = dict(center_solution)
    samples = max(2, int(args.validate_samples))
    for sample_idx in range(samples):
        elapsed = args.duration * float(sample_idx) / float(samples - 1)
        ramp_scale = _ramp_scale(elapsed, args.ramp_duration)
        offset = [value * ramp_scale for value in _target_offset(args, elapsed)]
        target = [center[idx] + offset[idx] for idx in range(3)]
        command_target = _command_target(args, target, elapsed, ramp_scale)
        converged, solution, err, iters = kin.solve_ik_position(
            command_target,
            last_solution,
            active_joints,
            locked_joints=locked_joints,
            max_iters=ik_max_iters,
            tolerance=ik_tolerance_m,
            # Select a branch only at the first sample, then preserve the
            # continuous branch used by the timed trajectory executor.
            multi_start=(sample_idx == 0),
        )
        ik_accepted = bool(converged) or float(err) <= ik_command_tolerance_m
        margins, min_margin = _joint_limit_margins(solution, limits, active_joints)
        margin_ok = min_margin >= float(args.joint_limit_margin)
        accepted = ik_accepted and margin_ok
        if ik_accepted:
            ik_accepted_count += 1
        if accepted:
            execution_accepted_count += 1
            last_solution = dict(solution)
        elif ik_accepted and not margin_ok:
            limit_margin_failures.append((sample_idx, elapsed, target, min_margin, dict(solution)))
        if converged:
            converged_count += 1
        max_err = max(max_err, float(err))

        step = 0.0
        for name in active_joints:
            if name in solution:
                step = max(step, abs(float(solution[name]) - float(last_solution.get(name, solution[name]))))
                joint_min[name] = min(joint_min.get(name, float(solution[name])), float(solution[name]))
                joint_max[name] = max(joint_max.get(name, float(solution[name])), float(solution[name]))
                joint_min_margin[name] = min(joint_min_margin.get(name, float("inf")), margins.get(name, float("inf")))
        max_commanded_step = max(max_commanded_step, step)

        row = {
            "sample": sample_idx,
            "elapsed": elapsed,
            "pattern": args.pattern,
            "center_source": center_source,
            "center_x": center[0],
            "center_y": center[1],
            "center_z": center[2],
            "offset_x": offset[0],
            "offset_y": offset[1],
            "offset_z": offset[2],
            "ramp_scale": ramp_scale,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "command_target_x": command_target[0],
            "command_target_y": command_target[1],
            "command_target_z": command_target[2],
            "x_feedforward_bias": command_target[0] - target[0],
            "y_feedforward_bias": command_target[1] - target[1],
            "z_feedforward_bias": command_target[2] - target[2],
            "ik_converged": converged,
            "ik_commanded": ik_accepted,
            "execution_accepted": accepted,
            "joint_limit_margin_ok": margin_ok,
            "minimum_joint_limit_margin_rad": min_margin,
            "ik_approximate": bool((not converged) and ik_accepted),
            "ik_error_m": float(err),
            "iterations": iters,
        }
        for name in JOINT_ORDER:
            if name in solution:
                row[name] = solution[name]
            if name in margins:
                row[name + "_limit_margin_rad"] = margins[name]
        rows.append(row)

    _write_csv(args.csv, rows)
    print("SO101 EE trajectory validation")
    print("  pattern:              %s" % args.pattern)
    print("  center_source:        %s" % center_source)
    print("  center_xyz_m:         %.4f %.4f %.4f" % (center[0], center[1], center[2]))
    print("  samples:              %d" % samples)
    print("  ik_converged:         %d/%d" % (converged_count, samples))
    print("  ik_command_accepted:  %d/%d" % (ik_accepted_count, samples))
    print("  execution_accepted:   %d/%d" % (execution_accepted_count, samples))
    print("  max_ik_error_m:       %.6f" % max_err)
    print("  command_tolerance_m:  %.6f" % ik_command_tolerance_m)
    print("  phase_z_profile:      %s" % ("enabled" if args.phase_z_profile is not None else "disabled"))
    print("  phase_xz_profile:     %s" % ("enabled" if getattr(args, "phase_xz_profile", None) is not None else "disabled"))
    print("  joint_limit_margin_m: %.6f" % float(args.joint_limit_margin))
    print("  joint_ranges_rad:")
    for name in active_joints:
        if name in joint_min:
            lo, hi = limits.get(name, ["", ""])
            print(
                "    %-14s %.4f to %.4f  limit=[%s, %s]  min_margin=%.4f"
                % (name, joint_min[name], joint_max[name], lo, hi, joint_min_margin[name])
            )
    if args.csv:
        print("  csv:                  %s" % args.csv)
    if ik_accepted_count != samples:
        print("  verdict:              NOT SAFE TO RUN - some targets are not IK-accepted")
        return 2
    if execution_accepted_count != samples:
        print("  verdict:              NOT SAFE TO RUN - joint limit margin is insufficient")
        for sample_idx, elapsed, target, min_margin, solution in limit_margin_failures[:5]:
            print(
                "    sample=%d t=%.2fs target=(%.4f,%.4f,%.4f) min_margin=%.5frad "
                "q2/q3/q4=(%.4f,%.4f,%.4f)"
                % (
                    sample_idx,
                    elapsed,
                    target[0],
                    target[1],
                    target[2],
                    min_margin,
                    solution.get("shoulder_lift", 0.0),
                    solution.get("elbow_flex", 0.0),
                    solution.get("wrist_flex", 0.0),
                )
            )
        return 2
    print("  verdict:              PRECHECK PASSED; --validate-only did not command the hardware")
    return 0


def _commandable_ik_joints(solution, active_joints, locked_joints):
    return [
        name
        for name in active_joints
        if name in solution and name not in locked_joints
    ]


def _solve_center_pose(kin, active_joints, locked_joints, home_positions, current_joints, center, args):
    seed = _complete_joint_seed(home_positions, locked_joints, current_joints=current_joints)
    ok, solution, err, iters = kin.solve_ik_position(
        center,
        seed,
        active_joints,
        locked_joints=locked_joints,
        max_iters=args.ik_max_iters_override or int(_param_any(["/so101_kinematics_node/ik_max_iters"], 200)),
        tolerance=args.ik_tolerance_override or float(_param_any(["/so101_kinematics_node/ik_tolerance_m"], 0.002)),
    )
    command_tol = args.ik_command_tolerance_override or float(
        _param_any(["/so101_kinematics_node/ik_command_tolerance_m"], 0.008)
    )
    if not (ok or float(err) <= command_tol):
        raise RuntimeError("Center IK failed: err=%.4fm after %d iterations" % (err, iters))
    return solution, err


def _build_planned_joint_path(args, cache, center, seed_positions=None):
    (
        kin,
        active_joints,
        locked_joints,
        limits,
        home_positions,
        ik_tolerance_m,
        ik_command_tolerance_m,
        ik_max_iters,
    ) = _load_kinematics_context(args)
    if args.ik_tolerance_override is not None:
        ik_tolerance_m = float(args.ik_tolerance_override)
    if args.planning_ik_tolerance is not None:
        ik_tolerance_m = float(args.planning_ik_tolerance)
    if args.ik_command_tolerance_override is not None:
        ik_command_tolerance_m = float(args.ik_command_tolerance_override)
    if args.ik_max_iters_override is not None:
        ik_max_iters = int(args.ik_max_iters_override)

    if seed_positions is None:
        _, current_joints, _, _, _ = cache.snapshot()
        seed_positions = current_joints
    # The first path point is the already-commanded center. Reusing its IK
    # branch prevents a discontinuous branch change at t=0.
    seed = _complete_joint_seed(home_positions, locked_joints, current_joints=seed_positions)
    last_solution = dict(seed)
    timed_positions = []
    plan_rows = []
    failures = []
    accepted_count = 0
    converged_count = 0
    max_err = 0.0
    joint_min = {}
    joint_max = {}
    joint_min_margin = {}
    samples = max(2, int(math.floor(args.duration * args.rate)) + 1)
    command_names = None

    for idx in range(samples):
        elapsed = min(float(args.duration), float(idx) / float(args.rate))
        ramp_scale = _ramp_scale(elapsed, args.ramp_duration)
        offset = [value * ramp_scale for value in _target_offset(args, elapsed)]
        target = [center[axis] + offset[axis] for axis in range(3)]
        command_target = _command_target(args, target, elapsed, ramp_scale)
        converged, solution, err, iters = kin.solve_ik_position(
            command_target,
            last_solution,
            active_joints,
            locked_joints=locked_joints,
            max_iters=ik_max_iters,
            tolerance=ik_tolerance_m,
            multi_start=False,
        )
        ik_accepted = bool(converged) or float(err) <= ik_command_tolerance_m
        margins, min_margin = _joint_limit_margins(solution, limits, active_joints)
        margin_ok = min_margin >= float(args.joint_limit_margin)
        accepted = ik_accepted and margin_ok
        max_err = max(max_err, float(err))
        if not accepted:
            failures.append((idx, elapsed, target, err, iters, min_margin, margin_ok))
        else:
            accepted_count += 1
            last_solution = dict(solution)
        if converged:
            converged_count += 1
        names = _commandable_ik_joints(solution, active_joints, locked_joints)
        if command_names is None:
            command_names = names
        if command_names != names:
            raise RuntimeError("IK command joint set changed during plan: %s -> %s" % (command_names, names))
        point_positions = {name: float(solution[name]) for name in command_names}
        timed_positions.append((elapsed, point_positions))
        for name, value in point_positions.items():
            joint_min[name] = min(joint_min.get(name, value), value)
            joint_max[name] = max(joint_max.get(name, value), value)
            joint_min_margin[name] = min(joint_min_margin.get(name, float("inf")), margins.get(name, float("inf")))
        row = {
            "sample": idx,
            "elapsed": elapsed,
            "pattern": args.pattern,
            "execution_mode": "joint_trajectory",
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "command_target_x": command_target[0],
            "command_target_y": command_target[1],
            "command_target_z": command_target[2],
            "x_feedforward_bias": command_target[0] - target[0],
            "y_feedforward_bias": command_target[1] - target[1],
            "z_feedforward_bias": command_target[2] - target[2],
            "offset_x": offset[0],
            "offset_y": offset[1],
            "offset_z": offset[2],
            "ramp_scale": ramp_scale,
            "ik_converged": converged,
            "ik_commanded": ik_accepted,
            "execution_accepted": accepted,
            "joint_limit_margin_ok": margin_ok,
            "minimum_joint_limit_margin_rad": min_margin,
            "ik_approximate": bool((not converged) and ik_accepted),
            "ik_error_m": float(err),
            "iterations": iters,
        }
        for name in command_names:
            row["planned_" + name] = point_positions[name]
            row["planned_" + name + "_limit_margin_rad"] = margins.get(name, float("inf"))
        plan_rows.append(row)

    if not command_names:
        raise RuntimeError("No commandable IK joints in planned trajectory")
    if failures:
        preview = failures[:5]
        detail = "; ".join(
            "idx=%d t=%.2f target=(%.3f,%.3f,%.3f) err=%.4f iters=%d margin=%.4f"
            % (idx, elapsed, target[0], target[1], target[2], err, iters, min_margin)
            for idx, elapsed, target, err, iters, min_margin, _margin_ok in preview
        )
        raise RuntimeError(
            "Planned path has %d rejected points (IK or --joint-limit-margin=%.3frad): %s"
            % (len(failures), float(args.joint_limit_margin), detail)
        )

    print("SO101 planned joint trajectory")
    print("  samples:              %d" % samples)
    print("  rate_hz:              %.1f" % args.rate)
    print("  ik_converged:         %d/%d" % (converged_count, samples))
    print("  ik_command_accepted:  %d/%d" % (accepted_count, samples))
    print("  max_ik_error_m:       %.6f" % max_err)
    print("  planning_tolerance_m: %.6f" % ik_tolerance_m)
    print("  joint_limit_margin_m: %.6f" % float(args.joint_limit_margin))
    print("  command_joints:       %s" % ", ".join(command_names))
    print("  joint_ranges_rad:")
    for name in command_names:
        lo, hi = limits.get(name, ["", ""])
        print(
            "    %-14s %.4f to %.4f  limit=[%s, %s]  min_margin=%.4f"
            % (name, joint_min[name], joint_max[name], lo, hi, joint_min_margin[name])
        )
    return command_names, timed_positions, plan_rows


def _nearest_plan_row(plan_rows, elapsed):
    if not plan_rows:
        return {}
    idx = int(round(float(elapsed) / max(1e-6, plan_rows[1]["elapsed"] - plan_rows[0]["elapsed"]))) if len(plan_rows) > 1 else 0
    idx = max(0, min(idx, len(plan_rows) - 1))
    return plan_rows[idx]


def _record_runtime_row(args, cache, center, elapsed, plan_row=None):
    ramp_scale = _ramp_scale(elapsed, args.ramp_duration)
    offset = [value * ramp_scale for value in _target_offset(args, elapsed)]
    target = [center[idx] + offset[idx] for idx in range(3)]
    command_target = _command_target(args, target, elapsed, ramp_scale)
    ee, joints, commanded_joints, servo_status, kin_status = cache.snapshot()
    measured = _pose_xyz(ee) if ee is not None else ["", "", ""]
    row = {
        "t": time.time(),
        "elapsed": elapsed,
        "pattern": args.pattern,
        "execution_mode": args.execution_mode,
        "axis": args.axis,
        "offset_x": offset[0],
        "offset_y": offset[1],
        "offset_z": offset[2],
        "ramp_scale": ramp_scale,
        "target_x": target[0],
        "target_y": target[1],
        "target_z": target[2],
        "command_target_x": command_target[0],
        "command_target_y": command_target[1],
        "command_target_z": command_target[2],
        "x_feedforward_bias": command_target[0] - target[0],
        "y_feedforward_bias": command_target[1] - target[1],
        "z_feedforward_bias": command_target[2] - target[2],
        "measured_x": measured[0],
        "measured_y": measured[1],
        "measured_z": measured[2],
        "ik_ok": kin_status.get("ok", ""),
        "ik_commanded": kin_status.get("ik_commanded", kin_status.get("ok", "")),
        "ik_converged": kin_status.get("ik_converged", ""),
        "ik_approximate": kin_status.get("ik_approximate", ""),
        "ik_error_m": kin_status.get("position_error_m", ""),
        "kinematics_message": kin_status.get("message", ""),
    }
    if plan_row:
        for key, value in plan_row.items():
            if key.startswith("planned_") or key.startswith("command_target_") or key in ("ik_converged", "ik_commanded", "ik_approximate", "ik_error_m", "x_feedforward_bias", "y_feedforward_bias", "z_feedforward_bias"):
                row[key] = value
    if ee is not None:
        errors = [float(measured[idx]) - target[idx] for idx in range(3)]
        row.update(
            {
                "error_x": errors[0],
                "error_y": errors[1],
                "error_z": errors[2],
                "error_norm": math.sqrt(sum(error * error for error in errors)),
            }
        )
    for joint in JOINT_ORDER:
        if joint in joints:
            row[joint] = joints[joint]
        if joint in commanded_joints:
            row["commanded_" + joint] = commanded_joints[joint]
            if joint in joints:
                row["joint_tracking_error_" + joint] = float(joints[joint]) - float(commanded_joints[joint])
    row.update(_servo_summary(servo_status))
    return row


def _run_center_tracking_diagnostic(args, cache, center, command_center, center_pose):
    """Record static center tracking without ever starting the path."""
    rows = []
    rate = rospy.Rate(max(1.0, float(args.center_diagnostic_rate)))
    started = time.time()
    deadline = started + max(0.0, float(args.center_diagnostic_duration))
    while time.time() < deadline and not rospy.is_shutdown():
        ee, joints, commanded_joints, servo_status, _kin_status = cache.snapshot()
        measured = _pose_xyz(ee) if ee is not None else ["", "", ""]
        row = {
            "elapsed": max(0.0, time.time() - started),
            "mode": "center_diagnostic",
            "target_x": center[0],
            "target_y": center[1],
            "target_z": center[2],
            "command_target_x": command_center[0],
            "command_target_y": command_center[1],
            "command_target_z": command_center[2],
            "x_feedforward_bias": command_center[0] - center[0],
            "y_feedforward_bias": command_center[1] - center[1],
            "z_feedforward_bias": command_center[2] - center[2],
            "measured_x": measured[0],
            "measured_y": measured[1],
            "measured_z": measured[2],
        }
        if ee is not None:
            row["error_x"] = float(measured[0]) - center[0]
            row["error_y"] = float(measured[1]) - center[1]
            row["error_z"] = float(measured[2]) - center[2]
            row["error_norm"] = math.sqrt(sum(row["error_" + axis] ** 2 for axis in ("x", "y", "z")))
        for name, target in center_pose.items():
            row["target_" + name] = target
            if name in joints:
                row[name] = joints[name]
                row["joint_tracking_error_" + name] = float(joints[name]) - float(target)
            if name in commanded_joints:
                row["commanded_" + name] = commanded_joints[name]
        row.update(_servo_summary(servo_status))
        rows.append(row)
        rate.sleep()

    _write_csv(args.csv, rows)
    print("SO101 center tracking diagnostic complete")
    print("  samples:", len(rows))
    print("  csv:", args.csv)
    for name, target in center_pose.items():
        errors = [float(row["joint_tracking_error_" + name]) for row in rows if "joint_tracking_error_" + name in row]
        if errors:
            print("  %-14s target=% .4f mean_error=%+.4f max_abs_error=%.4f rad" % (name, target, statistics.fmean(errors), max(abs(value) for value in errors)))
    return 0


def _run_joint_trajectory_mode(args, cache, center):
    joint_pub = rospy.Publisher("/so101/command_joint_positions", JointTrajectory, queue_size=1)
    _wait_for_publisher_connection(joint_pub, args.timeout, "/so101/command_joint_positions")

    (
        kin,
        active_joints,
        locked_joints,
        _limits,
        home_positions,
        _ik_tolerance_m,
        _ik_command_tolerance_m,
        _ik_max_iters,
    ) = _load_kinematics_context(args)
    _, current_joints, _, _, _ = cache.snapshot()
    center_bias_scale = 0.0 if (args.phase_z_profile is not None or getattr(args, "phase_xz_profile", None) is not None) else 1.0
    command_center = _command_target(args, center, 0.0, center_bias_scale)
    center_solution, center_err = _solve_center_pose(
        kin, active_joints, locked_joints, home_positions, current_joints, command_center, args
    )
    center_names = _commandable_ik_joints(center_solution, active_joints, locked_joints)
    center_pose = {name: float(center_solution[name]) for name in center_names}
    if args.move_to_center_duration > 0.0:
        print(
            "Moving to planned trajectory center with joint-space minimum-jerk, "
            "center_ik_error_m=%.6f, x_feedforward_bias=%.4fm, z_feedforward_bias=%.4fm"
            % (center_err, command_center[0] - center[0], command_center[2] - center[2])
        )
        _publish_joint_pose(joint_pub, center_pose, args.move_to_center_duration)
        center_ready = _wait_joints_close(
            cache,
            center_pose,
            max(args.timeout, args.move_to_center_duration + 2.0),
            args.center_joint_tolerance,
        )
        if not center_ready:
            max_center_error, center_details = _joint_error_details(cache, center_pose)
            detail_text = _format_joint_error_details(center_details)
            message = (
                "SO101 center pre-position tracking error exceeds the requested %.3frad tolerance "
                "(max=%.4frad, abort threshold=%.4frad). This is not a joint-limit rejection.\n%s"
                % (
                    args.center_joint_tolerance,
                    max_center_error,
                    args.center_max_start_error,
                    detail_text,
                )
            )
            if not args.center_diagnostic_only and (args.strict_center or max_center_error > args.center_max_start_error):
                raise RuntimeError(
                    "%s\nThe path was not started. Do not enlarge joint limits to bypass this; "
                    "run the center diagnostic with outer PI disabled, then inspect shoulder tracking "
                    "and the synchronous-write status before changing any servo gain." % message
                )
            print(
                "Warning: %s\n"
                "%s"
                % (
                    message,
                    "Center diagnostic will record this static tracking error without starting the path."
                    if args.center_diagnostic_only
                    else "Continuing because max joint error is within --center-max-start-error=%.3frad."
                    % args.center_max_start_error,
                ),
                file=sys.stderr,
            )
        rospy.sleep(args.settle)

    if args.center_diagnostic_only:
        return _run_center_tracking_diagnostic(args, cache, center, command_center, center_pose)

    _check_synchronous_writes(cache, args)

    command_names, timed_positions, plan_rows = _build_planned_joint_path(
        args, cache, center, seed_positions=center_solution
    )
    _publish_joint_trajectory(joint_pub, command_names, timed_positions)

    rows = []
    rate = rospy.Rate(args.rate)
    start = time.time()
    while not rospy.is_shutdown():
        elapsed = time.time() - start
        if elapsed > args.duration:
            break
        rows.append(_record_runtime_row(args, cache, center, elapsed, _nearest_plan_row(plan_rows, elapsed)))
        rate.sleep()

    if args.return_center:
        _publish_joint_pose(joint_pub, center_pose, args.return_duration)
        rospy.sleep(max(0.0, args.return_duration))
    _write_csv(args.csv, rows)
    print("samples:", len(rows))
    print("csv:", args.csv)
    return 0


def run(args):
    if args.frequency <= 0.0:
        raise RuntimeError("--frequency must be > 0")
    if args.rate <= 0.0:
        raise RuntimeError("--rate must be > 0")
    if args.pattern == "axis" and args.amplitude <= 0.0:
        raise RuntimeError("--amplitude must be > 0")
    if args.pattern != "axis":
        x_amp = args.x_amplitude if args.x_amplitude is not None else args.amplitude
        y_amp = args.y_amplitude if args.y_amplitude is not None else 0.0
        z_amp = args.z_amplitude if args.z_amplitude is not None else args.amplitude
        if max(abs(x_amp), abs(y_amp), abs(z_amp)) <= 0.0:
            raise RuntimeError("At least one non-axis amplitude must be > 0")

    cache = RosCache()
    _latest_center(cache, args.timeout)
    if args.validate_only:
        return _validate_trajectory(args, cache)
    _check_driver_preflight(cache, args)
    _prepare_pose(cache, args)
    center_msg = _latest_center(cache, args.timeout)
    center = _pose_xyz(center_msg) if args.center is None else list(args.center)
    _configure_compensation_profiles(args, center)
    frame_id = center_msg.header.frame_id or args.frame
    orientation = center_msg.pose.orientation

    if args.pattern == "xz_sine":
        x_amp = args.x_amplitude if args.x_amplitude is not None else args.amplitude
        z_amp = args.z_amplitude if args.z_amplitude is not None else args.amplitude
        print(
            "SO101 EE trajectory pattern=xz_sine center=(%.4f, %.4f, %.4f) "
            "x_amp=%.4fm z_amp=%.4fm total_width=%.4fm total_height=%.4fm frequency=%.4fHz duration=%.2fs"
            % (
                center[0],
                center[1],
                center[2],
                x_amp,
                z_amp,
                2.0 * x_amp,
                2.0 * z_amp,
                args.frequency,
                args.duration,
            )
        )
    else:
        print(
            "SO101 EE trajectory pattern=%s axis=%s center=(%.4f, %.4f, %.4f) amplitude=%.4fm frequency=%.4fHz duration=%.2fs"
            % (args.pattern, args.axis, center[0], center[1], center[2], args.amplitude, args.frequency, args.duration)
        )
    if args.execution_mode == "joint_trajectory":
        return _run_joint_trajectory_mode(args, cache, center)

    pub = rospy.Publisher("/so101/cartesian_target", PoseStamped, queue_size=1)
    deadline = time.time() + args.timeout
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)

    _hold_cartesian_target(
        pub,
        cache,
        frame_id,
        center,
        orientation,
        args.move_to_center_duration,
        args.rate,
        args.center_tolerance,
    )

    rows = []
    rate = rospy.Rate(args.rate)
    start = time.time()
    while not rospy.is_shutdown():
        now = time.time()
        elapsed = now - start
        if elapsed > args.duration:
            break
        ramp_scale = _ramp_scale(elapsed, args.ramp_duration)
        offset = [value * ramp_scale for value in _target_offset(args, elapsed)]
        target = [center[idx] + offset[idx] for idx in range(3)]
        command_target = _command_target(args, target, elapsed, ramp_scale)
        _publish_target(pub, frame_id, command_target, orientation)

        ee, joints, commanded_joints, servo_status, kin_status = cache.snapshot()
        measured = _pose_xyz(ee) if ee is not None else ["", "", ""]
        row = {
            "t": now,
            "elapsed": elapsed,
            "pattern": args.pattern,
            "axis": args.axis,
            "offset_x": offset[0],
            "offset_y": offset[1],
            "offset_z": offset[2],
            "ramp_scale": ramp_scale,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "command_target_x": command_target[0],
            "command_target_y": command_target[1],
            "command_target_z": command_target[2],
            "x_feedforward_bias": command_target[0] - target[0],
            "y_feedforward_bias": command_target[1] - target[1],
            "z_feedforward_bias": command_target[2] - target[2],
            "measured_x": measured[0],
            "measured_y": measured[1],
            "measured_z": measured[2],
            "ik_ok": kin_status.get("ok", ""),
            "ik_commanded": kin_status.get("ik_commanded", kin_status.get("ok", "")),
            "ik_converged": kin_status.get("ik_converged", ""),
            "ik_approximate": kin_status.get("ik_approximate", ""),
            "ik_error_m": kin_status.get("position_error_m", ""),
            "kinematics_message": kin_status.get("message", ""),
        }
        if ee is not None:
            errors = [float(measured[idx]) - target[idx] for idx in range(3)]
            row.update(
                {
                    "error_x": errors[0],
                    "error_y": errors[1],
                    "error_z": errors[2],
                    "error_norm": math.sqrt(sum(error * error for error in errors)),
                }
            )
        for joint in JOINT_ORDER:
            if joint in joints:
                row[joint] = joints[joint]
            if joint in commanded_joints:
                row["commanded_" + joint] = commanded_joints[joint]
                if joint in joints:
                    row["joint_tracking_error_" + joint] = float(joints[joint]) - float(commanded_joints[joint])
        row.update(_servo_summary(servo_status))
        rows.append(row)
        rate.sleep()

    if args.return_center:
        for _ in range(max(1, int(args.return_duration * args.rate))):
            _publish_target(pub, frame_id, center, orientation)
            rospy.sleep(1.0 / args.rate)
    _write_csv(args.csv, rows)
    print("samples:", len(rows))
    print("csv:", args.csv)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="SO101 Cartesian end-effector sine tracking test")
    parser.add_argument(
        "--pattern",
        choices=["axis", "xz_sine", "xz_edge_vertex8", "xz_vertex_diamond", "xz_zigzag", "circle", "figure8", "lissajous"],
        default="figure8",
    )
    parser.add_argument("--axis", choices=["x", "y", "z"], default="z")
    parser.add_argument("--center", nargs=3, type=float, default=None, help="Absolute center xyz in base_link; default: current EE pose")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--amplitude", type=float, default=0.02, help="Metres")
    parser.add_argument("--x-amplitude", type=float, default=None, help="Metres for non-axis patterns; default: --amplitude")
    parser.add_argument("--y-amplitude", type=float, default=None, help="Metres for non-axis patterns; default: 0.0 because shoulder_pan is locked")
    parser.add_argument("--z-amplitude", type=float, default=None, help="Metres for non-axis patterns; default: --amplitude")
    parser.add_argument("--frequency", type=float, default=0.03)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--csv", default="/tmp/so101_ee_sine.csv")
    parser.add_argument(
        "--execution-mode",
        choices=["joint_trajectory", "cartesian_stream"],
        default="joint_trajectory",
        help="joint_trajectory precomputes IK and sends one timed trajectory; cartesian_stream publishes live Cartesian targets",
    )
    parser.add_argument("--validate-only", action="store_true", help="Check IK/limits and write CSV without moving the arm")
    parser.add_argument("--validate-samples", type=int, default=180, help="Samples for --validate-only")
    parser.add_argument("--ik-tolerance-override", type=float, default=None, help="Override IK convergence tolerance in metres")
    parser.add_argument("--ik-command-tolerance-override", type=float, default=None, help="Override command acceptance tolerance in metres")
    parser.add_argument("--ik-max-iters-override", type=int, default=None, help="Override IK iteration limit")
    parser.add_argument("--planning-ik-tolerance", type=float, default=0.00005, help="IK tolerance for precomputed joint_trajectory mode, in metres")
    parser.add_argument("--z-feedforward-bias", type=float, default=0.0, help="Metres added to command target Z for static gravity/FK bias compensation")
    parser.add_argument("--phase-z-compensation-profile", default="", help="Bench-only Fourier Z bias profile produced by so101_fit_phase_z_compensation.py")
    parser.add_argument("--phase-xz-compensation-profile", default="", help="Bench-only Fourier X/Z bias profile produced by so101_fit_phase_xz_compensation.py")
    parser.add_argument(
        "--joint-limit-margin",
        type=float,
        default=0.05,
        help="Required distance from every active joint hard limit in radians; set to 0 only for offline diagnosis",
    )
    parser.add_argument("--move-to-center-duration", type=float, default=3.0, help="Seconds to hold the Cartesian center before starting the trajectory")
    parser.add_argument("--center-tolerance", type=float, default=0.020, help="Warning threshold in metres after moving to center")
    parser.add_argument("--center-joint-tolerance", type=float, default=0.03, help="Joint tolerance in radians before planned trajectory starts")
    parser.add_argument(
        "--center-max-start-error",
        type=float,
        default=0.04,
        help="Abort planned trajectory if center joint error exceeds this value in radians",
    )
    parser.add_argument("--strict-center", action="store_true", help="Abort if center does not meet --center-joint-tolerance exactly")
    parser.add_argument("--center-diagnostic-only", action="store_true", help="Record static center tracking and exit without starting the path")
    parser.add_argument("--center-diagnostic-duration", type=float, default=15.0, help="Seconds to record when --center-diagnostic-only is set")
    parser.add_argument("--center-diagnostic-rate", type=float, default=20.0, help="Hz for --center-diagnostic-only CSV sampling")
    parser.add_argument(
        "--allow-feedback-position-assist",
        action="store_true",
        help="Allow an enabled ROS outer P/PI loop. Diagnostic use only; unsafe for baseline tracking tests.",
    )
    parser.add_argument(
        "--allow-sequential-writes",
        action="store_true",
        help="Allow Feetech sequential per-servo writes. Diagnostic use only; do not use for Cartesian paths.",
    )
    parser.add_argument("--ramp-duration", type=float, default=5.0, help="Seconds to smoothly ramp trajectory amplitude from 0 to the requested amplitude")
    parser.add_argument("--prep-pose", choices=["none"] + sorted(SAFE_POSES), default="reach")
    parser.add_argument("--prep-duration", type=float, default=2.5)
    parser.add_argument("--prep-tolerance", type=float, default=0.08)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--return-center", action="store_true", default=True)
    parser.add_argument("--no-return-center", dest="return_center", action="store_false")
    parser.add_argument("--return-duration", type=float, default=2.0)
    return parser


def main():
    rospy.init_node("so101_ee_sine_test", anonymous=True)
    try:
        return run(build_parser().parse_args())
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 EE sine test error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

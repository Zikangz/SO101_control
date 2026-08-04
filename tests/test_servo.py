#!/usr/bin/env python3
"""Dependency-light tests for the SO101 Cartesian servo path.

Runnable two ways:
  * with pytest:   pytest tests/test_servo.py
  * standalone:    python tests/test_servo.py   (prints PASS/FAIL, exit code)

These cover the ROS-free servo building blocks so they can run in any Python
environment with numpy (ruckig optional -- the Ruckig-specific test skips when
it is not importable). They do not require ROS.
"""

import math
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG_SRC = os.path.join(ROOT, "ros1_ws", "src", "so101_ros1_bridge", "src")
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.control import JointSafetyFilter
from so101_ros1_bridge.kinematics import SO101Kinematics
from so101_ros1_bridge.servo import (
    PlanarCartesianServo,
    RuckigJointLimiter,
    SimpleJointLimiter,
    make_joint_limiter,
)

URDF_PATH = os.path.join(ROOT, "assets", "so101", "so101_new_calib.urdf")
ACTIVE = ["shoulder_lift", "elbow_flex", "wrist_flex"]
LOCKED = {"shoulder_pan": 0.0, "wrist_roll": 0.0, "gripper": 0.0}
LIMITS = {"shoulder_lift": [-0.6, 0.6], "elbow_flex": [-0.8, 0.9], "wrist_flex": [-0.6, 0.6]}
MAX_VEL = {"shoulder_lift": 0.5, "elbow_flex": 0.55, "wrist_flex": 0.6}
WORKSPACE = {"x": [0.07, 0.45], "y": [-0.25, 0.25], "z": [0.0, 0.38]}


def _kin():
    with open(URDF_PATH) as handle:
        urdf = handle.read()
    return SO101Kinematics.from_urdf(
        urdf, base_link="base_link", tip_link="gripper_frame_link", limits_override=LIMITS
    )


def _servo(prefer_ruckig=True):
    return PlanarCartesianServo(
        _kin(), ACTIVE, LOCKED, LIMITS, MAX_VEL,
        workspace_limits=WORKSPACE, axes=(0, 2), position_gain=6.0,
        max_ee_speed=0.25, control_dt=0.01, prefer_ruckig=prefer_ruckig,
    )


# --- kinematics -----------------------------------------------------------

def test_position_jacobian_planar_y_row_zero():
    """With pan locked the y-row of the position Jacobian must be ~0."""
    kin = _kin()
    seed = {n: 0.0 for n in kin.chain_joint_names}
    jac, cur, active = kin.position_jacobian(seed, ACTIVE, LOCKED)
    assert active == ACTIVE
    assert jac.shape == (3, 3)
    # y-coupling is zero analytically; residual is finite-difference noise on
    # the order of the step size (eps=1e-4), so allow a small tolerance.
    assert np.allclose(jac[1, :], 0.0, atol=1e-5), jac[1, :]


def test_resolve_ee_velocity_matches_jacobian():
    """qdot from resolve_ee_velocity should reproduce the requested planar v."""
    kin = _kin()
    seed = {"shoulder_lift": 0.1, "elbow_flex": -0.2, "wrist_flex": 0.05}
    v_des = [0.05, 0.0, -0.03]  # x-z plane
    qdot = kin.resolve_ee_velocity(seed, v_des, ACTIVE, LOCKED, damping=0.01, axes=(0, 2))
    jac, _cur, active = kin.position_jacobian(seed, ACTIVE, LOCKED)
    qd = np.array([qdot[n] for n in active])
    v_reconstructed = jac[[0, 2], :] @ qd
    assert np.allclose(v_reconstructed, [0.05, -0.03], atol=5e-3), v_reconstructed


def test_resolve_ee_velocity_zero_input():
    kin = _kin()
    seed = {n: 0.0 for n in kin.chain_joint_names}
    qdot = kin.resolve_ee_velocity(seed, [0.0, 0.0, 0.0], ACTIVE, LOCKED, axes=(0, 2))
    assert all(abs(v) < 1e-9 for v in qdot.values())


# --- limiters -------------------------------------------------------------

def test_simple_limiter_respects_velocity_and_accel():
    lim = SimpleJointLimiter(["a"], {"a": 0.5}, {"a": 2.0}, limits={"a": [-10, 10]})
    lim.reset({"a": 0.0})
    dt = 0.01
    prev_v = 0.0
    for _ in range(500):
        pos, vel = lim.update({"a": 100.0}, None, dt)  # huge target -> saturate
        assert abs(vel["a"]) <= 0.5 + 1e-9
        assert abs(vel["a"] - prev_v) <= 2.0 * dt + 1e-9
        prev_v = vel["a"]
    assert abs(vel["a"] - 0.5) < 1e-6  # reaches vmax


def test_simple_limiter_clips_to_joint_limits():
    lim = SimpleJointLimiter(["a"], {"a": 5.0}, {"a": 1e9}, limits={"a": [-0.2, 0.2]})
    lim.reset({"a": 0.0})
    for _ in range(200):
        pos, vel = lim.update({"a": 100.0}, None, 0.01)
    assert pos["a"] <= 0.2 + 1e-9


def test_ruckig_limiter_tracks_velocity():
    try:
        lim = RuckigJointLimiter(["a"], {"a": 0.5}, {"a": 3.0}, {"a": 30.0}, 0.01)
    except Exception:
        print("  (skipped: ruckig not importable)")
        return
    lim.reset({"a": 0.0})
    vel = 0.0
    prev_v = 0.0
    for _ in range(200):
        pos, out = lim.update({"a": 0.0}, {"a": 0.3}, 0.01)  # velocity interface
        vel = out["a"]
        assert abs(vel) <= 0.5 + 1e-6
        prev_v = vel
    assert abs(vel - 0.3) < 1e-2, vel


def test_make_joint_limiter_returns_working_limiter():
    lim = make_joint_limiter(["a"], {"a": 0.5}, {"a": 3.0}, {"a": 30.0}, 0.01)
    lim.reset({"a": 0.0})
    pos, vel = lim.update({"a": 0.1}, {"a": 0.1}, 0.01)
    assert "a" in pos and "a" in vel


# --- servo law ------------------------------------------------------------

def _converge(servo, target, steps=500, dt=0.01):
    measured = {n: 0.0 for n in servo.kin.chain_joint_names}
    servo.reset(measured)
    for _ in range(steps):
        servo.step(dt, measured_positions=measured, position_target=target)
        measured = dict(servo.q_cmd)  # perfect position follower
    return servo.status()


def test_servo_position_converges_ruckig():
    servo = _servo(prefer_ruckig=True)
    p0 = servo.kin.fk_position({n: 0.0 for n in servo.kin.chain_joint_names})
    target = [float(p0[0]) + 0.02, float(p0[1]), float(p0[2]) - 0.03]
    st = _converge(servo, target)
    assert st["limiter"] == "RuckigJointLimiter"
    assert st["cartesian_error_m"] < 1e-3, st["cartesian_error_m"]


def test_servo_position_converges_simple():
    servo = _servo(prefer_ruckig=False)
    p0 = servo.kin.fk_position({n: 0.0 for n in servo.kin.chain_joint_names})
    target = [float(p0[0]) - 0.02, float(p0[1]), float(p0[2]) + 0.02]
    st = _converge(servo, target)
    assert st["limiter"] == "SimpleJointLimiter"
    assert st["cartesian_error_m"] < 1e-3, st["cartesian_error_m"]


def test_servo_velocity_moves_in_commanded_direction():
    servo = _servo(prefer_ruckig=True)
    measured = {n: 0.0 for n in servo.kin.chain_joint_names}
    servo.reset(measured)
    p0 = list(servo.kin.fk_position(servo.q_cmd))
    for _ in range(100):  # 5 cm/s +x for 1 s
        servo.step(0.01, measured_positions=measured, velocity_cmd=[0.05, 0.0, 0.0])
        measured = dict(servo.q_cmd)
    dx = servo.status()["ee_xyz"][0] - p0[0]
    assert dx > 0.02, dx  # moved forward (below 0.05 due to jerk ramp-up)


def test_servo_respects_workspace_limit():
    servo = _servo(prefer_ruckig=True)
    measured = {n: 0.0 for n in servo.kin.chain_joint_names}
    servo.reset(measured)
    # Command a target far outside the workspace in +x; p_cmd must be clamped.
    for _ in range(300):
        servo.step(0.01, measured_positions=measured, position_target=[10.0, 0.0, 0.2])
        measured = dict(servo.q_cmd)
    assert servo.status()["p_cmd"][0] <= WORKSPACE["x"][1] + 1e-9


# --- safety filter servo mode --------------------------------------------

class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _filter(clock):
    joint_order = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    home = {n: 0.0 for n in joint_order}
    home["gripper"] = 0.5
    limits = {n: [-1.5, 1.5] for n in joint_order}
    limits["gripper"] = [0.0, 1.0]
    max_vel = {n: 1.0 for n in joint_order}
    return JointSafetyFilter(
        joint_order, ACTIVE, {"shoulder_pan": 0.0, "wrist_roll": 0.0},
        limits, max_vel, home, clock=clock,
    )


def test_set_servo_target_no_minimum_jerk_restart():
    """set_servo_target must not start a timed/minimum-jerk trajectory."""
    clock = _Clock()
    filt = _filter(clock)
    filt.freeze({n: 0.0 for n in filt.joint_order})
    filt.set_servo_target({"shoulder_lift": 0.4})
    assert filt.timed_trajectory is None
    assert filt.trajectory_duration == 0.0
    assert not filt.trajectory_active()
    assert abs(filt.target["shoulder_lift"] - 0.4) < 1e-9


def test_set_servo_target_velocity_limited_tracking():
    """step() should velocity-limit toward a continuously moving servo target."""
    clock = _Clock()
    filt = _filter(clock)
    filt.freeze({n: 0.0 for n in filt.joint_order})
    dt = 0.01
    # Ramp the target upward each tick; commanded must trail by <= max_vel*dt.
    prev = 0.0
    for i in range(200):
        clock.t += dt
        filt.set_servo_target({"shoulder_lift": min(0.5, 0.005 * i)})
        cmd = filt.step(dt)
        step = cmd["shoulder_lift"] - prev
        assert step <= 1.0 * dt + 1e-9  # respects max_velocity
        prev = cmd["shoulder_lift"]
    assert prev > 0.3  # made real progress toward the target


def test_set_servo_target_respects_locked_joints():
    clock = _Clock()
    filt = _filter(clock)
    filt.freeze({n: 0.0 for n in filt.joint_order})
    filt.set_servo_target({"shoulder_pan": 1.0, "shoulder_lift": 0.3})
    assert abs(filt.target["shoulder_pan"]) < 1e-9  # locked -> unchanged
    assert abs(filt.target["shoulder_lift"] - 0.3) < 1e-9


def _run_standalone():
    tests = [obj for name, obj in sorted(globals().items()) if name.startswith("test_") and callable(obj)]
    failures = 0
    for fn in tests:
        try:
            fn()
            print("PASS", fn.__name__)
        except AssertionError as exc:
            failures += 1
            print("FAIL", fn.__name__, "->", exc)
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print("ERROR", fn.__name__, "->", repr(exc))
    print("\n%d/%d passed" % (len(tests) - failures, len(tests)))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_run_standalone())

"""ROS-free building blocks for the SO101 real-time Cartesian servo.

This module intentionally has no ROS dependency so the exact same servo law can
be exercised by:

* the ROS1 node ``so101_servo_node.py`` (real hardware / mujoco bridge), and
* the standalone MuJoCo physics simulation ``mujoco_planar_control_sim.py``.

It provides:

* :class:`RuckigJointLimiter` -- online (streaming) jerk-limited smoothing using
  the Python ``ruckig`` library, one ``update`` per control tick toward a moving
  target.  This is the same library MoveIt uses for online trajectory
  generation, but here it runs in the closed servo loop instead of retiming a
  precomputed waypoint path.
* :class:`SimpleJointLimiter` -- a dependency-free fallback that clamps joint
  velocity and acceleration.
* :func:`make_joint_limiter` -- Ruckig-first factory that falls back to the
  simple limiter when ``ruckig`` is not importable.
* :class:`PlanarCartesianServo` -- the resolved-rate servo law: turn a Cartesian
  position target or end-effector velocity command into a smooth joint stream.

"""

import math

__all__ = [
    "RuckigJointLimiter",
    "SimpleJointLimiter",
    "make_joint_limiter",
    "PlanarCartesianServo",
]


def _clip(value, lo, hi):
    return max(float(lo), min(float(hi), float(value)))


class SimpleJointLimiter:
    """Dependency-free velocity + acceleration limiter.

    Each ``update`` clamps the requested per-joint velocity to ``max_velocity``
    and the change in velocity to ``max_acceleration * dt``, then integrates to
    a new position and clips it to the joint limits.  It does not bound jerk, so
    motion is C1 (velocity continuous) but not C2.  Use :class:`RuckigJointLimiter`
    when ``ruckig`` is available for jerk-limited (C2) motion.
    """

    def __init__(self, joint_names, max_velocity, max_acceleration, limits=None):
        self.joint_names = list(joint_names)
        self.max_velocity = {n: abs(float(max_velocity.get(n, math.inf))) for n in self.joint_names}
        self.max_acceleration = {
            n: abs(float(max_acceleration.get(n, math.inf))) for n in self.joint_names
        }
        self.limits = {n: tuple(v) for n, v in (limits or {}).items()}
        self.position = {n: 0.0 for n in self.joint_names}
        self.velocity = {n: 0.0 for n in self.joint_names}

    def reset(self, positions, velocities=None):
        self.position = {n: float(positions.get(n, 0.0)) for n in self.joint_names}
        velocities = velocities or {}
        self.velocity = {n: float(velocities.get(n, 0.0)) for n in self.joint_names}

    def update(self, target_positions, target_velocities=None, dt=0.01):
        dt = max(1e-4, float(dt))
        target_velocities = target_velocities or {}
        out_pos = {}
        out_vel = {}
        for name in self.joint_names:
            vmax = self.max_velocity.get(name, math.inf)
            amax = self.max_acceleration.get(name, math.inf)
            cur_p = self.position.get(name, 0.0)
            cur_v = self.velocity.get(name, 0.0)
            target_p = float(target_positions.get(name, cur_p))
            # Prefer an explicit target velocity; otherwise derive one from the
            # position error so the joint decelerates as it reaches target.
            if name in target_velocities:
                desired_v = float(target_velocities[name])
            else:
                desired_v = (target_p - cur_p) / dt
            desired_v = _clip(desired_v, -vmax, vmax)
            dv = desired_v - cur_v
            max_dv = amax * dt
            if not math.isinf(max_dv) and abs(dv) > max_dv:
                dv = math.copysign(max_dv, dv)
            new_v = cur_v + dv
            new_p = cur_p + new_v * dt
            if name in self.limits:
                lo, hi = self.limits[name]
                clipped = _clip(new_p, lo, hi)
                if clipped != new_p:
                    new_v = 0.0
                new_p = clipped
            self.position[name] = new_p
            self.velocity[name] = new_v
            out_pos[name] = new_p
            out_vel[name] = new_v
        return out_pos, out_vel


class RuckigJointLimiter:
    """Online jerk-limited smoothing of a moving joint target using Ruckig.

    This runs Ruckig in its streaming mode: one ``otg.update`` per control tick
    toward the current (possibly moving) target, carrying integrator state with
    ``out.pass_to_input(inp)``.  That differs from the offline waypoint retiming
    in ``scripts/mujoco_planar_control_sim.py`` (which loops ``update`` to
    ``Result.Finished`` for each fixed waypoint).  Here the target changes every
    frame, so we never loop to completion; we just take one jerk-limited step.
    """

    def __init__(
        self,
        joint_names,
        max_velocity,
        max_acceleration,
        max_jerk,
        dt,
        limits=None,
    ):
        from ruckig import ControlInterface, InputParameter, OutputParameter, Ruckig

        self.joint_names = list(joint_names)
        self.dofs = len(self.joint_names)
        self.dt = max(1e-4, float(dt))
        self.limits = {n: tuple(v) for n, v in (limits or {}).items()}
        self._otg = Ruckig(self.dofs, self.dt)
        self._inp = InputParameter(self.dofs)
        self._out = OutputParameter(self.dofs)
        # Velocity control interface: the servo law supplies the position
        # feedback (as a target velocity); Ruckig only enforces velocity,
        # acceleration and jerk continuity.  Using the position interface with a
        # one-tick-ahead target is inconsistent for a moving setpoint and makes
        # Ruckig plan to stop at that near point, which reverses the motion.
        self._inp.control_interface = ControlInterface.Velocity
        self._vmax = [max(1e-3, abs(float(max_velocity.get(n, 1.0)))) for n in self.joint_names]
        self._inp.max_velocity = list(self._vmax)
        self._inp.max_acceleration = [
            max(1e-3, abs(float(max_acceleration.get(n, 1.0)))) for n in self.joint_names
        ]
        self._inp.max_jerk = [
            max(1e-2, abs(float(max_jerk.get(n, 1.0)))) for n in self.joint_names
        ]
        self._inp.current_position = [0.0] * self.dofs
        self._inp.current_velocity = [0.0] * self.dofs
        self._inp.current_acceleration = [0.0] * self.dofs
        self._inp.target_velocity = [0.0] * self.dofs
        self._inp.target_acceleration = [0.0] * self.dofs

    def reset(self, positions, velocities=None):
        velocities = velocities or {}
        self._inp.current_position = [float(positions.get(n, 0.0)) for n in self.joint_names]
        self._inp.current_velocity = [float(velocities.get(n, 0.0)) for n in self.joint_names]
        self._inp.current_acceleration = [0.0] * self.dofs
        self._inp.target_velocity = [0.0] * self.dofs
        self._inp.target_acceleration = [0.0] * self.dofs

    def update(self, target_positions, target_velocities=None, dt=None):
        # dt is accepted for a uniform limiter interface; Ruckig uses the fixed
        # control period it was constructed with.  target_positions is used only
        # to clamp against joint limits (velocity interface has no target pos).
        target_velocities = target_velocities or {}
        target_positions = target_positions or {}
        target_vel = []
        for idx, name in enumerate(self.joint_names):
            v = float(target_velocities.get(name, 0.0))
            vmax = self._vmax[idx]
            # Decelerate before hitting a joint limit: zero the target velocity
            # when it points into a limit the joint is already at.
            if name in self.limits:
                lo, hi = self.limits[name]
                pos = float(self._inp.current_position[idx])
                if (pos >= hi and v > 0.0) or (pos <= lo and v < 0.0):
                    v = 0.0
            target_vel.append(_clip(v, -vmax, vmax))
        self._inp.target_velocity = target_vel
        self._inp.target_acceleration = [0.0] * self.dofs

        self._otg.update(self._inp, self._out)
        self._out.pass_to_input(self._inp)
        # Hard-clip integrated position to joint limits and reflect it back into
        # the integrator state so Ruckig cannot wind past a limit.
        out_pos = {}
        out_vel = {}
        for idx, name in enumerate(self.joint_names):
            p = float(self._inp.current_position[idx])
            v = float(self._inp.current_velocity[idx])
            if name in self.limits:
                lo, hi = self.limits[name]
                clipped = _clip(p, lo, hi)
                if clipped != p:
                    p = clipped
                    v = 0.0
                    self._inp.current_position[idx] = p
                    self._inp.current_velocity[idx] = 0.0
            out_pos[name] = p
            out_vel[name] = v
        return out_pos, out_vel


def make_joint_limiter(
    joint_names,
    max_velocity,
    max_acceleration,
    max_jerk,
    dt,
    limits=None,
    prefer_ruckig=True,
):
    """Ruckig-first limiter factory.

    Returns a :class:`RuckigJointLimiter` when ``ruckig`` imports successfully;
    otherwise falls back to :class:`SimpleJointLimiter`.  The returned object
    always exposes ``reset(positions, velocities=None)`` and
    ``update(target_positions, target_velocities=None, dt=...)``.
    """
    if prefer_ruckig:
        try:
            return RuckigJointLimiter(
                joint_names,
                max_velocity,
                max_acceleration,
                max_jerk,
                dt,
                limits=limits,
            )
        except Exception:
            pass
    return SimpleJointLimiter(joint_names, max_velocity, max_acceleration, limits=limits)


class PlanarCartesianServo:
    """Resolved-rate Cartesian servo for the planar SO101.

    Both control inputs are unified through an internal Cartesian setpoint
    ``p_cmd`` so a single control law drives the arm:

    * position target -> ``p_cmd`` snaps to the (workspace-clamped) target.
    * velocity command -> ``p_cmd`` integrates the command (clamped to the
      workspace) and the command is also fed forward.

    Each :meth:`step` computes ``v_des = clamp(v_ff + Kp * (p_cmd - fk(q_cmd)))``,
    maps it to joint velocities with the kinematics DLS Jacobian, integrates to a
    joint setpoint, and passes that through the jerk/velocity limiter.  The servo
    keeps its own commanded joint vector ``q_cmd`` for FK/Jacobian evaluation so
    downstream position-servo dynamics (Feetech PID, MuJoCo actuators) do not
    feed measurement noise back into the Jacobian.  An optional slow resync to
    the measured joints bounds long-term drift.
    """

    def __init__(
        self,
        kinematics,
        active_joints,
        locked_joints,
        limits,
        max_velocity,
        workspace_limits=None,
        axes=(0, 2),
        position_gain=6.0,
        max_ee_speed=0.25,
        ik_damping=0.06,
        accel_scale=6.0,
        jerk_scale=12.0,
        control_dt=0.01,
        resync_gain=0.0,
        prefer_ruckig=True,
    ):
        self.kin = kinematics
        self.locked_joints = dict(locked_joints or {})
        self.limits = {n: tuple(v) for n, v in (limits or {}).items()}
        self.axes = tuple(int(a) for a in axes)
        self.position_gain = float(position_gain)
        self.max_ee_speed = float(max_ee_speed)
        self.ik_damping = float(ik_damping)
        self.control_dt = max(1e-4, float(control_dt))
        self.resync_gain = max(0.0, float(resync_gain))
        self.workspace_limits = dict(workspace_limits or {})

        # Commandable joints: active, in the kinematic chain, not locked.
        self.command_joints = [
            n
            for n in active_joints
            if n in self.kin.chain_joint_names and n not in self.locked_joints
        ]
        max_vel = {n: abs(float(max_velocity.get(n, 1.0))) for n in self.command_joints}
        max_acc = {n: max(1e-3, max_vel[n] * float(accel_scale)) for n in self.command_joints}
        max_jerk = {n: max(1e-2, max_acc[n] * float(jerk_scale)) for n in self.command_joints}
        self.limiter = make_joint_limiter(
            self.command_joints,
            max_vel,
            max_acc,
            max_jerk,
            self.control_dt,
            limits=self.limits,
            prefer_ruckig=prefer_ruckig,
        )
        self.q_cmd = {}
        self.p_cmd = None
        self._last_status = {}

    def _clip_joint(self, name, value):
        lo, hi = self.limits.get(name, (-math.inf, math.inf))
        return _clip(value, lo, hi)

    def _clamp_workspace(self, xyz):
        out = list(float(v) for v in xyz)
        for idx, axis in enumerate(("x", "y", "z")):
            limit = self.workspace_limits.get(axis)
            if isinstance(limit, (list, tuple)) and len(limit) == 2:
                out[idx] = _clip(out[idx], limit[0], limit[1])
        return out

    def reset(self, measured_positions):
        """Seed the servo from the measured joint state."""
        self.q_cmd = {n: float(measured_positions.get(n, 0.0)) for n in self.kin.chain_joint_names}
        for name, value in self.locked_joints.items():
            if name in self.q_cmd:
                self.q_cmd[name] = float(value)
        self.p_cmd = list(self.kin.fk_position(self.q_cmd))
        self.limiter.reset({n: self.q_cmd.get(n, 0.0) for n in self.command_joints})
        self._last_status = {}
        return dict(self.q_cmd)

    def step(self, dt, measured_positions=None, position_target=None, velocity_cmd=None):
        """Advance one control tick and return commanded joint positions.

        Exactly one of ``position_target`` (xyz) or ``velocity_cmd`` (xyz) is the
        active input; velocity takes priority when both are given.  When both are
        ``None`` the servo holds its current Cartesian setpoint.
        """
        dt = max(1e-4, float(dt))
        if self.p_cmd is None:
            self.reset(measured_positions or {})

        # Optional slow resync of the internal commanded joints to measured.
        if measured_positions and self.resync_gain > 0.0:
            beta = min(1.0, self.resync_gain * dt)
            for name in self.command_joints:
                if name in measured_positions:
                    self.q_cmd[name] += beta * (float(measured_positions[name]) - self.q_cmd[name])

        current_xyz = self.kin.fk_position(self.q_cmd)

        v_ff = [0.0, 0.0, 0.0]
        if velocity_cmd is not None:
            for idx in range(3):
                v_ff[idx] = float(velocity_cmd[idx])
            self.p_cmd = self._clamp_workspace(
                [self.p_cmd[i] + v_ff[i] * dt for i in range(3)]
            )
        elif position_target is not None:
            self.p_cmd = self._clamp_workspace(position_target)

        # v_des = feed-forward + proportional pull toward the Cartesian setpoint.
        v_des = [0.0, 0.0, 0.0]
        for idx in range(3):
            err = float(self.p_cmd[idx]) - float(current_xyz[idx])
            v_des[idx] = v_ff[idx] + self.position_gain * err
        # Cap Cartesian speed over the servoed axes.
        speed = math.sqrt(sum(v_des[a] ** 2 for a in self.axes))
        if speed > self.max_ee_speed and speed > 1e-9:
            scale = self.max_ee_speed / speed
            for a in self.axes:
                v_des[a] *= scale

        qdot = self.kin.resolve_ee_velocity(
            self.q_cmd,
            v_des,
            self.command_joints,
            locked_joints=self.locked_joints,
            damping=self.ik_damping,
            axes=self.axes,
        )
        q_set = {}
        for name in self.command_joints:
            q_set[name] = self._clip_joint(name, self.q_cmd.get(name, 0.0) + qdot.get(name, 0.0) * dt)

        out_pos, out_vel = self.limiter.update(q_set, qdot, dt)
        for name in self.command_joints:
            self.q_cmd[name] = self._clip_joint(name, out_pos.get(name, self.q_cmd.get(name, 0.0)))

        achieved_xyz = self.kin.fk_position(self.q_cmd)
        self._last_status = {
            "command_joints": list(self.command_joints),
            "p_cmd": [float(v) for v in self.p_cmd],
            "ee_xyz": [float(v) for v in achieved_xyz],
            "cartesian_error_m": math.sqrt(
                sum((float(self.p_cmd[a]) - float(achieved_xyz[a])) ** 2 for a in self.axes)
            ),
            "v_des": [float(v) for v in v_des],
            "qdot": {n: float(qdot.get(n, 0.0)) for n in self.command_joints},
            "qdot_out": {n: float(out_vel.get(n, 0.0)) for n in self.command_joints},
            "limiter": type(self.limiter).__name__,
        }
        return {n: float(self.q_cmd[n]) for n in self.command_joints}

    def status(self):
        return dict(self._last_status)

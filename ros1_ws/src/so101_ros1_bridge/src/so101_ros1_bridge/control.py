import copy
import bisect
import math
import time


class JointSafetyFilter:
    def __init__(
        self,
        joint_order,
        active_joints,
        locked_joints,
        limits,
        max_velocity,
        home_positions,
        trajectory_interpolation="linear",
        clock=None,
    ):
        self.joint_order = list(joint_order)
        self.active_joints = set(active_joints)
        self.locked_joints = dict(locked_joints)
        self.limits = {name: tuple(value) for name, value in limits.items()}
        self.max_velocity = dict(max_velocity)
        self.home_positions = dict(home_positions)
        self.clock = clock or time.time
        self.target = self._complete_positions(home_positions)
        self.commanded = copy.deepcopy(self.target)
        self.trajectory_start = copy.deepcopy(self.commanded)
        self.trajectory_goal = copy.deepcopy(self.target)
        self.trajectory_start_time = self._now()
        self.trajectory_duration = 0.0
        self.timed_trajectory = None
        self.last_update_time = self._now()
        self.trajectory_interpolation = str(trajectory_interpolation or "linear")
        if self.trajectory_interpolation not in ("linear", "cubic"):
            raise ValueError("Unsupported trajectory_interpolation: %s" % self.trajectory_interpolation)

        missing = [name for name in self.joint_order if name not in self.target]
        if missing:
            raise ValueError("Missing home positions for joints: %s" % missing)

    def _now(self):
        return float(self.clock())

    def _clip_joint(self, name, value):
        lo, hi = self.limits.get(name, (-math.inf, math.inf))
        return max(lo, min(hi, float(value)))

    def _complete_positions(self, partial):
        values = {}
        for name in self.joint_order:
            if name in partial:
                values[name] = float(partial[name])
            elif name in self.locked_joints:
                values[name] = float(self.locked_joints[name])
            else:
                values[name] = 0.0
        for name, value in self.locked_joints.items():
            values[name] = float(value)
        return {name: self._clip_joint(name, values[name]) for name in self.joint_order}

    def _minimum_jerk(self, u):
        u = max(0.0, min(1.0, float(u)))
        return (10.0 * u ** 3) - (15.0 * u ** 4) + (6.0 * u ** 5)

    def _duration_for_move(self, start, goal, requested_duration):
        duration = max(0.0, float(requested_duration or 0.0))
        for name in self.joint_order:
            speed = abs(float(self.max_velocity.get(name, math.inf)))
            if speed <= 0.0 or math.isinf(speed):
                continue
            delta = abs(float(goal.get(name, 0.0)) - float(start.get(name, 0.0)))
            # Minimum-jerk peak speed is 1.875 * mean speed.
            duration = max(duration, 1.875 * delta / speed)
        return duration

    def _start_trajectory(self, goal, duration_s=0.0):
        now = self._now()
        self.timed_trajectory = None
        self.trajectory_start = self._complete_positions(self.commanded)
        self.trajectory_goal = self._complete_positions(goal)
        self.trajectory_start_time = now
        self.trajectory_duration = self._duration_for_move(
            self.trajectory_start,
            self.trajectory_goal,
            duration_s,
        )
        self.target = copy.deepcopy(self.trajectory_goal)
        self.last_update_time = now

    def set_timed_trajectory(self, joint_names, timed_positions, allow_locked=False):
        """Start a preplanned trajectory.

        timed_positions is an iterable of (time_from_start_s, {joint: position})
        waypoints.  Unlike repeated single-point commands, this keeps one
        trajectory clock and samples the path continuously.
        """
        names = list(joint_names)
        unknown = [name for name in names if name not in self.joint_order]
        if unknown:
            raise ValueError("Unknown joint in trajectory: %s" % ", ".join(unknown))

        start = self._complete_positions(self.commanded)
        waypoints = [(0.0, copy.deepcopy(start))]
        last = copy.deepcopy(start)
        last_t = 0.0
        for raw_t, partial in timed_positions:
            t = max(0.0, float(raw_t))
            if t < last_t:
                raise ValueError("Trajectory times must be nondecreasing")
            next_positions = copy.deepcopy(last)
            for name, value in partial.items():
                if name in self.locked_joints and not allow_locked:
                    next_positions[name] = float(self.locked_joints[name])
                elif name in self.joint_order:
                    next_positions[name] = self._clip_joint(name, value)
            next_positions = self._complete_positions(next_positions)
            if t <= 1e-9:
                # Keep the commanded start as the first sample unless the
                # caller explicitly provided the same start point.
                if all(abs(next_positions[name] - start[name]) <= 1e-6 for name in self.joint_order):
                    waypoints[0] = (0.0, next_positions)
                else:
                    waypoints.append((1e-3, next_positions))
                    last_t = 1e-3
                    last = next_positions
                continue
            if t <= last_t:
                t = last_t + 1e-3
            waypoints.append((t, next_positions))
            last_t = t
            last = next_positions

        if len(waypoints) < 2:
            self._start_trajectory(waypoints[-1][1], duration_s=0.0)
            return copy.deepcopy(self.target)

        velocities = self._estimate_waypoint_velocities(waypoints)
        self.timed_trajectory = {
            "start_time": self._now(),
            "times": [item[0] for item in waypoints],
            "positions": [item[1] for item in waypoints],
            "velocities": velocities,
        }
        self.trajectory_duration = 0.0
        self.target = copy.deepcopy(waypoints[-1][1])
        self.last_update_time = self._now()
        return copy.deepcopy(self.target)

    def _estimate_waypoint_velocities(self, waypoints):
        velocities = []
        if not waypoints:
            return velocities
        for idx, (_time_i, positions_i) in enumerate(waypoints):
            velocity = {}
            for name in self.joint_order:
                if idx == 0 or idx == len(waypoints) - 1:
                    velocity[name] = 0.0
                    continue
                t_prev, p_prev = waypoints[idx - 1]
                t_next, p_next = waypoints[idx + 1]
                dt = float(t_next) - float(t_prev)
                if dt <= 1e-9:
                    velocity[name] = 0.0
                else:
                    velocity[name] = (
                        float(p_next.get(name, positions_i.get(name, 0.0)))
                        - float(p_prev.get(name, positions_i.get(name, 0.0)))
                    ) / dt
            velocities.append(velocity)
        return velocities

    def _sample_segment(self, p0, p1, v0, v1, dt, alpha):
        alpha = max(0.0, min(1.0, float(alpha)))
        if self.trajectory_interpolation == "linear":
            return {
                name: self._clip_joint(
                    name,
                    float(p0.get(name, self.commanded.get(name, 0.0)))
                    + alpha
                    * (
                        float(p1.get(name, p0.get(name, self.commanded.get(name, 0.0))))
                        - float(p0.get(name, self.commanded.get(name, 0.0)))
                    ),
                )
                for name in self.joint_order
            }

        # Cubic Hermite interpolation keeps velocity continuous across dense
        # waypoints.  This is better for precomputed IK trajectories than
        # applying a zero-velocity ease-in/ease-out at every waypoint.
        a2 = alpha * alpha
        a3 = a2 * alpha
        h00 = 2.0 * a3 - 3.0 * a2 + 1.0
        h10 = a3 - 2.0 * a2 + alpha
        h01 = -2.0 * a3 + 3.0 * a2
        h11 = a3 - a2
        result = {}
        for name in self.joint_order:
            start = float(p0.get(name, self.commanded.get(name, 0.0)))
            goal = float(p1.get(name, start))
            start_vel = float(v0.get(name, 0.0))
            goal_vel = float(v1.get(name, 0.0))
            if name in self.locked_joints:
                goal = float(self.locked_joints[name])
                start = goal
                start_vel = 0.0
                goal_vel = 0.0
            value = h00 * start + h10 * dt * start_vel + h01 * goal + h11 * dt * goal_vel
            result[name] = self._clip_joint(name, value)
        return result

    def _sample_timed_trajectory(self, now):
        trajectory = self.timed_trajectory
        if not trajectory:
            return None
        elapsed = max(0.0, float(now) - float(trajectory["start_time"]))
        times = trajectory["times"]
        positions = trajectory["positions"]
        if elapsed >= times[-1]:
            self.timed_trajectory = None
            return copy.deepcopy(positions[-1])
        idx = bisect.bisect_right(times, elapsed) - 1
        idx = max(0, min(idx, len(times) - 2))
        t0, t1 = times[idx], times[idx + 1]
        p0, p1 = positions[idx], positions[idx + 1]
        if t1 <= t0:
            return copy.deepcopy(p1)
        alpha = (elapsed - t0) / (t1 - t0)
        velocities = trajectory.get("velocities") or [{} for _ in positions]
        return self._sample_segment(p0, p1, velocities[idx], velocities[idx + 1], t1 - t0, alpha)

    def set_target_positions(self, positions_by_name, allow_locked=False, duration_s=0.0):
        next_target = copy.deepcopy(self.target)
        for name, value in positions_by_name.items():
            if name not in self.joint_order:
                raise ValueError("Unknown joint in command: %s" % name)
            if name in self.locked_joints and not allow_locked:
                continue
            next_target[name] = self._clip_joint(name, value)
        self._start_trajectory(next_target, duration_s=duration_s)
        return copy.deepcopy(self.target)

    def set_servo_target(self, positions_by_name, allow_locked=False):
        """Update the tracking target for a high-rate streaming servo.

        Unlike ``set_target_positions``, this does NOT start a new minimum-jerk
        move.  Restarting the minimum-jerk profile on every streamed setpoint is
        the documented cause of phase lag and end-effector jitter (see
        ROS1_TRAJECTORY_DEBUG.md).  Instead we only move ``self.target`` and let
        ``step()`` velocity-limit ``commanded`` toward it continuously.  The
        upstream servo node is responsible for velocity/acceleration/jerk
        smoothing, so the setpoints arriving here are already feasible.
        """
        next_target = copy.deepcopy(self.target)
        for name, value in positions_by_name.items():
            if name not in self.joint_order:
                raise ValueError("Unknown joint in servo command: %s" % name)
            if name in self.locked_joints and not allow_locked:
                continue
            next_target[name] = self._clip_joint(name, value)
        # Cancel any in-flight timed trajectory / minimum-jerk move so step()
        # uses its velocity-limited tracking branch toward the new target.
        self.timed_trajectory = None
        self.trajectory_duration = 0.0
        self.target = self._complete_positions(next_target)
        self.last_update_time = self._now()
        return copy.deepcopy(self.target)

    def apply_deltas(self, deltas_by_name, duration_s=0.0):
        next_target = copy.deepcopy(self.target)
        for name, delta in deltas_by_name.items():
            if name not in self.joint_order:
                raise ValueError("Unknown joint in delta command: %s" % name)
            if name in self.locked_joints:
                continue
            if name not in self.active_joints:
                continue
            next_target[name] = self._clip_joint(name, next_target[name] + float(delta))
        self._start_trajectory(next_target, duration_s=duration_s)
        return copy.deepcopy(self.target)

    def home(self, duration_s=3.0):
        self._start_trajectory(self.home_positions, duration_s=duration_s)
        return copy.deepcopy(self.target)

    def freeze(self, current_positions):
        self.target = self._complete_positions(current_positions)
        self.commanded = self._complete_positions(current_positions)
        self.trajectory_start = copy.deepcopy(self.commanded)
        self.trajectory_goal = copy.deepcopy(self.target)
        self.trajectory_start_time = self._now()
        self.trajectory_duration = 0.0
        self.timed_trajectory = None
        self.last_update_time = self._now()
        return copy.deepcopy(self.target)

    def step(self, dt):
        dt = max(0.0, float(dt))
        now = self._now()
        timed_command = self._sample_timed_trajectory(now)
        if timed_command is not None:
            next_commanded = timed_command
        elif self.trajectory_duration > 0.0:
            elapsed = now - self.trajectory_start_time
            alpha = self._minimum_jerk(elapsed / self.trajectory_duration)
            next_commanded = {}
            for name in self.joint_order:
                start = float(self.trajectory_start.get(name, 0.0))
                goal = float(self.trajectory_goal.get(name, start))
                if name in self.locked_joints:
                    goal = float(self.locked_joints[name])
                next_commanded[name] = self._clip_joint(name, start + alpha * (goal - start))
            if elapsed >= self.trajectory_duration:
                self.trajectory_duration = 0.0
        else:
            next_commanded = copy.deepcopy(self.commanded)
            for name in self.joint_order:
                goal = self._clip_joint(name, self.target[name])
                if name in self.locked_joints:
                    goal = self._clip_joint(name, self.locked_joints[name])
                current = self.commanded.get(name, goal)
                max_step = abs(float(self.max_velocity.get(name, math.inf))) * dt
                if math.isinf(max_step):
                    next_commanded[name] = goal
                else:
                    err = goal - current
                    if abs(err) <= max_step:
                        next_commanded[name] = goal
                    else:
                        next_commanded[name] = current + math.copysign(max_step, err)
                next_commanded[name] = self._clip_joint(name, next_commanded[name])

        self.commanded = self._complete_positions(next_commanded)
        return copy.deepcopy(self.commanded)

    def is_command_stale(self, timeout_s):
        if timeout_s is None or timeout_s <= 0:
            return False
        return (self._now() - self.last_update_time) > timeout_s

    def trajectory_active(self):
        return bool(self.timed_trajectory) or self.trajectory_duration > 0.0

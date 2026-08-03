#!/usr/bin/env python3
"""FollowJointTrajectory action wrapper for the SO101 ROS1 bridge.

This node does not replace the existing driver.  It exposes the standard
control_msgs/FollowJointTrajectory action expected by MoveIt/ros_control-style
clients and forwards accepted trajectories to /so101/command_joint_positions.
"""

import math
import os
import sys
import threading
import time

import actionlib
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SO101FollowJointTrajectoryServer:
    def __init__(self):
        self.action_name = rospy.get_param("~action_name", "/so101/follow_joint_trajectory")
        self.command_topic = rospy.get_param("~command_topic", "/so101/command_joint_positions")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/so101/joint_states")
        self.joint_order = rospy.get_param(
            "~joint_order",
            ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
        )
        self.default_goal_tolerance_rad = float(rospy.get_param("~goal_tolerance_rad", 0.05))
        self.default_gripper_tolerance = float(rospy.get_param("~goal_tolerance_gripper", 0.03))
        self.goal_time_margin_s = float(rospy.get_param("~goal_time_margin_s", 1.0))
        self.feedback_rate_hz = float(rospy.get_param("~feedback_rate_hz", 20.0))
        self.enforce_path_tolerance = bool(rospy.get_param("~enforce_path_tolerance", False))
        self.old_goal_tolerance_s = float(rospy.get_param("~old_goal_tolerance_s", 0.5))
        self.resample_hz = float(rospy.get_param("~resample_hz", 0.0))
        self.max_resampled_points = int(rospy.get_param("~max_resampled_points", 10000))

        self.lock = threading.RLock()
        self.positions = {}
        self.have_joint_state = False

        self.pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        rospy.Subscriber(self.joint_states_topic, JointState, self._on_joint_state, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("SO101 FollowJointTrajectory action ready: %s -> %s", self.action_name, self.command_topic)

    def _on_joint_state(self, msg):
        with self.lock:
            self.positions = dict(zip(msg.name, msg.position))
            self.have_joint_state = True

    def _snapshot_positions(self):
        with self.lock:
            return dict(self.positions)

    def _wait_for_driver(self, timeout_s=6.0):
        deadline = time.time() + timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.pub.get_num_connections() > 0 and self.have_joint_state:
                return True
            rospy.sleep(0.05)
        return False

    def _abort(self, code, message):
        result = FollowJointTrajectoryResult()
        result.error_code = code
        result.error_string = message
        self.server.set_aborted(result, message)

    def _succeed(self, message="SO101 trajectory completed"):
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        result.error_string = message
        self.server.set_succeeded(result, message)

    def _validate_goal(self, goal):
        traj = goal.trajectory
        if not traj.joint_names:
            return FollowJointTrajectoryResult.INVALID_JOINTS, "trajectory.joint_names is empty"
        unknown = [name for name in traj.joint_names if name not in self.joint_order]
        if unknown:
            return (
                FollowJointTrajectoryResult.INVALID_JOINTS,
                "unknown SO101 joint(s): %s" % ", ".join(unknown),
            )
        if not traj.points:
            return FollowJointTrajectoryResult.INVALID_GOAL, "trajectory has no points"
        last_t = -1.0
        for idx, point in enumerate(traj.points):
            if len(point.positions) != len(traj.joint_names):
                return (
                    FollowJointTrajectoryResult.INVALID_GOAL,
                    "point %d positions length %d != joint_names length %d"
                    % (idx, len(point.positions), len(traj.joint_names)),
                )
            t = float(point.time_from_start.to_sec())
            if t < last_t:
                return FollowJointTrajectoryResult.INVALID_GOAL, "trajectory times must be nondecreasing"
            last_t = t
        if traj.header.stamp and traj.header.stamp != rospy.Time(0):
            age = (rospy.Time.now() - traj.header.stamp).to_sec()
            if age > self.old_goal_tolerance_s:
                return FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP, "trajectory header stamp is too old"
        return FollowJointTrajectoryResult.SUCCESSFUL, ""

    def _goal_tolerances(self, goal):
        tolerances = {}
        for name in goal.trajectory.joint_names:
            tolerances[name] = self.default_gripper_tolerance if name == "gripper" else self.default_goal_tolerance_rad
        for tol in goal.goal_tolerance:
            if tol.name in tolerances and tol.position > 0.0:
                tolerances[tol.name] = float(tol.position)
        return tolerances

    def _path_tolerances(self, goal):
        tolerances = {}
        for tol in goal.path_tolerance:
            if tol.name in goal.trajectory.joint_names and tol.position > 0.0:
                tolerances[tol.name] = float(tol.position)
        return tolerances

    def _publish_hold_current(self, duration_s=0.25):
        positions = self._snapshot_positions()
        names = [name for name in self.joint_order if name in positions]
        if not names:
            return
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(positions[name]) for name in names]
        point.time_from_start = rospy.Duration(duration_s)
        msg.points = [point]
        self.pub.publish(msg)

    def _publish_feedback(self, goal, desired_point):
        positions = self._snapshot_positions()
        feedback = FollowJointTrajectoryFeedback()
        feedback.header.stamp = rospy.Time.now()
        feedback.joint_names = list(goal.trajectory.joint_names)
        feedback.desired = desired_point
        feedback.actual.positions = [float(positions.get(name, 0.0)) for name in feedback.joint_names]
        feedback.error.positions = [
            float(feedback.actual.positions[idx]) - float(desired_point.positions[idx])
            for idx in range(len(feedback.joint_names))
        ]
        self.server.publish_feedback(feedback)

    def _interpolate_point(self, points, joint_names, elapsed):
        if elapsed <= points[0].time_from_start.to_sec():
            return points[0]
        for idx in range(len(points) - 1):
            t0 = points[idx].time_from_start.to_sec()
            t1 = points[idx + 1].time_from_start.to_sec()
            if t0 <= elapsed <= t1:
                if t1 <= t0:
                    return points[idx + 1]
                alpha = (elapsed - t0) / (t1 - t0)
                point = JointTrajectoryPoint()
                point.positions = [
                    (1.0 - alpha) * points[idx].positions[j] + alpha * points[idx + 1].positions[j]
                    for j in range(len(joint_names))
                ]
                point.time_from_start = rospy.Duration(elapsed)
                return point
        return points[-1]

    def _nearest_desired_point(self, goal, elapsed):
        return self._interpolate_point(goal.trajectory.points, goal.trajectory.joint_names, elapsed)

    def _resampled_trajectory(self, traj):
        if self.resample_hz <= 0.0:
            return traj
        duration = float(traj.points[-1].time_from_start.to_sec())
        if duration <= 0.0:
            return traj
        dt = 1.0 / max(1.0, self.resample_hz)
        count = int(math.floor(duration / dt)) + 1
        if count > self.max_resampled_points:
            raise RuntimeError(
                "resampled trajectory would have %d points; reduce duration/resample_hz or raise max_resampled_points"
                % count
            )
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = list(traj.joint_names)
        for idx in range(count + 1):
            elapsed = min(duration, idx * dt)
            point = self._interpolate_point(traj.points, traj.joint_names, elapsed)
            out = JointTrajectoryPoint()
            out.positions = list(point.positions)
            out.time_from_start = rospy.Duration(elapsed)
            msg.points.append(out)
            if elapsed >= duration:
                break
        return msg

    def _within_goal_tolerance(self, goal, tolerances):
        positions = self._snapshot_positions()
        final = goal.trajectory.points[-1]
        for idx, name in enumerate(goal.trajectory.joint_names):
            measured = positions.get(name)
            if measured is None:
                return False
            if abs(float(measured) - float(final.positions[idx])) > tolerances[name]:
                return False
        return True

    def _path_tolerance_violated(self, goal, desired_point, tolerances):
        if not tolerances:
            return False, ""
        positions = self._snapshot_positions()
        for idx, name in enumerate(goal.trajectory.joint_names):
            if name not in tolerances or name not in positions:
                continue
            err = abs(float(positions[name]) - float(desired_point.positions[idx]))
            if err > tolerances[name]:
                return True, "%s path error %.4f > %.4f" % (name, err, tolerances[name])
        return False, ""

    def _execute(self, goal):
        code, message = self._validate_goal(goal)
        if code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._abort(code, message)
            return
        if not self._wait_for_driver():
            self._abort(
                FollowJointTrajectoryResult.INVALID_GOAL,
                "SO101 driver is not connected to %s or no joint_states received" % self.command_topic,
            )
            return

        traj = goal.trajectory
        if traj.header.stamp and traj.header.stamp != rospy.Time(0):
            delay = (traj.header.stamp - rospy.Time.now()).to_sec()
            if delay > 0.0:
                rospy.sleep(delay)

        try:
            msg = self._resampled_trajectory(traj)
        except RuntimeError as exc:
            self._abort(FollowJointTrajectoryResult.INVALID_GOAL, str(exc))
            return
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

        duration = float(traj.points[-1].time_from_start.to_sec())
        extra = goal.goal_time_tolerance.to_sec() if goal.goal_time_tolerance else 0.0
        deadline = time.time() + duration + max(extra, self.goal_time_margin_s)
        start = time.time()
        rate = rospy.Rate(max(1.0, self.feedback_rate_hz))
        goal_tolerances = self._goal_tolerances(goal)
        path_tolerances = self._path_tolerances(goal) if self.enforce_path_tolerance else {}

        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                self._publish_hold_current()
                self.server.set_preempted(text="SO101 trajectory preempted")
                return
            elapsed = time.time() - start
            desired = self._nearest_desired_point(goal, min(elapsed, duration))
            self._publish_feedback(goal, desired)
            violated, reason = self._path_tolerance_violated(goal, desired, path_tolerances)
            if violated:
                self._publish_hold_current()
                self._abort(FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED, reason)
                return
            if elapsed >= duration and self._within_goal_tolerance(goal, goal_tolerances):
                self._succeed()
                return
            if time.time() > deadline:
                self._abort(
                    FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED,
                    "SO101 final joint state did not meet goal tolerance",
                )
                return
            rate.sleep()


def main():
    rospy.init_node("so101_follow_joint_trajectory_server")
    SO101FollowJointTrajectoryServer()
    rospy.spin()


if __name__ == "__main__":
    main()

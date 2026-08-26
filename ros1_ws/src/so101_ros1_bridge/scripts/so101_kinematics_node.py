#!/usr/bin/env python3
import json
import os
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.kinematics import SO101Kinematics


class SO101KinematicsNode:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tip_link = rospy.get_param("~tip_link", "gripper_frame_link")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/so101/joint_states")
        self.pose_topic = rospy.get_param("~pose_topic", "/so101/end_effector_pose")
        self.target_topic = rospy.get_param("~target_topic", "/so101/cartesian_target")
        self.command_topic = rospy.get_param("~command_topic", "/so101/command_joint_positions")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.command_duration_s = float(rospy.get_param("~command_duration_s", 1.0))
        self.ik_tolerance_m = float(rospy.get_param("~ik_tolerance_m", 0.006))
        self.ik_command_tolerance_m = float(rospy.get_param("~ik_command_tolerance_m", 0.008))
        self.ik_max_iters = int(rospy.get_param("~ik_max_iters", 120))
        self.ik_damping = float(rospy.get_param("~ik_damping", 0.035))
        self.ik_max_step = float(rospy.get_param("~ik_max_step", 0.08))
        self.ik_multi_start_first_target = bool(rospy.get_param("~ik_multi_start_first_target", True))
        self.ik_multi_start_every_target = bool(rospy.get_param("~ik_multi_start_every_target", False))
        self.ik_seed_reset_error_rad = float(rospy.get_param("~ik_seed_reset_error_rad", 0.35))

        self.joint_order = rospy.get_param("~joint_order", [])
        self.active_joints = rospy.get_param("~ik_active_joints", rospy.get_param("~active_joints", []))
        self.locked_joints = rospy.get_param("~locked_joints", {})
        self.limits = rospy.get_param("~limits", {})
        self.home_positions = rospy.get_param("~home_positions", {})
        self.workspace_limits = rospy.get_param("~workspace_limits", {})

        urdf = rospy.get_param("/robot_description", "")
        if not urdf:
            raise RuntimeError("Missing /robot_description; start with with_description:=true")
        self.kin = SO101Kinematics.from_urdf(
            urdf,
            base_link=self.base_frame,
            tip_link=self.tip_link,
            limits_override=self.limits,
        )

        self.lock = threading.RLock()
        self.positions = dict(self.home_positions)
        self.have_joint_state = False
        self.last_ik_solution = None
        self.last_status = {
            "ok": True,
            "message": "waiting for joint_states",
            "base_frame": self.base_frame,
            "tip_link": self.tip_link,
            "ik_active_joints": list(self.active_joints),
            "ik_tolerance_m": self.ik_tolerance_m,
            "ik_command_tolerance_m": self.ik_command_tolerance_m,
            "ik_damping": self.ik_damping,
            "ik_max_step": self.ik_max_step,
            "workspace_limits": self.workspace_limits,
        }

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher("/so101/kinematics_status", String, queue_size=10)
        self.command_pub = rospy.Publisher(self.command_topic, JointTrajectory, queue_size=1)
        rospy.Subscriber(self.joint_states_topic, JointState, self._on_joint_state, queue_size=1)
        rospy.Subscriber(self.target_topic, PoseStamped, self._on_cartesian_target, queue_size=1)

    def _on_joint_state(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.positions[name] = float(pos)
            self.have_joint_state = True

    def _pose_from_fk(self):
        transform = self.kin.fk(self.positions)
        quat = quaternion_from_matrix(transform)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(transform[0, 3])
        msg.pose.position.y = float(transform[1, 3])
        msg.pose.position.z = float(transform[2, 3])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        return msg

    def _publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.last_status, sort_keys=True)))

    def _target_in_workspace(self, target):
        for idx, axis in enumerate(("x", "y", "z")):
            limit = self.workspace_limits.get(axis)
            if limit is None:
                continue
            if not isinstance(limit, (list, tuple)) or len(limit) != 2:
                continue
            lower, upper = float(limit[0]), float(limit[1])
            if target[idx] < lower or target[idx] > upper:
                return False, "%s=%.4f outside [%.4f, %.4f]" % (axis, target[idx], lower, upper)
        return True, ""

    def _on_cartesian_target(self, msg):
        frame = msg.header.frame_id or self.base_frame
        if frame != self.base_frame:
            self.last_status = {"ok": False, "message": "target frame %s != %s" % (frame, self.base_frame)}
            rospy.logwarn(self.last_status["message"])
            return
        if not self.have_joint_state:
            self.last_status = {"ok": False, "message": "no joint_states received yet"}
            rospy.logwarn(self.last_status["message"])
            return

        target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        in_workspace, workspace_message = self._target_in_workspace(target)
        if not in_workspace:
            self.last_status = {
                "ok": False,
                "message": "target outside workspace: %s" % workspace_message,
                "target": target,
                "workspace_limits": self.workspace_limits,
            }
            rospy.logwarn(self.last_status["message"])
            return
        with self.lock:
            measured_seed = dict(self.positions)
            seed = dict(self.last_ik_solution) if self.last_ik_solution is not None else dict(measured_seed)

        seed_source = "last_ik_solution" if self.last_ik_solution is not None else "joint_states"
        if self.last_ik_solution is not None and self.ik_seed_reset_error_rad > 0.0:
            active_errors = [
                abs(float(measured_seed.get(name, 0.0)) - float(self.last_ik_solution.get(name, 0.0)))
                for name in self.active_joints
                if name in measured_seed and name in self.last_ik_solution
            ]
            if active_errors and max(active_errors) > self.ik_seed_reset_error_rad:
                seed = dict(measured_seed)
                seed_source = "joint_states_reset"
                with self.lock:
                    self.last_ik_solution = None

        use_multi_start = bool(self.ik_multi_start_every_target)
        if self.last_ik_solution is None and self.ik_multi_start_first_target:
            use_multi_start = True
        ok, solution, err, iters = self.kin.solve_ik_position(
            target,
            seed,
            self.active_joints,
            locked_joints=self.locked_joints,
            max_iters=self.ik_max_iters,
            tolerance=self.ik_tolerance_m,
            damping=self.ik_damping,
            max_step=self.ik_max_step,
            multi_start=use_multi_start,
        )
        accepted = bool(ok) or float(err) <= self.ik_command_tolerance_m
        self.last_status = {
            "ok": bool(accepted),
            "ik_commanded": bool(accepted),
            "ik_converged": bool(ok),
            "ik_approximate": bool((not ok) and accepted),
            "message": "ik solved" if ok else ("ik approximate accepted" if accepted else "ik did not meet tolerance"),
            "position_error_m": err,
            "iterations": iters,
            "target": target,
            "ik_tolerance_m": self.ik_tolerance_m,
            "ik_command_tolerance_m": self.ik_command_tolerance_m,
            "ik_damping": self.ik_damping,
            "ik_max_step": self.ik_max_step,
            "ik_seed_source": seed_source,
            "ik_multi_start": use_multi_start,
        }
        if not accepted:
            rospy.logwarn("SO101 IK target not reached: err=%.4fm after %d iterations", err, iters)
            return
        with self.lock:
            self.last_ik_solution = dict(solution)

        names = [name for name in self.active_joints if name in solution and name not in self.locked_joints]
        if not names:
            rospy.logwarn("SO101 IK solved but no commandable joints are active")
            return
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(solution[name]) for name in names]
        point.time_from_start = rospy.Duration(self.command_duration_s)
        traj.points = [point]
        self.command_pub.publish(traj)

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                pose = self._pose_from_fk()
            self.pose_pub.publish(pose)
            self._publish_status()
            rate.sleep()


def main():
    rospy.init_node("so101_kinematics_node")
    node = SO101KinematicsNode()
    node.spin()


if __name__ == "__main__":
    main()

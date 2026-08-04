#!/usr/bin/env python3
"""Real-time Cartesian servo for the planar SO101 arm.

This node closes a fixed-rate resolved-rate loop between an upper-layer command
(a Cartesian position target or an end-effector velocity command) and the SO101
driver's streaming servo channel.  It is the "online IK + limiter" path
described in the project notes, and is the intended integration point for the
aerial-manipulation experiments where a drone / RL policy continuously tells the
arm where its end effector should go.

Inputs (either one may drive the arm; velocity takes priority when fresh):
  * /so101/cartesian_servo_target  (geometry_msgs/PoseStamped)  -- xyz target
  * /so101/ee_velocity_cmd         (geometry_msgs/TwistStamped) -- linear m/s

Output:
  * /so101/command_joint_servo     (sensor_msgs/JointState)     -- joint stream
  * /so101/cartesian_servo_status  (std_msgs/String, JSON)

The servo law itself lives in the ROS-free so101_ros1_bridge.servo module so the
exact same control law can be replayed in the standalone MuJoCo simulation.

# CHUNK: body
"""

import json
import os
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.kinematics import SO101Kinematics
from so101_ros1_bridge.servo import PlanarCartesianServo


class SO101ServoNode:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.tip_link = rospy.get_param("~tip_link", "gripper_frame_link")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/so101/joint_states")
        self.command_topic = rospy.get_param("~servo_command_topic", "/so101/command_joint_servo")
        self.status_topic = rospy.get_param("~cartesian_servo_status_topic", "/so101/cartesian_servo_status")
        self.pose_target_topic = rospy.get_param("~cartesian_servo_target_topic", "/so101/cartesian_servo_target")
        self.velocity_topic = rospy.get_param("~ee_velocity_topic", "/so101/ee_velocity_cmd")

        self.control_rate_hz = float(rospy.get_param("~servo_rate_hz", 100.0))
        self.input_timeout_s = float(rospy.get_param("~servo_input_timeout_s", 0.3))
        self.joint_state_timeout_s = float(rospy.get_param("~servo_joint_state_timeout_s", 0.5))

        # Servo law tuning (see PlanarCartesianServo).
        self.position_gain = float(rospy.get_param("~servo_position_gain", 6.0))
        self.max_ee_speed = float(rospy.get_param("~servo_max_ee_speed", 0.25))
        self.ik_damping = float(rospy.get_param("~servo_ik_damping", 0.06))
        self.accel_scale = float(rospy.get_param("~servo_accel_scale", 6.0))
        self.jerk_scale = float(rospy.get_param("~servo_jerk_scale", 12.0))
        self.resync_gain = float(rospy.get_param("~servo_resync_gain", 0.0))
        self.prefer_ruckig = bool(rospy.get_param("~servo_prefer_ruckig", True))

        self.joint_order = rospy.get_param("~joint_order", [])
        self.active_joints = rospy.get_param(
            "~ik_active_joints", rospy.get_param("~active_joints", [])
        )
        self.locked_joints = rospy.get_param("~locked_joints", {})
        self.limits = rospy.get_param("~limits", {})
        self.max_velocity = rospy.get_param("~max_velocity", {})
        self.home_positions = rospy.get_param("~home_positions", {})
        self.workspace_limits = rospy.get_param("~workspace_limits", {})

        urdf = rospy.get_param("/robot_description", "")
        if not urdf:
            raise RuntimeError("Missing /robot_description; start the bridge with with_description:=true")
        self.kin = SO101Kinematics.from_urdf(
            urdf,
            base_link=self.base_frame,
            tip_link=self.tip_link,
            limits_override=self.limits,
        )

        self.servo = PlanarCartesianServo(
            self.kin,
            self.active_joints,
            self.locked_joints,
            self.limits,
            self.max_velocity,
            workspace_limits=self.workspace_limits,
            axes=(0, 2),
            position_gain=self.position_gain,
            max_ee_speed=self.max_ee_speed,
            ik_damping=self.ik_damping,
            accel_scale=self.accel_scale,
            jerk_scale=self.jerk_scale,
            control_dt=1.0 / max(1.0, self.control_rate_hz),
            resync_gain=self.resync_gain,
            prefer_ruckig=self.prefer_ruckig,
        )

        self.lock = threading.RLock()
        self.measured = dict(self.home_positions)
        self.have_joint_state = False
        self.last_joint_state_time = 0.0
        self.pose_target = None
        self.pose_target_time = 0.0
        self.velocity_cmd = None
        self.velocity_cmd_time = 0.0
        self.active = False
        self.enabled = True

        self.command_pub = rospy.Publisher(self.command_topic, JointState, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.joint_states_topic, JointState, self._on_joint_state, queue_size=1)
        rospy.Subscriber(self.pose_target_topic, PoseStamped, self._on_pose_target, queue_size=1)
        rospy.Subscriber(self.velocity_topic, TwistStamped, self._on_velocity_cmd, queue_size=1)
        rospy.Subscriber("/so101/servo_enable", Empty, self._on_enable, queue_size=1)
        rospy.Subscriber("/so101/servo_disable", Empty, self._on_disable, queue_size=1)

    def _now(self):
        return rospy.Time.now().to_sec()

    def _on_joint_state(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                self.measured[name] = float(pos)
            self.have_joint_state = True
            self.last_joint_state_time = self._now()

    def _on_pose_target(self, msg):
        frame = msg.header.frame_id or self.base_frame
        if frame != self.base_frame:
            rospy.logwarn_throttle(2.0, "Servo target frame %s != %s; ignoring", frame, self.base_frame)
            return
        with self.lock:
            self.pose_target = [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ]
            self.pose_target_time = self._now()

    def _on_velocity_cmd(self, msg):
        with self.lock:
            self.velocity_cmd = [
                float(msg.twist.linear.x),
                float(msg.twist.linear.y),
                float(msg.twist.linear.z),
            ]
            self.velocity_cmd_time = self._now()

    def _on_enable(self, _msg):
        with self.lock:
            self.enabled = True

    def _on_disable(self, _msg):
        with self.lock:
            self.enabled = False
            self.active = False
            self.velocity_cmd = None
            self.pose_target = None

    def _select_input(self, now):
        """Return (position_target, velocity_cmd) for this tick.

        Velocity commands win when fresh.  Stale inputs are dropped so the servo
        holds its last Cartesian setpoint instead of chasing an old command.
        """
        vel = None
        pos = None
        if self.velocity_cmd is not None and (now - self.velocity_cmd_time) <= self.input_timeout_s:
            vel = list(self.velocity_cmd)
        if self.pose_target is not None and (now - self.pose_target_time) <= self.input_timeout_s:
            pos = list(self.pose_target)
        return pos, vel

    def spin(self):
        rate = rospy.Rate(self.control_rate_hz)
        dt = 1.0 / max(1.0, self.control_rate_hz)
        while not rospy.is_shutdown():
            now = self._now()
            with self.lock:
                measured = dict(self.measured)
                have_state = self.have_joint_state
                state_fresh = have_state and (now - self.last_joint_state_time) <= self.joint_state_timeout_s
                enabled = self.enabled
                pos_target, vel_cmd = self._select_input(now)

            if not (enabled and state_fresh):
                # Drop out of servo control; re-seed on the next activation so we
                # never step from a stale internal state.
                if self.active:
                    with self.lock:
                        self.active = False
                self._publish_status(now, active=False, state_fresh=state_fresh, enabled=enabled)
                rate.sleep()
                continue

            with self.lock:
                if not self.active:
                    self.servo.reset(measured)
                    self.active = True
                # Hold position when there is no fresh command.
                q_cmd = self.servo.step(
                    dt,
                    measured_positions=measured,
                    position_target=pos_target,
                    velocity_cmd=vel_cmd,
                )
                status = self.servo.status()

            self._publish_command(now, q_cmd)
            self._publish_status(now, active=True, state_fresh=True, enabled=True, servo_status=status,
                                 has_pos=pos_target is not None, has_vel=vel_cmd is not None)
            rate.sleep()

    def _publish_command(self, now, q_cmd):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        names = list(q_cmd.keys())
        msg.name = names
        msg.position = [float(q_cmd[name]) for name in names]
        self.command_pub.publish(msg)

    def _publish_status(self, now, active, state_fresh, enabled, servo_status=None, has_pos=False, has_vel=False):
        payload = {
            "stamp": now,
            "active": bool(active),
            "enabled": bool(enabled),
            "joint_state_fresh": bool(state_fresh),
            "control_rate_hz": self.control_rate_hz,
            "input_timeout_s": self.input_timeout_s,
            "has_position_target": bool(has_pos),
            "has_velocity_cmd": bool(has_vel),
            "servo": servo_status or {},
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rospy.init_node("so101_servo_node")
    SO101ServoNode().spin()


if __name__ == "__main__":
    main()

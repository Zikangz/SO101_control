#!/usr/bin/env python3
"""Plot SO101 Cartesian planned and feedback trajectories.

Offline mode reads the CSV produced by so101_ee_sine_test.py.  Live mode
subscribes to the bridge and plots the current planned IK path, filtered motor
command, and end-effector feedback FK without publishing any control command.
"""

import argparse
import csv
import os
import sys
import threading
import time
from collections import deque

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

PKG_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.kinematics import SO101Kinematics


JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def _float_or_none(value):
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _read_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def _xyz_rows(rows, prefix):
    points = []
    for row in rows:
        point = [_float_or_none(row.get(prefix + axis)) for axis in ("x", "y", "z")]
        if all(value is not None for value in point):
            points.append(point)
    return points


def _time_series(rows, column):
    values = []
    for row in rows:
        elapsed = _float_or_none(row.get("elapsed"))
        value = _float_or_none(row.get(column))
        if elapsed is not None and value is not None:
            values.append((elapsed, value))
    return values


def _matplotlib(live):
    if not live:
        os.environ.setdefault("MPLBACKEND", "Agg")
    config_dir = os.environ.setdefault("MPLCONFIGDIR", "/tmp/so101_matplotlib")
    os.makedirs(config_dir, exist_ok=True)
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required. Install it with: "
            "python3 -m pip install --user 'matplotlib>=3.3,<3.8'"
        ) from exc
    return plt


def _plot_path(ax, title, planned, commanded, measured):
    if planned:
        ax.plot(
            [point[0] * 1000.0 for point in planned],
            [point[2] * 1000.0 for point in planned],
            "--",
            color="tab:green",
            linewidth=1.8,
            label="IK target",
        )
    if commanded:
        ax.plot(
            [point[0] * 1000.0 for point in commanded],
            [point[2] * 1000.0 for point in commanded],
            color="tab:blue",
            linewidth=1.2,
            alpha=0.8,
            label="filtered command FK",
        )
    if measured:
        ax.plot(
            [point[0] * 1000.0 for point in measured],
            [point[2] * 1000.0 for point in measured],
            color="tab:red",
            linewidth=1.5,
            label="feedback FK",
        )
    ax.set_title(title)
    ax.set_xlabel("base_link X (mm)")
    ax.set_ylabel("base_link Z (mm)")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    if ax.lines:
        ax.legend(loc="best")


def _plot_offline_csv(path, output_dir, show, plt):
    rows = _read_rows(path)
    if not rows:
        raise RuntimeError("CSV has no rows: %s" % path)
    planned = _xyz_rows(rows, "command_target_") or _xyz_rows(rows, "target_")
    nominal = _xyz_rows(rows, "target_")
    measured = _xyz_rows(rows, "measured_")
    if not planned or not measured:
        raise RuntimeError(
            "CSV must contain command_target_x/y/z (or target_x/y/z) and measured_x/y/z: %s" % path
        )

    stem = os.path.splitext(os.path.basename(path))[0]
    directory = output_dir or os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)

    fig, axes = plt.subplots(2, 1, figsize=(10, 9), constrained_layout=True)
    _plot_path(axes[0], "SO101 end-effector trajectory", planned, [], measured)
    if nominal and nominal != planned:
        axes[0].plot(
            [point[0] * 1000.0 for point in nominal],
            [point[2] * 1000.0 for point in nominal],
            ":",
            color="0.35",
            linewidth=1.0,
            label="nominal target",
        )
        axes[0].legend(loc="best")

    for column, label, color in (
        ("error_x", "X error", "tab:blue"),
        ("error_z", "Z error", "tab:red"),
        ("error_norm", "norm", "tab:purple"),
    ):
        values = _time_series(rows, column)
        if values:
            axes[1].plot(
                [value[0] for value in values],
                [value[1] * 1000.0 for value in values],
                linewidth=1.0,
                color=color,
                label=label,
            )
    axes[1].axhline(0.0, color="0.25", linewidth=0.8)
    axes[1].set_title("Cartesian tracking error")
    axes[1].set_xlabel("elapsed time (s)")
    axes[1].set_ylabel("error (mm)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="best")
    ee_output = os.path.join(directory, stem + "_ee_tracking.png")
    fig.savefig(ee_output, dpi=160)

    joint_rows = [joint for joint in JOINTS if _time_series(rows, "planned_" + joint)]
    joint_output = ""
    if joint_rows:
        fig_joint, axes_joint = plt.subplots(
            len(joint_rows),
            1,
            figsize=(11, 3.0 * len(joint_rows)),
            sharex=True,
            constrained_layout=True,
        )
        if len(joint_rows) == 1:
            axes_joint = [axes_joint]
        for axis, joint in zip(axes_joint, joint_rows):
            for column, label, color, style in (
                ("planned_" + joint, "IK planned", "tab:green", "--"),
                ("commanded_" + joint, "filtered command", "tab:blue", "-"),
                (joint, "feedback", "tab:red", "-"),
            ):
                values = _time_series(rows, column)
                if values:
                    axis.plot(
                        [value[0] for value in values],
                        [value[1] for value in values],
                        linestyle=style,
                        color=color,
                        linewidth=1.1,
                        label=label,
                    )
            axis.set_ylabel("%s (rad)" % joint)
            axis.grid(True, alpha=0.3)
            axis.legend(loc="best")
        axes_joint[-1].set_xlabel("elapsed time (s)")
        joint_output = os.path.join(directory, stem + "_joint_tracking.png")
        fig_joint.savefig(joint_output, dpi=160)
        if not show:
            plt.close(fig_joint)

    print("ee_plot:", ee_output)
    if joint_output:
        print("joint_plot:", joint_output)
    if show:
        plt.show()
    else:
        plt.close(fig)


class LiveTrajectoryPlot:
    def __init__(self, args, plt):
        urdf = rospy.get_param("/robot_description", "")
        if not urdf:
            raise RuntimeError("Missing /robot_description; start the SO101 bridge with with_description:=true")
        limits = rospy.get_param("/so101_driver_node/limits", {})
        self.kin = SO101Kinematics.from_urdf(
            urdf,
            base_link=args.frame,
            tip_link=args.tip_link,
            limits_override=limits,
        )
        self.args = args
        self.plt = plt
        self.lock = threading.RLock()
        self.actual_positions = {}
        self.commanded_positions = {}
        self.planned_path = []
        self.actual_path = deque()
        self.commanded_path = deque()
        self.error_history = deque()
        self.last_trajectory_stamp = 0.0

        self.fig, (self.path_axis, self.error_axis) = plt.subplots(2, 1, figsize=(9, 9), constrained_layout=True)
        self.fig.canvas.manager.set_window_title("SO101 IK and feedback trajectory")
        rospy.Subscriber(args.joint_states_topic, JointState, self._on_joint_state, queue_size=20)
        rospy.Subscriber(args.commanded_joint_states_topic, JointState, self._on_commanded_joint_state, queue_size=20)
        rospy.Subscriber(args.command_topic, JointTrajectory, self._on_joint_trajectory, queue_size=2)

    def _fk(self, positions):
        if not positions:
            return None
        transform = self.kin.fk(positions)
        return [float(transform[0, 3]), float(transform[1, 3]), float(transform[2, 3])]

    @staticmethod
    def _message_time(msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        return stamp if stamp > 0.0 else time.time()

    def _trim(self, now):
        cutoff = now - max(1.0, float(self.args.history_seconds))
        for values in (self.actual_path, self.commanded_path, self.error_history):
            while values and values[0][0] < cutoff:
                values.popleft()

    def _on_joint_state(self, msg):
        now = self._message_time(msg)
        with self.lock:
            self.actual_positions.update(dict(zip(msg.name, msg.position)))
            point = self._fk(self.actual_positions)
            if point is None:
                return
            self.actual_path.append((now, point))
            command = self._fk(self.commanded_positions)
            if command is not None:
                self.error_history.append(
                    (now, (point[0] - command[0]) * 1000.0, (point[2] - command[2]) * 1000.0)
                )
            self._trim(now)

    def _on_commanded_joint_state(self, msg):
        now = self._message_time(msg)
        with self.lock:
            self.commanded_positions.update(dict(zip(msg.name, msg.position)))
            point = self._fk(self.commanded_positions)
            if point is not None:
                self.commanded_path.append((now, point))
            self._trim(now)

    def _on_joint_trajectory(self, msg):
        if not msg.joint_names or not msg.points:
            return
        with self.lock:
            seed = dict(self.commanded_positions or self.actual_positions)
            path = []
            for point in msg.points:
                if len(point.positions) != len(msg.joint_names):
                    return
                seed.update(dict(zip(msg.joint_names, point.positions)))
                xyz = self._fk(seed)
                if xyz is not None:
                    path.append(xyz)
            if path:
                self.planned_path = path
                self.last_trajectory_stamp = time.time()
                if self.args.reset_on_trajectory:
                    self.actual_path.clear()
                    self.commanded_path.clear()
                    self.error_history.clear()

    def redraw(self):
        with self.lock:
            planned = list(self.planned_path)
            commanded = [point for _stamp, point in self.commanded_path]
            actual = [point for _stamp, point in self.actual_path]
            errors = list(self.error_history)
        self.path_axis.clear()
        self.error_axis.clear()
        _plot_path(self.path_axis, "Live SO101 end-effector trajectory", planned, commanded, actual)
        if errors:
            t0 = errors[0][0]
            self.error_axis.plot(
                [stamp - t0 for stamp, _x, _z in errors],
                [x for _stamp, x, _z in errors],
                color="tab:blue",
                linewidth=1.0,
                label="feedback - command X",
            )
            self.error_axis.plot(
                [stamp - t0 for stamp, _x, _z in errors],
                [z for _stamp, _x, z in errors],
                color="tab:red",
                linewidth=1.0,
                label="feedback - command Z",
            )
            self.error_axis.legend(loc="best")
        self.error_axis.axhline(0.0, color="0.25", linewidth=0.8)
        self.error_axis.set_title("Latest feedback minus filtered command FK")
        self.error_axis.set_xlabel("history time (s)")
        self.error_axis.set_ylabel("Cartesian error (mm)")
        self.error_axis.grid(True, alpha=0.3)
        self.fig.canvas.draw_idle()

    def run(self):
        self.plt.ion()
        self.plt.show(block=False)
        rate = rospy.Rate(max(1.0, self.args.update_rate_hz))
        while not rospy.is_shutdown() and self.plt.fignum_exists(self.fig.number):
            self.redraw()
            self.plt.pause(0.001)
            rate.sleep()
        if self.args.save:
            self.fig.savefig(self.args.save, dpi=160)
            print("live_plot:", self.args.save)


def build_parser():
    parser = argparse.ArgumentParser(description="Plot SO101 planned IK and feedback-FK trajectories")
    parser.add_argument("--csv", nargs="+", default=[], help="EE trajectory CSV file(s) for offline plots")
    parser.add_argument("--output-dir", default="", help="Directory for offline PNG output; default: each CSV directory")
    parser.add_argument("--show", action="store_true", help="Show offline figures after saving")
    parser.add_argument("--live", action="store_true", help="Show a live ROS plot; does not publish control commands")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--tip-link", default="gripper_frame_link")
    parser.add_argument("--joint-states-topic", default="/so101/joint_states")
    parser.add_argument("--commanded-joint-states-topic", default="/so101/commanded_joint_states")
    parser.add_argument("--command-topic", default="/so101/command_joint_positions")
    parser.add_argument("--history-seconds", type=float, default=120.0)
    parser.add_argument("--update-rate-hz", type=float, default=15.0)
    parser.add_argument("--reset-on-trajectory", action="store_true", default=True)
    parser.add_argument("--no-reset-on-trajectory", dest="reset_on_trajectory", action="store_false")
    parser.add_argument("--save", default="", help="Save the final live window to this PNG path when it closes")
    return parser


def main():
    args = build_parser().parse_args()
    if not args.csv and not args.live:
        raise RuntimeError("Specify --csv FILE [FILE ...] or --live")
    plt = _matplotlib(args.live)
    if args.csv:
        for path in args.csv:
            _plot_offline_csv(path, args.output_dir, args.show, plt)
    if args.live:
        rospy.init_node("so101_plot_ee_trajectory", anonymous=True)
        LiveTrajectoryPlot(args, plt).run()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, rospy.ROSException) as exc:
        print("SO101 trajectory plot error: %s" % exc, file=sys.stderr)
        raise SystemExit(1)

#!/usr/bin/env python3
"""Offline SO101 Cartesian trajectory validator.

This script does not start ROS nodes, publish topics, or touch hardware.  It
loads the SO101 URDF/xacro plus bridge YAML config, then checks whether a
locked shoulder_pan/wrist_roll planar 3R arm can follow an XZ figure-eight.
"""

import argparse
import csv
import json
import math
import os
import shutil
import statistics
import subprocess
import sys

import yaml

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG_SRC = os.path.join(PKG_DIR, "src")
if PKG_SRC not in sys.path:
    sys.path.insert(0, PKG_SRC)

from so101_ros1_bridge.kinematics import SO101Kinematics
from so101_ros1_bridge.poses import JOINT_ORDER, SAFE_POSES


ACTIVE_PLANAR_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
LOCKED_JOINTS = {"shoulder_pan": 0.0, "wrist_roll": 0.0}
TIP_LINK = "gripper_frame_link"
BASE_LINK = "base_link"


def _repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))


def _default_config_path():
    return os.path.join(PKG_DIR, "config", "so101_simplified_4dof.yaml")


def _default_xacro_path():
    return os.path.join(_repo_root(), "ros1_ws", "src", "so101_description", "urdf", "so101_arm.urdf.xacro")


def _load_yaml(path):
    with open(path, "r") as handle:
        return yaml.safe_load(handle) or {}


def _xacro_to_urdf(path):
    if not path.endswith(".xacro"):
        with open(path, "r") as handle:
            return handle.read()

    xacro_bin = shutil.which("xacro") or "/opt/ros/noetic/bin/xacro"
    if not os.path.exists(xacro_bin):
        raise RuntimeError("xacro not found; source /opt/ros/noetic/setup.bash first")

    env = os.environ.copy()
    ros_src = os.path.join(_repo_root(), "ros1_ws", "src")
    env["ROS_PACKAGE_PATH"] = ros_src + (":" + env["ROS_PACKAGE_PATH"] if env.get("ROS_PACKAGE_PATH") else "")
    env.setdefault("ROS_HOME", "/tmp/so101_ros_home")
    os.makedirs(env["ROS_HOME"], exist_ok=True)
    cmd = [xacro_bin, path, "variant:=follower", "use_ros2_control:=false"]
    result = subprocess.run(cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    if result.returncode != 0:
        raise RuntimeError("xacro failed:\n%s" % result.stderr.strip())
    return result.stdout


def _joint_seed(config, pose_name, center_joints=None):
    seed = {}
    seed.update({name: float(value) for name, value in (config.get("home_positions") or {}).items()})
    seed.update(LOCKED_JOINTS)
    if pose_name != "none":
        seed.update({name: float(value) for name, value in SAFE_POSES[pose_name].items() if name in JOINT_ORDER})
    if center_joints:
        seed.update({name: float(value) for name, value in center_joints.items()})
    return seed


def _trajectory_offset(pattern, theta, x_amp, z_amp):
    if pattern == "figure8_xz":
        return [x_amp * math.sin(theta), 0.0, z_amp * math.sin(2.0 * theta)]
    if pattern == "sine_x":
        return [x_amp * math.sin(theta), 0.0, 0.0]
    if pattern == "sine_z":
        return [0.0, 0.0, z_amp * math.sin(theta)]
    raise RuntimeError("unsupported pattern: %s" % pattern)


def _limits_for_mode(kin, config, mode):
    if mode == "mechanical":
        return {name: tuple(value) for name, value in kin.limits.items()}
    if mode == "conservative":
        merged = {name: tuple(value) for name, value in kin.limits.items()}
        for name, value in (config.get("limits") or {}).items():
            if isinstance(value, (list, tuple)) and len(value) == 2:
                merged[name] = (float(value[0]), float(value[1]))
        return merged
    raise RuntimeError("unsupported limits mode: %s" % mode)


def _workspace_violation(target, workspace_limits):
    for idx, axis in enumerate(("x", "y", "z")):
        bounds = (workspace_limits or {}).get(axis)
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            continue
        lo, hi = float(bounds[0]), float(bounds[1])
        if target[idx] < lo or target[idx] > hi:
            return "%s=%.4f outside [%.4f, %.4f]" % (axis, target[idx], lo, hi)
    return ""


def _limit_hits(solution, limits, joint_names, eps=1e-4):
    hits = []
    outside = []
    for name in joint_names:
        if name not in solution or name not in limits:
            continue
        value = float(solution[name])
        lo, hi = limits[name]
        if value < lo - eps or value > hi + eps:
            outside.append(name)
        if abs(value - lo) <= eps or abs(value - hi) <= eps:
            hits.append(name)
    return hits, outside


def _joint_limit_margins(solution, limits, joint_names):
    margins = {}
    for name in joint_names:
        if name not in solution or name not in limits:
            continue
        lo, hi = limits[name]
        value = float(solution[name])
        margins[name] = min(value - float(lo), float(hi) - value)
    return margins, min(margins.values()) if margins else float("inf")


def _bounds(points):
    return {
        "x": [min(point[0] for point in points), max(point[0] for point in points)],
        "y": [min(point[1] for point in points), max(point[1] for point in points)],
        "z": [min(point[2] for point in points), max(point[2] for point in points)],
    }


def _validate(kin, config, args, mode, center, center_seed, quiet=False):
    limits = _limits_for_mode(kin, config, mode)
    kin_mode = args.kinematics_by_mode[mode]
    workspace_limits = config.get("workspace_limits") or {}
    accept_tolerance = float(args.accept_tolerance if args.accept_tolerance is not None else config.get("ik_command_tolerance_m", 0.008))
    solve_tolerance = float(args.solve_tolerance if args.solve_tolerance is not None else config.get("ik_tolerance_m", 0.002))
    max_iters = int(args.max_iters if args.max_iters is not None else config.get("ik_max_iters", 200))

    rows = []
    failures = []
    accepted_count = 0
    converged_count = 0
    last_solution = dict(center_seed)
    targets = []
    command_targets = []
    residuals = []
    joint_min_margin = {name: float("inf") for name in ACTIVE_PLANAR_JOINTS}
    for idx in range(args.samples):
        theta = 2.0 * math.pi * float(idx) / float(args.samples)
        offset = _trajectory_offset(args.pattern, theta, args.x_amp, args.z_amp)
        target = [center[axis] + offset[axis] for axis in range(3)]
        targets.append(target)
        command_target = list(target)
        command_target[2] += float(args.z_feedforward_bias)
        command_targets.append(command_target)

        workspace_reason = _workspace_violation(command_target, workspace_limits)
        converged, solution, residual, iters = kin_mode.solve_ik_position(
            command_target,
            last_solution,
            ACTIVE_PLANAR_JOINTS,
            locked_joints=LOCKED_JOINTS,
            max_iters=max_iters,
            tolerance=solve_tolerance,
            # Match so101_ee_sine_test.py: choose an IK branch once and
            # continue it, rather than jumping branches at every point.
            multi_start=(idx == 0),
        )
        residual = float(residual)
        residuals.append(residual)
        joint_limit_hits, joint_outside = _limit_hits(solution, limits, ACTIVE_PLANAR_JOINTS)
        joint_margins, min_margin = _joint_limit_margins(solution, limits, ACTIVE_PLANAR_JOINTS)
        margin_ok = min_margin >= float(args.joint_limit_margin)
        accepted = (not workspace_reason) and (bool(converged) or residual <= accept_tolerance) and margin_ok
        if accepted:
            accepted_count += 1
            last_solution = dict(solution)
        if converged:
            converged_count += 1

        if workspace_reason:
            reason = "workspace_limit"
        elif joint_outside:
            reason = "joint_limit_exceeded"
        elif not margin_ok:
            reason = "joint_limit_margin_insufficient"
        elif joint_limit_hits and not accepted:
            reason = "joint_limit_saturated"
        elif not accepted:
            reason = "solver_not_converged_or_geometric_unreachable"
        elif not converged:
            reason = "approximate_within_command_tolerance"
        else:
            reason = "ok"

        row = {
            "idx": idx,
            "limits_mode": mode,
            "pattern": args.pattern,
            "frame": args.frame,
            "center_x": center[0],
            "center_y": center[1],
            "center_z": center[2],
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "command_target_x": command_target[0],
            "command_target_y": command_target[1],
            "command_target_z": command_target[2],
            "z_feedforward_bias": float(args.z_feedforward_bias),
            "residual_m": residual,
            "converged": bool(converged),
            "accepted": bool(accepted),
            "iterations": int(iters),
            "reason": reason,
            "workspace_violation": workspace_reason,
            "joint_limit_hits": ",".join(joint_limit_hits),
            "joint_outside_limits": ",".join(joint_outside),
            "minimum_joint_limit_margin_rad": min_margin,
            "joint_limit_margin_ok": margin_ok,
        }
        for name in ACTIVE_PLANAR_JOINTS:
            row[name] = float(solution.get(name, 0.0))
            if name in limits:
                row[name + "_limit_lo"] = float(limits[name][0])
                row[name + "_limit_hi"] = float(limits[name][1])
                row[name + "_limit_margin_rad"] = joint_margins.get(name, float("inf"))
                joint_min_margin[name] = min(joint_min_margin[name], joint_margins.get(name, float("inf")))
        rows.append(row)
        if not accepted:
            failures.append(row)

    target_bounds = _bounds(targets)
    summary = {
        "limits_mode": mode,
        "frame": args.frame,
        "tip_link": args.tip_link,
        "locked_joints": dict(LOCKED_JOINTS),
        "active_planar_joints": list(ACTIVE_PLANAR_JOINTS),
        "gripper_note": "gripper/q6 is not in the IK chain to gripper_frame_link and is not used for EE trajectory tracking",
        "pattern": args.pattern,
        "total_width_m": args.total_width,
        "total_height_m": args.total_height,
        "x_amp_m": args.x_amp,
        "z_amp_m": args.z_amp,
        "center_xyz_m": list(center),
        "target_min_max_m": target_bounds,
        "command_target_min_max_m": _bounds(command_targets),
        "z_feedforward_bias_m": float(args.z_feedforward_bias),
        "joint_limit_margin_required_rad": float(args.joint_limit_margin),
        "joint_minimum_margin_rad": joint_min_margin,
        "accepted": accepted_count,
        "total": args.samples,
        "converged": converged_count,
        "max_residual_m": max(residuals) if residuals else None,
        "mean_residual_m": statistics.fmean(residuals) if residuals else None,
        "failed_indices": [int(row["idx"]) for row in failures],
        "failure_count": len(failures),
    }
    if not quiet:
        _print_summary(summary, failures, args.max_failures_print)
    return summary, rows, failures


def _print_summary(summary, failures, max_failures_print):
    print("SO101 offline EE trajectory validation")
    print("  limits_mode:          %s" % summary["limits_mode"])
    print("  frame:                %s" % summary["frame"])
    print("  tip_link:             %s" % summary["tip_link"])
    print("  active_planar_joints: %s" % ", ".join(summary["active_planar_joints"]))
    print("  locked_joints:        shoulder_pan=%.4f, wrist_roll=%.4f" % (
        summary["locked_joints"]["shoulder_pan"],
        summary["locked_joints"]["wrist_roll"],
    ))
    print("  q6/gripper:           ignored for EE tracking")
    print("  pattern:              %s" % summary["pattern"])
    print("  total_width/height:   %.3f / %.3f m" % (summary["total_width_m"], summary["total_height_m"]))
    print("  x_amp/z_amp:          %.3f / %.3f m" % (summary["x_amp_m"], summary["z_amp_m"]))
    print("  center_xyz_m:         %.4f %.4f %.4f" % tuple(summary["center_xyz_m"]))
    print("  target_x_min_max_m:   %.4f %.4f" % tuple(summary["target_min_max_m"]["x"]))
    print("  target_y_min_max_m:   %.4f %.4f" % tuple(summary["target_min_max_m"]["y"]))
    print("  target_z_min_max_m:   %.4f %.4f" % tuple(summary["target_min_max_m"]["z"]))
    print("  command_z_min_max_m:  %.4f %.4f" % tuple(summary["command_target_min_max_m"]["z"]))
    print("  z_feedforward_bias_m: %.6f" % summary["z_feedforward_bias_m"])
    print("  joint_limit_margin_m: %.6f" % summary["joint_limit_margin_required_rad"])
    print("  accepted/total:       %d/%d" % (summary["accepted"], summary["total"]))
    print("  converged/total:      %d/%d" % (summary["converged"], summary["total"]))
    print("  max_residual_m:       %.6f" % summary["max_residual_m"])
    print("  mean_residual_m:      %.6f" % summary["mean_residual_m"])
    print("  joint_min_margin_rad:")
    for name in ACTIVE_PLANAR_JOINTS:
        print("    %-14s %.6f" % (name, summary["joint_minimum_margin_rad"].get(name, float("inf"))))
    if failures:
        print("  failed_indices:       %s" % ",".join(str(row["idx"]) for row in failures))
        print("  failed_points:")
        for row in failures[:max_failures_print]:
            print(
                "    idx={idx:03d} target=({target_x:.4f},{target_y:.4f},{target_z:.4f}) "
                "residual={residual_m:.6f} q2/q3/q4=({shoulder_lift:.4f},{elbow_flex:.4f},{wrist_flex:.4f}) "
                "min_margin={minimum_joint_limit_margin_rad:.6f} reason={reason}".format(**row)
            )
        if len(failures) > max_failures_print:
            print("    ... %d more failures in CSV/JSON" % (len(failures) - max_failures_print))
    else:
        print("  failed_indices:       none")


def _write_outputs(prefix, summaries, rows):
    if not prefix:
        return
    os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
    csv_path = prefix + ".csv"
    json_path = prefix + ".json"
    keys = sorted({key for row in rows for key in row})
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    with open(json_path, "w") as handle:
        json.dump({"summaries": summaries, "rows": rows}, handle, indent=2, sort_keys=True)
    print("  csv:                  %s" % csv_path)
    print("  json:                 %s" % json_path)


def _key_points_pass(config, args, mode, center, center_seed):
    kin_mode = args.kinematics_by_mode[mode]
    limits = _limits_for_mode(kin_mode, config, mode)
    workspace_limits = config.get("workspace_limits") or {}
    solve_tolerance = float(args.solve_tolerance if args.solve_tolerance is not None else config.get("ik_tolerance_m", 0.002))
    max_iters = int(args.scan_max_iters)
    key_thetas = [
        0.0,
        math.pi / 4.0,
        math.pi / 2.0,
        3.0 * math.pi / 4.0,
        math.pi,
        5.0 * math.pi / 4.0,
        3.0 * math.pi / 2.0,
        7.0 * math.pi / 4.0,
    ]
    q = dict(center_seed)
    for theta in key_thetas:
        offset = _trajectory_offset(args.pattern, theta, args.x_amp, args.z_amp)
        target = [center[axis] + offset[axis] for axis in range(3)]
        command_target = list(target)
        command_target[2] += float(args.z_feedforward_bias)
        if _workspace_violation(command_target, workspace_limits):
            return False
        converged, solution, residual, _ = kin_mode.solve_ik_position(
            command_target,
            q,
            ACTIVE_PLANAR_JOINTS,
            locked_joints=LOCKED_JOINTS,
            max_iters=max_iters,
            tolerance=solve_tolerance,
            multi_start=(theta == key_thetas[0]),
        )
        hits, outside = _limit_hits(solution, limits, ACTIVE_PLANAR_JOINTS)
        _margins, min_margin = _joint_limit_margins(solution, limits, ACTIVE_PLANAR_JOINTS)
        if outside or (float(residual) > args.accept_tolerance_for_scan and hits):
            return False
        if min_margin < float(args.joint_limit_margin):
            return False
        if (not converged) and float(residual) > args.accept_tolerance_for_scan:
            return False
        q = dict(solution)
    return True


def _scan_center(kin, config, args, mode, original_center, center_seed):
    workspace = config.get("workspace_limits") or {}
    x_bounds = workspace.get("x", [0.07, 0.45])
    z_bounds = workspace.get("z", [0.0, 0.38])
    x_lo = float(x_bounds[0]) + args.x_amp
    x_hi = float(x_bounds[1]) - args.x_amp
    z_lo = float(z_bounds[0]) + args.z_amp
    z_hi = float(z_bounds[1]) - args.z_amp
    if x_lo > x_hi or z_lo > z_hi:
        print("  scan_center:          impossible inside workspace box after amplitude margins")
        return None

    best = None
    scan_args = argparse.Namespace(**vars(args))
    scan_args.samples = min(args.samples, args.scan_samples)
    scan_args.max_iters = args.scan_max_iters
    x_count = int(math.floor((x_hi - x_lo) / args.scan_step_m)) + 1
    z_count = int(math.floor((z_hi - z_lo) / args.scan_step_m)) + 1
    for xi in range(x_count + 1):
        x0 = min(x_hi, x_lo + xi * args.scan_step_m)
        for zi in range(z_count + 1):
            z0 = min(z_hi, z_lo + zi * args.scan_step_m)
            center = [x0, original_center[1], z0]
            if not _key_points_pass(config, args, mode, center, center_seed):
                continue
            summary, _, _ = _validate(kin, config, scan_args, mode, center, center_seed, quiet=True)
            if summary["accepted"] != summary["total"]:
                continue
            if summary["max_residual_m"] is None or summary["max_residual_m"] > args.accept_tolerance_for_scan:
                continue
            distance = math.sqrt((x0 - original_center[0]) ** 2 + (z0 - original_center[2]) ** 2)
            min_margin = min(summary["joint_minimum_margin_rad"].values())
            score = (-min_margin, summary["max_residual_m"], summary["mean_residual_m"], distance)
            if best is None or score < best["score"]:
                best = {"center": center, "summary": summary, "score": score}

    if best is None:
        print("  scan_center:          no 100%% accepted center found at step %.3fm" % args.scan_step_m)
        return None

    center = best["center"]
    print("  scan_center:          found")
    print("  recommended_center:   %.4f %.4f %.4f" % tuple(center))
    print("  recommended_params:   pattern=%s total_width=%.3f total_height=%.3f x_amp=%.3f z_amp=%.3f" % (
        args.pattern,
        args.total_width,
        args.total_height,
        args.x_amp,
        args.z_amp,
    ))
    print("  scan_max_residual_m:  %.6f" % best["summary"]["max_residual_m"])
    print("  scan_mean_residual_m: %.6f" % best["summary"]["mean_residual_m"])
    print("  scan_min_margin_rad:  %.6f" % min(best["summary"]["joint_minimum_margin_rad"].values()))
    return best


def build_parser():
    parser = argparse.ArgumentParser(description="Validate locked q1/q5 SO101 EE trajectories offline")
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--urdf", default=_default_xacro_path(), help="SO101 URDF or xacro")
    parser.add_argument("--frame", default=BASE_LINK, help="Trajectory frame; current SO101 URDF uses base_link")
    parser.add_argument("--tip-link", default=TIP_LINK)
    parser.add_argument("--pattern", choices=["figure8_xz", "sine_x", "sine_z"], default="figure8_xz")
    parser.add_argument("--center", nargs=3, type=float, default=None, help="Center xyz in --frame")
    parser.add_argument("--center-pose", choices=["none"] + sorted(SAFE_POSES), default="reach")
    parser.add_argument("--total-width", type=float, default=0.120, help="Peak-to-peak X size in metres")
    parser.add_argument("--total-height", type=float, default=0.060, help="Peak-to-peak Z size in metres")
    parser.add_argument("--x-amplitude", type=float, default=None, help="Override X half-amplitude in metres")
    parser.add_argument("--z-amplitude", type=float, default=None, help="Override Z half-amplitude in metres")
    parser.add_argument("--samples", type=int, default=180)
    parser.add_argument("--limits-mode", choices=["conservative", "mechanical", "both"], default="both")
    parser.add_argument("--solve-tolerance", type=float, default=None)
    parser.add_argument("--accept-tolerance", type=float, default=None)
    parser.add_argument("--max-iters", type=int, default=None)
    parser.add_argument("--max-failures-print", type=int, default=25)
    parser.add_argument("--output-prefix", default="")
    parser.add_argument("--scan-center", action="store_true")
    parser.add_argument("--scan-step-m", type=float, default=0.010)
    parser.add_argument("--scan-samples", type=int, default=72)
    parser.add_argument("--scan-max-iters", type=int, default=80)
    parser.add_argument("--accept-tolerance-for-scan", type=float, default=0.008)
    parser.add_argument("--z-feedforward-bias", type=float, default=0.0, help="Metres added to IK command Z, matching so101_ee_sine_test.py")
    parser.add_argument("--joint-limit-margin", type=float, default=0.05, help="Required active-joint distance from hard limits in radians")
    return parser


def main():
    args = build_parser().parse_args()
    args.x_amp = float(args.x_amplitude if args.x_amplitude is not None else args.total_width / 2.0)
    args.z_amp = float(args.z_amplitude if args.z_amplitude is not None else args.total_height / 2.0)
    args.total_width = float(args.total_width if args.x_amplitude is None else 2.0 * args.x_amp)
    args.total_height = float(args.total_height if args.z_amplitude is None else 2.0 * args.z_amp)
    if args.samples < 4:
        raise SystemExit("--samples must be >= 4")
    if args.scan_step_m <= 0.0:
        raise SystemExit("--scan-step-m must be > 0")

    config = _load_yaml(args.config)
    urdf = _xacro_to_urdf(args.urdf)
    kin = SO101Kinematics.from_urdf(urdf, base_link=args.frame, tip_link=args.tip_link)
    center_seed = _joint_seed(config, args.center_pose)
    center = list(args.center) if args.center is not None else list(kin.fk(center_seed)[:3, 3])
    modes = ["conservative", "mechanical"] if args.limits_mode == "both" else [args.limits_mode]
    args.kinematics_by_mode = {}
    for mode in ("conservative", "mechanical"):
        args.kinematics_by_mode[mode] = SO101Kinematics.from_urdf(
            urdf,
            base_link=args.frame,
            tip_link=args.tip_link,
            limits_override=_limits_for_mode(kin, config, mode),
        )

    print("Coordinate convention")
    print("  frame:                %s (SO101 arm base frame; no separate arm_base frame exists in this ROS model)" % args.frame)
    print("  axes:                 +X forward from base, +Y lateral about shoulder_pan, +Z upward")
    print("  center_source:        %s" % ("cli" if args.center is not None else "SAFE_POSES[%s] FK" % args.center_pose))
    print("  current_script_note:  so101_ee_sine_test.py uses x/y amplitudes as half-amplitudes; its figure8 z term is 0.5*z_amplitude*sin(2t).")

    all_rows = []
    summaries = []
    exit_code = 0
    for mode in modes:
        summary, rows, failures = _validate(kin, config, args, mode, center, center_seed, quiet=False)
        summaries.append(summary)
        all_rows.extend(rows)
        if failures:
            exit_code = 2
        if args.scan_center:
            _scan_center(kin, config, args, mode, center, center_seed)
    _write_outputs(args.output_prefix, summaries, all_rows)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

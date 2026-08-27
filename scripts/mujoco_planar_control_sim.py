from __future__ import annotations

import argparse
import collections
import collections.abc
import copy
import csv
import fractions
import math
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
ROS1_SRC = ROOT / "ros1_ws" / "src" / "so101_ros1_bridge" / "src"
if str(ROS1_SRC) not in sys.path:
    sys.path.insert(0, str(ROS1_SRC))

os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt

from so101_ros1_bridge.control import JointSafetyFilter
from so101_ros1_bridge.kinematics import SO101Kinematics
from so101_ros1_bridge.poses import JOINT_ORDER, SAFE_POSES
from so101_ros1_bridge.servo import PlanarCartesianServo
from video_utils import write_mp4

DEFAULT_CONFIG = ROOT / "ros1_ws" / "src" / "so101_ros1_bridge" / "config" / "so101_planar_3dof_gripper.yaml"
DEFAULT_MODEL = ROOT / "assets" / "so101" / "scene.xml"
DEFAULT_URDF = ROOT / "assets" / "so101" / "so101_new_calib.urdf"
EE_SITE = "gripperframe"
EE_BODY = "gripper"
ARM_IK_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
PLANAR_AXES = (0, 2)
REPO_ARGO_ROOT = ROOT / "third_party" / "Argo-Robot-controls-raw"
EXTERNAL_ARGO_ROOT = ROOT.parents[0] / "external" / "Argo-Robot-controls-raw"
ARGO_ROOT = REPO_ARGO_ROOT if REPO_ARGO_ROOT.exists() else EXTERNAL_ARGO_ROOT


class SimClock:
    def __init__(self):
        self.t = 0.0

    def time(self) -> float:
        return float(self.t)

    def advance(self, dt: float) -> None:
        self.t += max(0.0, float(dt))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pure Python MuJoCo simulation of the planar SO101 control pipeline."
    )
    parser.add_argument("--model-path", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--controller",
        choices=["joint_trajectory", "cartesian_stream", "moveit_like", "moveit_ruckig", "argo_like", "argo_external", "servo"],
        default=None,
        help=(
            "Controller to execute. If omitted, --execution-mode is used for backward "
            "compatibility. moveit_like retimes the IK waypoint path with joint velocity "
            "limits; moveit_ruckig uses the Python Ruckig library used by MoveIt for jerk-limited "
            "smoothing; argo_external uses the downloaded Argo-Robot/controls URDF IK; servo runs "
            "the real-time resolved-rate Cartesian servo (so101_ros1_bridge.servo) shared with the "
            "ROS servo node, driving the EE target directly each control tick."
        ),
    )
    parser.add_argument(
        "--execution-mode",
        choices=["joint_trajectory", "cartesian_stream"],
        default="joint_trajectory",
        help="Backward-compatible alias for --controller joint_trajectory/cartesian_stream.",
    )
    parser.add_argument("--prep-pose", choices=sorted(SAFE_POSES), default="reach")
    parser.add_argument(
        "--start-at-first-target",
        action="store_true",
        help=(
            "Initialize the simulated arm at the IK solution for the first target. "
            "Useful for rear/backbend trajectories where starting from the front reach pose "
            "would dominate the max error."
        ),
    )
    parser.add_argument("--ee-frame", choices=["site_gripperframe", "body_gripper"], default="site_gripperframe")
    parser.add_argument("--limit-profile", choices=["config", "mujoco"], default="config")
    parser.add_argument("--center", nargs=3, type=float, default=None, help="Absolute target center xyz in MuJoCo base frame.")
    parser.add_argument("--x-amplitude", type=float, default=0.03)
    parser.add_argument("--z-amplitude", type=float, default=0.03)
    parser.add_argument("--frequency", type=float, default=0.05)
    parser.add_argument("--cycles", type=float, default=1.0, help="Number of full figure-eight cycles when --duration is omitted.")
    parser.add_argument("--duration", type=float, default=None, help="Simulation trajectory duration. Defaults to cycles / frequency.")
    parser.add_argument("--plan-rate", type=float, default=20.0)
    parser.add_argument("--control-rate", type=float, default=100.0)
    parser.add_argument("--post-hold", type=float, default=1.0)
    parser.add_argument("--command-duration", type=float, default=None)
    parser.add_argument("--trajectory-interpolation", choices=["linear", "cubic"], default=None)
    parser.add_argument("--moveit-velocity-scaling", type=float, default=0.7)
    parser.add_argument("--moveit-acceleration-scale", type=float, default=4.0)
    parser.add_argument("--moveit-jerk-scale", type=float, default=40.0)
    parser.add_argument("--argo-k", type=float, default=0.8)
    parser.add_argument("--argo-iters", type=int, default=50)
    parser.add_argument("--ik-iters", type=int, default=160)
    parser.add_argument("--ik-tolerance", type=float, default=5e-5)
    parser.add_argument("--ik-damping", type=float, default=0.035)
    parser.add_argument("--ik-max-dq", type=float, default=0.08)
    # --controller servo tuning (shared PlanarCartesianServo law).
    parser.add_argument("--urdf-path", type=Path, default=DEFAULT_URDF, help="URDF for the servo's analytic-free kinematics")
    parser.add_argument("--servo-input", choices=["position", "velocity"], default="position",
                        help="How the servo controller consumes the figure-eight target: as a position setpoint or as its finite-difference velocity")
    parser.add_argument("--servo-position-gain", type=float, default=6.0)
    parser.add_argument("--servo-max-ee-speed", type=float, default=0.25)
    parser.add_argument("--servo-ik-damping", type=float, default=0.06)
    parser.add_argument("--servo-accel-scale", type=float, default=6.0)
    parser.add_argument("--servo-jerk-scale", type=float, default=12.0)
    parser.add_argument("--servo-no-ruckig", dest="servo_prefer_ruckig", action="store_false")
    parser.set_defaults(servo_prefer_ruckig=True)
    parser.add_argument("--plan-multistart-every-target", action="store_true")
    parser.add_argument("--stream-multistart-every-target", action="store_true")
    parser.add_argument("--log-rate", type=float, default=100.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--viewer-real-time", action="store_true")
    parser.add_argument("--no-viewer-markers", dest="viewer_markers", action="store_false")
    parser.add_argument("--live-plot", action="store_true", help="Show live target/EE XZ trajectory and error plots.")
    parser.add_argument("--plot-rate", type=float, default=10.0, help="Live plot refresh rate in Hz.")
    parser.add_argument("--record-video", action="store_true", help="Record the MuJoCo simulation to an mp4 file.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate used for --record-video outputs.")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "mujoco_planar_control_sim")
    return parser.parse_args()


def load_config(path: Path) -> dict:
    with path.open() as handle:
        return yaml.safe_load(handle)


def resolve_run_settings(args: argparse.Namespace) -> None:
    args.controller_name = args.controller or args.execution_mode
    if args.duration is None:
        if args.frequency <= 0.0:
            raise ValueError("--frequency must be positive when --duration is omitted")
        if args.cycles <= 0.0:
            raise ValueError("--cycles must be positive")
        args.duration = float(args.cycles) / float(args.frequency)
    args.duration = float(args.duration)
    completed_cycles = args.duration * float(args.frequency)
    if completed_cycles < 0.999:
        print(
            "Warning: duration %.3fs at %.3f Hz only covers %.3f figure-eight cycles. "
            "Omit --duration or set --duration >= %.3f for one full cycle."
            % (args.duration, args.frequency, completed_cycles, 1.0 / max(args.frequency, 1e-9)),
            file=sys.stderr,
        )


def make_model(args: argparse.Namespace):
    model = mujoco.MjModel.from_xml_path(str(args.model_path))
    data = mujoco.MjData(model)
    if args.ee_frame == "body_gripper":
        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, EE_BODY)
        if ee_id < 0:
            raise ValueError("MuJoCo body not found: %s" % EE_BODY)
        ee_ref = {"kind": "body", "name": EE_BODY, "id": int(ee_id)}
    else:
        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, EE_SITE)
        if ee_id < 0:
            raise ValueError("MuJoCo site not found: %s" % EE_SITE)
        ee_ref = {"kind": "site", "name": EE_SITE, "id": int(ee_id)}
    return model, data, ee_ref


def ee_position(data: mujoco.MjData, ee_ref: dict) -> np.ndarray:
    if ee_ref["kind"] == "body":
        return data.xpos[ee_ref["id"]].copy()
    return data.site_xpos[ee_ref["id"]].copy()


def ee_jacobian_position(model: mujoco.MjModel, data: mujoco.MjData, ee_ref: dict) -> np.ndarray:
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    if ee_ref["kind"] == "body":
        mujoco.mj_jacBody(model, data, jacp, None, ee_ref["id"])
    else:
        mujoco.mj_jacSite(model, data, jacp, None, ee_ref["id"])
    return jacp


def model_maps(model: mujoco.MjModel, joint_order: list[str]) -> dict:
    qpos_addr = {}
    qvel_addr = {}
    actuator_id = {}
    joint_range = {}
    for name in joint_order:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if jid < 0:
            raise ValueError("MuJoCo joint not found: %s" % name)
        if aid < 0:
            raise ValueError("MuJoCo actuator not found: %s" % name)
        qpos_addr[name] = int(model.jnt_qposadr[jid])
        qvel_addr[name] = int(model.jnt_dofadr[jid])
        actuator_id[name] = int(aid)
        joint_range[name] = tuple(float(v) for v in model.jnt_range[jid])
    return {
        "qpos_addr": qpos_addr,
        "qvel_addr": qvel_addr,
        "actuator_id": actuator_id,
        "joint_range": joint_range,
    }


def apply_limit_profile(config: dict, maps: dict, args: argparse.Namespace) -> dict:
    out = copy.deepcopy(config)
    if args.limit_profile == "mujoco":
        limits = dict(out.get("limits", {}))
        for name in ARM_IK_JOINTS:
            limits[name] = list(maps["joint_range"][name])
        out["limits"] = limits
    return out


def control_range(model: mujoco.MjModel, maps: dict, joint: str) -> tuple[float, float]:
    ctrl_range = model.actuator_ctrlrange[maps["actuator_id"][joint]]
    if float(ctrl_range[1]) > float(ctrl_range[0]):
        return float(ctrl_range[0]), float(ctrl_range[1])
    return maps["joint_range"][joint]


def control_to_mujoco(model: mujoco.MjModel, maps: dict, joint: str, value: float) -> float:
    if joint == "gripper":
        lo, hi = control_range(model, maps, joint)
        normalized = max(0.0, min(1.0, float(value)))
        return lo + normalized * (hi - lo)
    return float(value)


def mujoco_to_control(model: mujoco.MjModel, maps: dict, joint: str, value: float) -> float:
    if joint == "gripper":
        lo, hi = control_range(model, maps, joint)
        if hi <= lo:
            return 0.0
        return max(0.0, min(1.0, (float(value) - lo) / (hi - lo)))
    return float(value)


def complete_positions(config: dict, pose: dict | None = None) -> dict[str, float]:
    pose = pose or {}
    home = config.get("home_positions", {})
    locked = config.get("locked_joints", {})
    out = {}
    for name in config["joint_order"]:
        out[name] = float(home.get(name, 0.0))
    for name, value in pose.items():
        out[name] = float(value)
    for name, value in locked.items():
        out[name] = float(value)
    return out


def read_control_positions(model: mujoco.MjModel, data: mujoco.MjData, maps: dict, joint_order: list[str]) -> dict[str, float]:
    return {
        name: mujoco_to_control(model, maps, name, data.qpos[maps["qpos_addr"][name]])
        for name in joint_order
    }


def write_qpos(model: mujoco.MjModel, data: mujoco.MjData, maps: dict, positions: dict[str, float]) -> None:
    for name, value in positions.items():
        if name in maps["qpos_addr"]:
            data.qpos[maps["qpos_addr"][name]] = control_to_mujoco(model, maps, name, value)


def write_ctrl(model: mujoco.MjModel, data: mujoco.MjData, maps: dict, positions: dict[str, float]) -> None:
    for name, value in positions.items():
        if name in maps["actuator_id"]:
            data.ctrl[maps["actuator_id"][name]] = control_to_mujoco(model, maps, name, value)


def clip_joint(config: dict, name: str, value: float) -> float:
    lo, hi = config.get("limits", {}).get(name, (-math.inf, math.inf))
    return max(float(lo), min(float(hi), float(value)))


def target_at(center: np.ndarray, args: argparse.Namespace, elapsed: float) -> np.ndarray:
    phase = 2.0 * math.pi * args.frequency * float(elapsed)
    return center + np.array(
        [
            args.x_amplitude * math.sin(phase),
            0.0,
            args.z_amplitude * math.sin(2.0 * phase),
        ],
        dtype=np.float64,
    )


def solve_planar_dls_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    maps: dict,
    config: dict,
    ee_ref: dict,
    target: np.ndarray,
    seed: dict[str, float],
    args: argparse.Namespace,
    multi_start: bool = False,
) -> tuple[bool, dict[str, float], float, int]:
    seeds = [dict(seed)]
    if multi_start:
        seeds.extend(
            [
                complete_positions(config, SAFE_POSES["ready"]),
                complete_positions(config, SAFE_POSES["reach"]),
                complete_positions(config, SAFE_POSES["stow"]),
            ]
        )

    best = None
    for candidate_seed in seeds:
        q = complete_positions(config, candidate_seed)
        write_qpos(model, data, maps, q)
        mujoco.mj_forward(model, data)
        err_norm = float("inf")
        iterations = 0
        for iteration in range(max(1, int(args.ik_iters))):
            iterations = iteration + 1
            current = ee_position(data, ee_ref)
            err = target[list(PLANAR_AXES)] - current[list(PLANAR_AXES)]
            err_norm = float(np.linalg.norm(err))
            if err_norm <= args.ik_tolerance:
                break
            jacp = ee_jacobian_position(model, data, ee_ref)
            cols = [maps["qvel_addr"][name] for name in ARM_IK_JOINTS]
            jac = jacp[list(PLANAR_AXES), :][:, cols]
            lhs = jac @ jac.T + (float(args.ik_damping) ** 2) * np.eye(len(PLANAR_AXES))
            try:
                dq = jac.T @ np.linalg.solve(lhs, err)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(jac) @ err
            step_norm = float(np.linalg.norm(dq))
            if step_norm > args.ik_max_dq:
                dq = dq * (args.ik_max_dq / step_norm)
            for idx, name in enumerate(ARM_IK_JOINTS):
                q[name] = clip_joint(config, name, q[name] + float(dq[idx]))
            write_qpos(model, data, maps, q)
            mujoco.mj_forward(model, data)

        current = ee_position(data, ee_ref)
        final_err = float(np.linalg.norm(target[list(PLANAR_AXES)] - current[list(PLANAR_AXES)]))
        continuity = math.sqrt(sum((q[name] - seed.get(name, q[name])) ** 2 for name in ARM_IK_JOINTS))
        score = final_err + 0.001 * continuity
        candidate = (score, final_err, q, iterations)
        if best is None or candidate[0] < best[0]:
            best = candidate

    _, err_norm, solution, iterations = best
    return err_norm <= args.ik_tolerance, dict(solution), float(err_norm), int(iterations)


def build_planned_path(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    maps: dict,
    config: dict,
    ee_ref: dict,
    center: np.ndarray,
    seed: dict[str, float],
    args: argparse.Namespace,
) -> tuple[list[str], list[tuple[float, dict[str, float]]], list[dict[str, float]]]:
    active = [name for name in config["active_joints"] if name not in config.get("locked_joints", {})]
    samples = int(math.floor(args.duration * args.plan_rate)) + 1
    last_solution = dict(seed)
    timed_positions = []
    plan_rows = []
    for idx in range(samples):
        elapsed = min(args.duration, idx / args.plan_rate)
        target = target_at(center, args, elapsed)
        ok, solution, ik_err, iters = solve_planar_dls_ik(
            model,
            data,
            maps,
            config,
            ee_ref,
            target,
            last_solution,
            args,
            multi_start=(idx == 0 or args.plan_multistart_every_target),
        )
        last_solution = dict(solution)
        point = {name: float(solution[name]) for name in active if name in solution}
        timed_positions.append((elapsed, point))
        plan_rows.append(
            {
                "elapsed": elapsed,
                "target_x": float(target[0]),
                "target_y": float(target[1]),
                "target_z": float(target[2]),
                "ik_ok": float(bool(ok)),
                "ik_error_xz_m": ik_err,
                "ik_iterations": float(iters),
                **{"planned_" + name: float(point[name]) for name in point},
            }
        )
    return active, timed_positions, plan_rows


def retime_timed_positions(
    timed_positions: list[tuple[float, dict[str, float]]],
    max_velocity: dict,
    velocity_scaling: float,
) -> list[tuple[float, dict[str, float]]]:
    """MoveIt-style time parameterization with joint velocity limits.

    This is intentionally lightweight: it keeps the same IK waypoint path but
    lengthens segment durations when the requested timing would exceed the
    configured joint velocity limits. It gives a local, ROS-free baseline that
    can later be replaced by MoveIt's Iterative Parabolic / TOTG / Ruckig
    retiming in a Noetic workspace.
    """
    if len(timed_positions) < 2:
        return list(timed_positions)
    scaling = max(1e-3, min(1.0, float(velocity_scaling)))
    retimed = [(0.0, dict(timed_positions[0][1]))]
    new_t = 0.0
    prev_raw_t, prev = timed_positions[0]
    for raw_t, positions in timed_positions[1:]:
        original_dt = max(0.0, float(raw_t) - float(prev_raw_t))
        required_dt = 0.0
        for name, value in positions.items():
            vmax = abs(float(max_velocity.get(name, math.inf))) * scaling
            if vmax <= 0.0 or math.isinf(vmax):
                continue
            required_dt = max(required_dt, abs(float(value) - float(prev.get(name, value))) / vmax)
        new_t += max(original_dt, required_dt)
        retimed.append((new_t, dict(positions)))
        prev_raw_t = raw_t
        prev = positions
    return retimed


def estimate_waypoint_velocities(
    timed_positions: list[tuple[float, dict[str, float]]],
    command_names: list[str],
) -> list[dict[str, float]]:
    velocities = []
    for idx, (_time_i, positions_i) in enumerate(timed_positions):
        velocity = {}
        for name in command_names:
            if idx == 0 or idx == len(timed_positions) - 1:
                velocity[name] = 0.0
                continue
            t_prev, p_prev = timed_positions[idx - 1]
            t_next, p_next = timed_positions[idx + 1]
            dt = float(t_next) - float(t_prev)
            if dt <= 1e-9:
                velocity[name] = 0.0
            else:
                velocity[name] = (float(p_next.get(name, positions_i.get(name, 0.0))) - float(p_prev.get(name, positions_i.get(name, 0.0)))) / dt
        velocities.append(velocity)
    return velocities


def ruckig_retime_timed_positions(
    timed_positions: list[tuple[float, dict[str, float]]],
    command_names: list[str],
    max_velocity: dict,
    control_dt: float,
    args: argparse.Namespace,
) -> list[tuple[float, dict[str, float]]]:
    try:
        from ruckig import InputParameter, OutputParameter, Result, Ruckig
    except ImportError as exc:
        raise RuntimeError("moveit_ruckig requires the Python ruckig package") from exc

    if len(timed_positions) < 2:
        return list(timed_positions)

    dofs = len(command_names)
    dt = max(1e-4, float(control_dt))
    waypoint_velocities = estimate_waypoint_velocities(timed_positions, command_names)
    otg = Ruckig(dofs, dt)
    inp = InputParameter(dofs)
    out = OutputParameter(dofs)

    first = timed_positions[0][1]
    inp.current_position = [float(first.get(name, 0.0)) for name in command_names]
    inp.current_velocity = [0.0] * dofs
    inp.current_acceleration = [0.0] * dofs
    vmax = [max(1e-3, abs(float(max_velocity.get(name, 1.0))) * float(args.moveit_velocity_scaling)) for name in command_names]
    inp.max_velocity = vmax
    inp.max_acceleration = [max(0.05, v * float(args.moveit_acceleration_scale)) for v in vmax]
    inp.max_jerk = [max(0.5, a * float(args.moveit_jerk_scale)) for a in inp.max_acceleration]

    smoothed = [(0.0, dict(first))]
    t_global = 0.0
    for waypoint_idx, (_raw_t, target_positions) in enumerate(timed_positions[1:], start=1):
        inp.target_position = [float(target_positions.get(name, smoothed[-1][1].get(name, 0.0))) for name in command_names]
        inp.target_velocity = [
            max(-vmax[j], min(vmax[j], float(waypoint_velocities[waypoint_idx].get(name, 0.0))))
            for j, name in enumerate(command_names)
        ]
        inp.target_acceleration = [0.0] * dofs
        for iteration in range(200000):
            result = otg.update(inp, out)
            t_global += dt
            point = {name: float(out.new_position[j]) for j, name in enumerate(command_names)}
            smoothed.append((t_global, point))
            out.pass_to_input(inp)
            if result == Result.Finished:
                break
        else:
            raise RuntimeError("Ruckig failed to reach waypoint %d" % waypoint_idx)
    return smoothed


def _patch_argo_dependencies() -> None:
    for name in ("Mapping", "MutableMapping", "Sequence", "Set", "Iterable"):
        if not hasattr(collections, name):
            setattr(collections, name, getattr(collections.abc, name))
    if not hasattr(fractions, "gcd"):
        fractions.gcd = math.gcd
    for name, typ in (("int", int), ("float", float), ("bool", bool)):
        if not hasattr(np, name):
            setattr(np, name, typ)


def make_argo_context() -> dict:
    if not ARGO_ROOT.exists():
        raise RuntimeError("Argo source not found: %s" % ARGO_ROOT)
    _patch_argo_dependencies()
    if str(ARGO_ROOT) not in sys.path:
        sys.path.insert(0, str(ARGO_ROOT))
    from scripts.kinematics import URDF_Kinematics
    from scripts.model import RobotModel, URDF_loader

    urdf_path = ARGO_ROOT / "models" / "so101" / "so101.urdf"
    loader = URDF_loader()
    loader.load(str(urdf_path))
    return {
        "kin": URDF_Kinematics(),
        "robot": RobotModel(loader),
        "ee_link": "gripper_link",
    }


def solve_argo_external_ik(
    config: dict,
    target: np.ndarray,
    seed: dict[str, float],
    argo_context: dict,
    args: argparse.Namespace,
) -> tuple[bool, dict[str, float], float, int]:
    q_control = complete_positions(config, seed)
    q_mujoco = np.array([q_control[name] for name in config["joint_order"]], dtype=np.float64)
    q_argo = q_mujoco[::-1].copy()

    kin = argo_context["kin"]
    robot = argo_context["robot"]
    ee_link = argo_context["ee_link"]
    desired = kin._forward_kinematics_baseTn(robot, q_argo, ee_link)
    desired[:3, 3] = np.asarray(target, dtype=np.float64)
    q_next = kin._inverse_kinematics_step_baseTn(
        robot,
        q_argo,
        desired,
        ee_link,
        False,
        float(args.argo_k),
        int(args.argo_iters),
    )
    q_next_mujoco = q_next[::-1]
    solution = {}
    for idx, name in enumerate(config["joint_order"]):
        solution[name] = clip_joint(config, name, float(q_next_mujoco[idx]))
    final_q_argo = np.array([solution[name] for name in config["joint_order"]], dtype=np.float64)[::-1]
    final_pose = kin._forward_kinematics_baseTn(robot, final_q_argo, ee_link)
    final_err = float(np.linalg.norm(np.asarray(target)[list(PLANAR_AXES)] - final_pose[:3, 3][list(PLANAR_AXES)]))
    return final_err <= args.ik_tolerance, solution, final_err, int(args.argo_iters)


def nearest_plan_row(plan_rows: list[dict[str, float]], elapsed: float) -> dict[str, float]:
    if not plan_rows:
        return {}
    if len(plan_rows) == 1:
        return plan_rows[0]
    dt = max(1e-9, plan_rows[1]["elapsed"] - plan_rows[0]["elapsed"])
    idx = int(round(elapsed / dt))
    return plan_rows[max(0, min(idx, len(plan_rows) - 1))]


def make_filter(config: dict, interpolation: str, clock: SimClock) -> JointSafetyFilter:
    return JointSafetyFilter(
        config["joint_order"],
        config["active_joints"],
        config.get("locked_joints", {}),
        config.get("limits", {}),
        config.get("max_velocity", {}),
        config.get("home_positions", {}),
        trajectory_interpolation=interpolation,
        clock=clock.time,
    )


class LivePlotter:
    def __init__(self, refresh_rate_hz: float):
        self.refresh_period = 1.0 / max(0.1, float(refresh_rate_hz))
        self.last_update_t = -float("inf")
        plt.ion()
        self.fig = plt.figure(figsize=(10, 7))
        self.ax_xz = self.fig.add_subplot(2, 1, 1)
        self.ax_err = self.fig.add_subplot(2, 1, 2)
        (self.target_line,) = self.ax_xz.plot([], [], label="target_xz")
        (self.ee_line,) = self.ax_xz.plot([], [], label="ee_xz")
        (self.err_line,) = self.ax_err.plot([], [], label="tracking_error_xz")
        self.ax_xz.set_title("SO101 Planar MuJoCo Control Simulation")
        self.ax_xz.set_xlabel("x [m]")
        self.ax_xz.set_ylabel("z [m]")
        self.ax_xz.legend()
        self.ax_err.set_xlabel("time [s]")
        self.ax_err.set_ylabel("error [m]")
        self.ax_err.legend()
        self.fig.tight_layout()
        self.fig.show()

    def update(self, rows: list[dict[str, float]]) -> None:
        if not rows:
            return
        elapsed = float(rows[-1]["elapsed"])
        if elapsed - self.last_update_t < self.refresh_period:
            return
        self.last_update_t = elapsed

        target_x = [row["target_x"] for row in rows]
        target_z = [row["target_z"] for row in rows]
        ee_x = [row["ee_x"] for row in rows]
        ee_z = [row["ee_z"] for row in rows]
        time_s = [row["elapsed"] for row in rows]
        error = [row["tracking_error_xz_m"] for row in rows]

        self.target_line.set_data(target_x, target_z)
        self.ee_line.set_data(ee_x, ee_z)
        self.err_line.set_data(time_s, error)
        self.ax_xz.relim()
        self.ax_xz.autoscale_view()
        self.ax_xz.axis("equal")
        self.ax_err.relim()
        self.ax_err.autoscale_view()
        self.fig.canvas.draw_idle()
        plt.pause(0.001)


def update_viewer_markers(viewer, target: np.ndarray, ee: np.ndarray) -> None:
    if viewer is None:
        return
    try:
        scene = viewer.user_scn
    except AttributeError:
        return

    def _set_markers() -> None:
        scene.ngeom = 0
        mat = np.eye(3, dtype=np.float64).reshape(-1)
        specs = [
            (target, (1.0, 0.1, 0.1, 0.9), 0.012),
            (ee, (0.1, 0.9, 0.2, 0.9), 0.009),
        ]
        for pos, rgba, radius in specs:
            if scene.ngeom >= scene.maxgeom:
                return
            mujoco.mjv_initGeom(
                scene.geoms[scene.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                np.array([radius, radius, radius], dtype=np.float64),
                np.asarray(pos, dtype=np.float64),
                mat,
                np.asarray(rgba, dtype=np.float32),
            )
            scene.ngeom += 1

    try:
        lock = getattr(viewer, "lock", None)
        if lock is None:
            _set_markers()
        else:
            with lock():
                _set_markers()
    except Exception:
        return


def write_outputs(rows: list[dict[str, float]], plan_rows: list[dict[str, float]], args: argparse.Namespace) -> None:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    controller_name = getattr(args, "controller_name", args.execution_mode)
    csv_path = args.output_dir / ("%s.csv" % controller_name)
    with csv_path.open("w", newline="") as handle:
        keys = sorted({key for row in rows for key in row})
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)

    if plan_rows:
        plan_path = args.output_dir / ("%s_plan.csv" % controller_name)
        with plan_path.open("w", newline="") as handle:
            keys = sorted({key for row in plan_rows for key in row})
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            writer.writerows(plan_rows)

    elapsed = [row["elapsed"] for row in rows]
    error_xz = [row["tracking_error_xz_m"] for row in rows]
    fig = plt.figure(figsize=(10, 7))
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.plot([row["target_x"] for row in rows], [row["target_z"] for row in rows], label="target_xz")
    ax1.plot([row["ee_x"] for row in rows], [row["ee_z"] for row in rows], label="ee_xz")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("z [m]")
    ax1.set_title("SO101 Planar MuJoCo Control Simulation")
    ax1.axis("equal")
    ax1.legend()

    ax2 = fig.add_subplot(2, 1, 2)
    ax2.plot(elapsed, error_xz, label="tracking_error_xz")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("error [m]")
    ax2.legend()
    fig.tight_layout()
    plot_path = args.output_dir / ("%s_tracking.png" % controller_name)
    fig.savefig(plot_path, dpi=160)

    video_path = None
    if getattr(args, "record_video", False):
        recorded_frames = getattr(args, "recorded_frames", None)
        if recorded_frames:
            video_path = args.output_dir / ("%s_tracking.mp4" % controller_name)
            write_mp4(recorded_frames, video_path, fps=getattr(args, "video_fps", 30))

    errors = np.array(error_xz, dtype=np.float64)
    print("controller=%s" % controller_name)
    print("duration_s=%.3f" % float(args.duration))
    print("trajectory_cycles=%.3f" % float(args.duration * args.frequency))
    print("samples=%d" % len(rows))
    print("mean_tracking_error_xz_m=%.6f" % float(errors.mean()))
    print("max_tracking_error_xz_m=%.6f" % float(errors.max()))
    print("final_tracking_error_xz_m=%.6f" % float(errors[-1]))
    print("csv=%s" % csv_path)
    print("plot=%s" % plot_path)
    if video_path is not None:
        print("video=%s" % video_path)


def run(args: argparse.Namespace) -> None:
    resolve_run_settings(args)
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.plan_rate <= 0.0 or args.control_rate <= 0.0:
        raise ValueError("--plan-rate and --control-rate must be positive")

    config = load_config(args.config)
    model, data, ee_ref = make_model(args)
    maps = model_maps(model, config["joint_order"])
    config = apply_limit_profile(config, maps, args)
    interpolation = args.trajectory_interpolation or config.get("trajectory_interpolation", "cubic")
    command_duration = args.command_duration
    if command_duration is None:
        command_duration = float(config.get("command_duration_s", 0.04))

    prep = complete_positions(config, SAFE_POSES[args.prep_pose])
    write_qpos(model, data, maps, prep)
    write_ctrl(model, data, maps, prep)
    mujoco.mj_forward(model, data)
    initial_ee = ee_position(data, ee_ref)
    center = np.array(args.center, dtype=np.float64) if args.center is not None else initial_ee.copy()
    center[1] = initial_ee[1]

    planning_data = mujoco.MjData(model)
    command_names, timed_positions, plan_rows = build_planned_path(
        model, planning_data, maps, config, ee_ref, center, prep, args
    )
    controller_name = args.controller_name
    if controller_name == "argo_external" and args.ee_frame != "body_gripper":
        raise ValueError("--controller argo_external requires --ee-frame body_gripper for a fair frame match")
    if controller_name == "moveit_like":
        timed_positions = retime_timed_positions(
            timed_positions,
            config.get("max_velocity", {}),
            args.moveit_velocity_scaling,
        )
    elif controller_name == "moveit_ruckig":
        timed_positions = ruckig_retime_timed_positions(
            timed_positions,
            command_names,
            config.get("max_velocity", {}),
            1.0 / args.control_rate,
            args,
        )
    start_positions = dict(prep)
    if args.start_at_first_target and timed_positions:
        start_positions = complete_positions(config, timed_positions[0][1])

    execution_duration = float(timed_positions[-1][0]) if timed_positions else float(args.duration)
    execution_duration = max(execution_duration, float(args.duration))

    write_qpos(model, data, maps, start_positions)
    write_ctrl(model, data, maps, start_positions)
    mujoco.mj_forward(model, data)

    clock = SimClock()
    control_filter = make_filter(config, interpolation, clock)
    control_filter.freeze(read_control_positions(model, data, maps, config["joint_order"]))

    next_stream_t = 0.0
    stream_seed = dict(start_positions)
    argo_context = make_argo_context() if controller_name == "argo_external" else None
    if controller_name in {"joint_trajectory", "moveit_like", "moveit_ruckig"}:
        control_filter.set_timed_trajectory(command_names, timed_positions)

    cartesian_servo = None
    servo_prev_target = None
    if controller_name == "servo":
        # Reuse the exact resolved-rate servo law that the ROS servo node runs.
        # Fidelity note: this uses the URDF finite-difference Jacobian from the
        # shared kinematics module, not MuJoCo's analytic mj_jacSite. That is the
        # point -- it validates the same code path that runs on hardware. The DLS
        # controllers above (cartesian_stream/argo_like) use the MuJoCo Jacobian
        # and remain available for an analytic-Jacobian comparison.
        servo_urdf = args.urdf_path.read_text()
        servo_kin = SO101Kinematics.from_urdf(
            servo_urdf,
            base_link="base_link",
            tip_link="gripper_frame_link",
            limits_override=config.get("limits", {}),
        )
        cartesian_servo = PlanarCartesianServo(
            servo_kin,
            config["active_joints"],
            config.get("locked_joints", {}),
            config.get("limits", {}),
            config.get("max_velocity", {}),
            workspace_limits=config.get("workspace_limits", {}),
            axes=PLANAR_AXES,
            position_gain=args.servo_position_gain,
            max_ee_speed=args.servo_max_ee_speed,
            ik_damping=args.servo_ik_damping,
            accel_scale=args.servo_accel_scale,
            jerk_scale=args.servo_jerk_scale,
            control_dt=1.0 / args.control_rate,
            prefer_ruckig=args.servo_prefer_ruckig,
        )
        cartesian_servo.reset(read_control_positions(model, data, maps, config["joint_order"]))
        # The servo controls its own URDF FK frame; the tracking metric measures
        # the MuJoCo EE frame. Measure their constant offset at the start pose so
        # targets given in the MuJoCo frame map correctly into the servo frame.
        servo_frame_offset = ee_position(data, ee_ref) - np.asarray(cartesian_servo.p_cmd, dtype=np.float64)

    control_dt = 1.0 / args.control_rate
    sim_steps = int(round(control_dt / model.opt.timestep))
    sim_steps = max(1, sim_steps)
    log_every = max(1, int(round(args.control_rate / max(args.log_rate, 1e-9))))
    total_steps = int(math.ceil((execution_duration + args.post_hold) / control_dt))
    rows = []
    args.recorded_frames = []
    video_frame_dt = 1.0 / max(1, int(getattr(args, "video_fps", 30)))
    next_video_frame_t = 0.0

    viewer_context = None
    viewer = None
    renderer = None
    if args.viewer:
        from mujoco import viewer as mujoco_viewer

        viewer_context = mujoco_viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True)
        viewer = viewer_context.__enter__()
    if args.record_video:
        renderer = mujoco.Renderer(model, height=480, width=640)

    live_plotter = None
    if args.live_plot:
        try:
            live_plotter = LivePlotter(args.plot_rate)
        except Exception as exc:
            print("Warning: live plot disabled: %s" % exc, file=sys.stderr)

    try:
        wall_start = time.perf_counter()
        for step in range(total_steps + 1):
            elapsed = clock.time()
            if controller_name in {"moveit_like", "moveit_ruckig"} and execution_duration > 1e-9:
                target_elapsed = min(args.duration, elapsed * args.duration / execution_duration)
            else:
                target_elapsed = min(elapsed, args.duration)
            target = target_at(center, args, target_elapsed)

            if controller_name == "cartesian_stream" and elapsed >= next_stream_t and elapsed <= execution_duration:
                ok, solution, _ik_err, _iters = solve_planar_dls_ik(
                    model,
                    planning_data,
                    maps,
                    config,
                    ee_ref,
                    target,
                    stream_seed,
                    args,
                    multi_start=args.stream_multistart_every_target,
                )
                if ok:
                    stream_seed = dict(solution)
                    point = {name: float(solution[name]) for name in command_names if name in solution}
                    control_filter.set_target_positions(point, duration_s=command_duration)
                next_stream_t += 1.0 / args.plan_rate

            if controller_name == "argo_like":
                if elapsed <= execution_duration:
                    ok, solution, _ik_err, _iters = solve_planar_dls_ik(
                        model,
                        planning_data,
                        maps,
                        config,
                        ee_ref,
                        target,
                        stream_seed,
                        args,
                        multi_start=False,
                    )
                    if ok:
                        stream_seed = dict(solution)
                desired = complete_positions(config, stream_seed)
            elif controller_name == "argo_external":
                if elapsed <= execution_duration:
                    ok, solution, _ik_err, _iters = solve_argo_external_ik(
                        config,
                        target,
                        stream_seed,
                        argo_context,
                        args,
                    )
                    if ok:
                        stream_seed = dict(solution)
                desired = complete_positions(config, stream_seed)
            elif controller_name == "servo":
                measured = read_control_positions(model, data, maps, config["joint_order"])
                # Map the MuJoCo-frame target into the servo's URDF FK frame.
                servo_target = (np.asarray(target, dtype=np.float64) - servo_frame_offset)
                position_target = None
                velocity_cmd = None
                if elapsed <= execution_duration:
                    if args.servo_input == "velocity":
                        if servo_prev_target is None:
                            servo_prev_target = servo_target.copy()
                        velocity_cmd = list((servo_target - servo_prev_target) / control_dt)
                        servo_prev_target = servo_target.copy()
                    else:
                        position_target = list(servo_target)
                else:
                    # Hold the last commanded EE setpoint during --post-hold.
                    position_target = list(cartesian_servo.p_cmd)
                q_cmd = cartesian_servo.step(
                    control_dt,
                    measured_positions=measured,
                    position_target=position_target,
                    velocity_cmd=velocity_cmd,
                )
                desired = complete_positions(config, q_cmd)
            else:
                desired = control_filter.step(control_dt)
            write_ctrl(model, data, maps, desired)
            for _ in range(sim_steps):
                mujoco.mj_step(model, data)

            if step % log_every == 0:
                ee = ee_position(data, ee_ref)
                actual = read_control_positions(model, data, maps, config["joint_order"])
                plan_row = nearest_plan_row(plan_rows, target_elapsed)
                error_xz = target[list(PLANAR_AXES)] - ee[list(PLANAR_AXES)]
                error_xyz = target - ee
                row = {
                    "elapsed": elapsed,
                    "target_elapsed": target_elapsed,
                    "controller": controller_name,
                    "execution_mode": args.execution_mode,
                    "target_x": float(target[0]),
                    "target_y": float(target[1]),
                    "target_z": float(target[2]),
                    "ee_x": float(ee[0]),
                    "ee_y": float(ee[1]),
                    "ee_z": float(ee[2]),
                    "tracking_error_xz_m": float(np.linalg.norm(error_xz)),
                    "tracking_error_xyz_m": float(np.linalg.norm(error_xyz)),
                    "command_duration_s": float(command_duration),
                }
                for name in config["joint_order"]:
                    row[name] = float(actual.get(name, 0.0))
                    row["commanded_" + name] = float(desired.get(name, 0.0))
                    row["joint_tracking_error_" + name] = row[name] - row["commanded_" + name]
                for key, value in plan_row.items():
                    if key.startswith("planned_") or key.startswith("ik_"):
                        row[key] = float(value)
                rows.append(row)
                if live_plotter is not None:
                    live_plotter.update(rows)

            if viewer is not None:
                if not viewer.is_running():
                    break
                if args.viewer_markers:
                    update_viewer_markers(viewer, target, ee_position(data, ee_ref))
                viewer.sync()
                if args.viewer_real_time:
                    expected = elapsed / max(args.speed, 1e-6)
                    sleep_s = expected - (time.perf_counter() - wall_start)
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)

            if renderer is not None and elapsed + 1e-12 >= next_video_frame_t:
                renderer.update_scene(data)
                args.recorded_frames.append(renderer.render())
                next_video_frame_t += video_frame_dt

            clock.advance(control_dt)
    finally:
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)
        if renderer is not None:
            renderer.close()

    write_outputs(rows, plan_rows, args)


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()

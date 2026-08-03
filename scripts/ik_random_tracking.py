from __future__ import annotations

import argparse
import csv
import os
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt

SCENE_PATH = ROOT / "assets" / "so101" / "scene.xml"
ARM_DOF = 5
HOME_QPOS = np.array([0.0, 0.25, -0.45, 0.45, 0.0, 0.45], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone DLS IK tracking of smooth random 3D SO-101 endpoint trajectories."
    )
    parser.add_argument("--segments", type=int, default=8, help="Number of random trajectory segments.")
    parser.add_argument("--steps-per-segment", type=int, default=80)
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--ik-iters", type=int, default=3)
    parser.add_argument("--gain", type=float, default=0.8)
    parser.add_argument("--damping", type=float, default=0.03)
    parser.add_argument("--max-dq", type=float, default=0.06, help="Max joint update per IK iteration [rad].")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--viewer", action="store_true", help="Show MuJoCo motion while IK tracks the trajectory.")
    parser.add_argument("--real-time", action="store_true", help="Throttle viewer playback to MuJoCo sim time.")
    parser.add_argument("--speed", type=float, default=1.0, help="Viewer playback speed when --real-time is set.")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "ik_random_tracking")
    return parser.parse_args()


def smoothstep(t: np.ndarray) -> np.ndarray:
    return t * t * (3.0 - 2.0 * t)


def sample_waypoints(rng: np.random.Generator, segments: int, start: np.ndarray) -> np.ndarray:
    """Sample conservative reachable waypoints around the current stage-1 workspace."""
    center = np.array([0.31, 0.0, 0.20], dtype=np.float64)
    half_range = np.array([0.045, 0.055, 0.030], dtype=np.float64)
    waypoints = center + rng.uniform(-1.0, 1.0, size=(segments + 1, 3)) * half_range
    waypoints[:, 2] = np.clip(waypoints[:, 2], 0.15, 0.24)
    waypoints[0] = start
    return waypoints


def build_target_trajectory(waypoints: np.ndarray, steps_per_segment: int) -> np.ndarray:
    targets = []
    for start, end in zip(waypoints[:-1], waypoints[1:], strict=True):
        alpha = smoothstep(np.linspace(0.0, 1.0, steps_per_segment, endpoint=False))[:, None]
        targets.append(start + alpha * (end - start))
    targets.append(waypoints[-1][None, :])
    return np.vstack(targets)


def dls_step(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    target: np.ndarray,
    damping: float,
) -> tuple[np.ndarray, np.ndarray]:
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, jacp, None, site_id)
    jac = jacp[:, :ARM_DOF]
    err = target - data.site_xpos[site_id]
    lhs = jac @ jac.T + (damping**2) * np.eye(3)
    dq = jac.T @ np.linalg.solve(lhs, err)
    return dq, err


def step_ik_control(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    target: np.ndarray,
    home: np.ndarray,
    ctrl_min: np.ndarray,
    ctrl_max: np.ndarray,
    args: argparse.Namespace,
) -> None:
    for _ in range(args.ik_iters):
        dq, _ = dls_step(model, data, site_id, target, args.damping)
        dq = np.clip(args.gain * dq, -args.max_dq, args.max_dq)
        data.ctrl[:ARM_DOF] = np.clip(data.qpos[:ARM_DOF] + dq, ctrl_min[:ARM_DOF], ctrl_max[:ARM_DOF])
        data.ctrl[ARM_DOF] = home[ARM_DOF]
        for _ in range(args.frame_skip):
            mujoco.mj_step(model, data)


def write_outputs(rows: list[dict[str, float | int]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "ik_random_tracking.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot([r["target_x"] for r in rows], [r["target_y"] for r in rows], [r["target_z"] for r in rows], label="target")
    ax.plot([r["ee_x"] for r in rows], [r["ee_y"] for r in rows], [r["ee_z"] for r in rows], label="dls_ik")
    ax.set_title("SO-101 Random 3D Trajectory Tracking with DLS IK")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend()
    fig.tight_layout()
    plot_path = output_dir / "ik_random_tracking.png"
    fig.savefig(plot_path, dpi=160)

    errors = np.array([r["tracking_error"] for r in rows], dtype=np.float64)
    print(f"mean_tracking_error={float(errors.mean()):.4f} m")
    print(f"final_tracking_error={float(errors[-1]):.4f} m")
    print(f"max_tracking_error={float(errors.max()):.4f} m")
    print(f"csv={csv_path}")
    print(f"plot={plot_path}")


def main() -> None:
    args = parse_args()
    rng = np.random.default_rng(args.seed)

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    if site_id < 0:
        raise ValueError("site 'gripperframe' not found in SO-101 model")

    ctrl_min = model.actuator_ctrlrange[:, 0].copy()
    ctrl_max = model.actuator_ctrlrange[:, 1].copy()
    home = np.clip(HOME_QPOS, ctrl_min, ctrl_max)

    mujoco.mj_resetData(model, data)
    data.qpos[:] = home
    data.ctrl[:] = home
    mujoco.mj_forward(model, data)
    targets = build_target_trajectory(
        sample_waypoints(rng, args.segments, data.site_xpos[site_id].copy()),
        args.steps_per_segment,
    )

    rows: list[dict[str, float | int]] = []

    def run_step(step: int, target: np.ndarray) -> None:
        step_ik_control(model, data, site_id, target, home, ctrl_min, ctrl_max, args)
        ee = data.site_xpos[site_id].copy()
        err = float(np.linalg.norm(target - ee))
        rows.append(
            {
                "step": step,
                "tracking_error": err,
                "ee_x": float(ee[0]),
                "ee_y": float(ee[1]),
                "ee_z": float(ee[2]),
                "target_x": float(target[0]),
                "target_y": float(target[1]),
                "target_z": float(target[2]),
            }
        )

    if args.viewer:
        from mujoco import viewer as mujoco_viewer

        speed = max(float(args.speed), 1e-6)
        last_wall = time.perf_counter()
        last_sim = float(data.time)
        with mujoco_viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
            for step, target in enumerate(targets):
                if not viewer.is_running():
                    break
                run_step(step, target)
                if args.real_time:
                    now_wall = time.perf_counter()
                    now_sim = float(data.time)
                    sleep_s = (now_sim - last_sim) / speed - (now_wall - last_wall)
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    last_wall = time.perf_counter()
                    last_sim = now_sim
                viewer.sync()
    else:
        for step, target in enumerate(targets):
            run_step(step, target)

    write_outputs(rows, args.output_dir)


if __name__ == "__main__":
    main()

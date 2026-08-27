from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt

from video_utils import write_mp4

SCENE_PATH = ROOT / "assets" / "so101" / "scene.xml"
ARM_DOF = 5
HOME_QPOS = np.array([0.0, 0.25, -0.45, 0.45, 0.0, 0.45], dtype=np.float64)


def target_at(step: int, episode_steps: int) -> np.ndarray:
    phase = 2.0 * np.pi * step / max(1, episode_steps)
    radius = 0.055
    center = np.array([0.31, 0.0, 0.20], dtype=np.float64)
    return center + np.array(
        [
            radius * np.cos(phase),
            radius * np.sin(phase),
            0.025 * np.sin(2.0 * phase),
        ],
        dtype=np.float64,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Damped-least-squares IK baseline for SO-101 tracking.")
    parser.add_argument("--episode-steps", type=int, default=300)
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--ik-iters", type=int, default=3)
    parser.add_argument("--gain", type=float, default=0.8)
    parser.add_argument("--damping", type=float, default=0.03)
    parser.add_argument("--max-dq", type=float, default=0.06, help="Max joint update per control step [rad].")
    parser.add_argument("--record-video", action="store_true", help="Record the baseline rollout to mp4.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate for --record-video outputs.")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "ik_baseline")
    return parser.parse_args()


def dls_step(model: mujoco.MjModel, data: mujoco.MjData, site_id: int, target: np.ndarray, damping: float):
    jacp = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, jacp, None, site_id)
    jac = jacp[:, :ARM_DOF]
    err = target - data.site_xpos[site_id]
    lhs = jac @ jac.T + (damping**2) * np.eye(3)
    dq = jac.T @ np.linalg.solve(lhs, err)
    return dq, err


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
    ctrl_min = model.actuator_ctrlrange[:, 0].copy()
    ctrl_max = model.actuator_ctrlrange[:, 1].copy()
    home = np.clip(HOME_QPOS, ctrl_min, ctrl_max)

    mujoco.mj_resetData(model, data)
    data.qpos[:] = home
    data.ctrl[:] = home
    mujoco.mj_forward(model, data)

    rows = []
    frames: list[np.ndarray] = []
    renderer = mujoco.Renderer(model, height=480, width=640) if args.record_video else None
    try:
        for step in range(args.episode_steps):
            target = target_at(step, args.episode_steps)
            for _ in range(args.ik_iters):
                dq, _ = dls_step(model, data, site_id, target, args.damping)
                dq = np.clip(args.gain * dq, -args.max_dq, args.max_dq)
                data.ctrl[:ARM_DOF] = np.clip(data.qpos[:ARM_DOF] + dq, ctrl_min[:ARM_DOF], ctrl_max[:ARM_DOF])
                data.ctrl[ARM_DOF] = home[ARM_DOF]
                for _ in range(args.frame_skip):
                    mujoco.mj_step(model, data)

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
            if renderer is not None:
                renderer.update_scene(data)
                frames.append(renderer.render())
    finally:
        if renderer is not None:
            renderer.close()

    csv_path = args.output_dir / "ik_tracking.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot([r["target_x"] for r in rows], [r["target_y"] for r in rows], [r["target_z"] for r in rows], label="target")
    ax.plot([r["ee_x"] for r in rows], [r["ee_y"] for r in rows], [r["ee_z"] for r in rows], label="ik")
    ax.set_title("SO-101 DLS IK Tracking Baseline")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend()
    fig.tight_layout()
    plot_path = args.output_dir / "ik_tracking.png"
    fig.savefig(plot_path, dpi=160)

    errors = np.array([r["tracking_error"] for r in rows], dtype=np.float64)
    print(f"mean_tracking_error={float(errors.mean()):.4f} m")
    print(f"final_tracking_error={float(errors[-1]):.4f} m")
    print(f"csv={csv_path}")
    print(f"plot={plot_path}")
    if args.record_video and frames:
        video_path = args.output_dir / "ik_tracking.mp4"
        write_mp4(frames, video_path, fps=args.video_fps)
        print(f"video={video_path}")


if __name__ == "__main__":
    main()

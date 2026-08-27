from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys

import numpy as np
from stable_baselines3 import SAC

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt

sys.path.insert(0, str(ROOT))

from so101_tracking import SO101TrackingEnv  # noqa: E402
from video_utils import write_mp4


def find_latest_model() -> Path:
    candidates = list((ROOT / "outputs" / "sac").glob("*/models/best_model.zip"))
    if not candidates:
        candidates = list((ROOT / "outputs" / "sac").glob("*/models/final_model.zip"))
    if not candidates:
        raise FileNotFoundError("No SAC model found under so101_mujoco_tracking/outputs/sac")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a trained SO-101 SAC tracking policy.")
    parser.add_argument("--model", type=Path, default=None, help="Path to .zip model. Defaults to latest.")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--episode-steps", type=int, default=300)
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--control-mode", choices=["joint_delta", "ik_residual"], default="ik_residual")
    parser.add_argument("--action-scale", type=float, default=0.04)
    parser.add_argument("--residual-scale", type=float, default=0.015)
    parser.add_argument("--ik-iters", type=int, default=3)
    parser.add_argument("--ik-gain", type=float, default=0.8)
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument("--ik-max-dq", type=float, default=0.06)
    parser.add_argument("--trajectory-mode", choices=["lissajous", "random"], default="lissajous")
    parser.add_argument("--random-segments", type=int, default=6)
    parser.add_argument("--random-center", type=float, nargs=3, default=(0.31, 0.0, 0.20))
    parser.add_argument("--random-half-range", type=float, nargs=3, default=(0.045, 0.055, 0.030))
    parser.add_argument("--deterministic", action="store_true", default=True)
    parser.add_argument("--record-video", action="store_true", help="Record the first evaluation episode to mp4.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate for --record-video outputs.")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "eval")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_path = args.model or find_latest_model()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    env = SO101TrackingEnv(
        episode_steps=args.episode_steps,
        frame_skip=args.frame_skip,
        action_scale=args.action_scale,
        control_mode=args.control_mode,
        residual_scale=args.residual_scale,
        ik_iters=args.ik_iters,
        ik_gain=args.ik_gain,
        ik_damping=args.ik_damping,
        ik_max_dq=args.ik_max_dq,
        trajectory_mode=args.trajectory_mode,
        random_segments=args.random_segments,
        random_center=tuple(args.random_center),
        random_half_range=tuple(args.random_half_range),
        render_mode="rgb_array" if args.record_video else None,
    )
    model = SAC.load(model_path, env=env, device="auto")

    rows: list[dict[str, float | int]] = []
    video_frames: list[np.ndarray] = []
    episode_errors = []
    episode_max_errors = []
    episode_final_errors = []
    for ep in range(args.episodes):
        obs, info = env.reset(seed=ep)
        done = False
        step = 0
        errors = []
        while not done:
            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            ee = info["ee_pos"]
            target = info["target_pos"]
            err = float(info["tracking_error"])
            errors.append(err)
            rows.append(
                {
                    "episode": ep,
                    "step": step,
                    "reward": float(reward),
                    "tracking_error": err,
                    "ee_x": float(ee[0]),
                    "ee_y": float(ee[1]),
                    "ee_z": float(ee[2]),
                    "target_x": float(target[0]),
                    "target_y": float(target[1]),
                    "target_z": float(target[2]),
                }
            )
            if args.record_video and ep == 0:
                frame = env.render()
                if frame is not None:
                    video_frames.append(frame)
            step += 1
        episode_errors.append(float(np.mean(errors)))
        episode_max_errors.append(float(np.max(errors)))
        episode_final_errors.append(float(errors[-1]))

    csv_path = args.output_dir / "tracking_eval.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    first_ep = [r for r in rows if r["episode"] == 0]
    ax.plot([r["target_x"] for r in first_ep], [r["target_y"] for r in first_ep], [r["target_z"] for r in first_ep], label="target")
    ax.plot([r["ee_x"] for r in first_ep], [r["ee_y"] for r in first_ep], [r["ee_z"] for r in first_ep], label="ee")
    ax.set_title("SO-101 Endpoint Tracking")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend()
    fig.tight_layout()
    plot_path = args.output_dir / "tracking_eval_episode0.png"
    fig.savefig(plot_path, dpi=160)

    print(f"model={model_path}")
    print(f"mean_episode_error={float(np.mean(episode_errors)):.4f} m")
    print(f"mean_max_error={float(np.mean(episode_max_errors)):.4f} m")
    print(f"mean_final_error={float(np.mean(episode_final_errors)):.4f} m")
    print(f"csv={csv_path}")
    print(f"plot={plot_path}")
    if args.record_video and video_frames:
        video_path = args.output_dir / "tracking_eval_episode0.mp4"
        write_mp4(video_frames, video_path, fps=args.video_fps)
        print(f"video={video_path}")
    env.close()


if __name__ == "__main__":
    main()

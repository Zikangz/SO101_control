from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import time

import numpy as np
from stable_baselines3 import SAC

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt

sys.path.insert(0, str(ROOT))

from so101_tracking import SO101PickLiftEnv  # noqa: E402


def find_latest_model() -> Path:
    base = ROOT / "outputs" / "pick_lift_sac"
    candidates = list(base.glob("*/models/best_model.zip"))
    if not candidates:
        candidates = list(base.glob("*/models/final_model.zip"))
    if not candidates:
        raise FileNotFoundError("No SAC model found under so101_mujoco_tracking/outputs/pick_lift_sac")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate a trained SO-101 pick-and-lift SAC policy.")
    parser.add_argument("--model", type=Path, default=None, help="Path to .zip model. Defaults to latest.")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--episode-steps", type=int, default=200)
    parser.add_argument("--frame-skip", type=int, default=5)
    parser.add_argument("--control-mode", choices=["ee_delta", "joint_delta"], default="ee_delta")
    parser.add_argument("--ee-action-scale", type=float, default=0.025)
    parser.add_argument("--joint-action-scale", type=float, default=0.04)
    parser.add_argument("--ik-gain", type=float, default=0.8)
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument("--ik-max-dq", type=float, default=0.06)
    parser.add_argument("--cube-xy-center", type=float, nargs=2, default=(0.34, 0.0))
    parser.add_argument("--cube-xy-range", type=float, nargs=2, default=(0.045, 0.045))
    parser.add_argument("--lift-height", type=float, default=0.12)
    parser.add_argument("--virtual-grasp", dest="virtual_grasp", action="store_true", default=True)
    parser.add_argument("--no-virtual-grasp", dest="virtual_grasp", action="store_false")
    parser.add_argument("--grasp-threshold", type=float, default=0.045)
    parser.add_argument("--deterministic", action="store_true", default=True)
    parser.add_argument("--viewer", action="store_true", help="Show MuJoCo viewer during evaluation.")
    parser.add_argument("--real-time", action="store_true", help="Throttle viewer to simulation time.")
    parser.add_argument("--viewer-speed", type=float, default=1.0)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "eval_pick_lift")
    return parser.parse_args()


def make_env(args: argparse.Namespace) -> SO101PickLiftEnv:
    return SO101PickLiftEnv(
        episode_steps=args.episode_steps,
        frame_skip=args.frame_skip,
        control_mode=args.control_mode,
        ee_action_scale=args.ee_action_scale,
        joint_action_scale=args.joint_action_scale,
        ik_gain=args.ik_gain,
        ik_damping=args.ik_damping,
        ik_max_dq=args.ik_max_dq,
        cube_xy_center=tuple(args.cube_xy_center),
        cube_xy_range=tuple(args.cube_xy_range),
        lift_height=args.lift_height,
        virtual_grasp=args.virtual_grasp,
        grasp_threshold=args.grasp_threshold,
    )


def maybe_sync_viewer(env: SO101PickLiftEnv, viewer, args: argparse.Namespace) -> None:
    if viewer is None:
        return
    viewer.sync()
    if args.real_time:
        dt = env.model.opt.timestep * env.frame_skip / max(args.viewer_speed, 1e-6)
        time.sleep(dt)


def main() -> None:
    args = parse_args()
    model_path = args.model or find_latest_model()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    env = make_env(args)
    model = SAC.load(model_path, env=env, device="auto")

    viewer_context = None
    viewer = None
    if args.viewer:
        import mujoco.viewer

        viewer_context = mujoco.viewer.launch_passive(
            env.model,
            env.data,
            show_left_ui=True,
            show_right_ui=True,
        )
        viewer = viewer_context.__enter__()

    rows: list[dict[str, float | int | bool]] = []
    ep_rewards: list[float] = []
    ep_successes: list[float] = []
    ep_max_heights: list[float] = []

    try:
        for ep in range(args.episodes):
            obs, info = env.reset(seed=ep)
            done = False
            step = 0
            rewards = []
            max_height = float(info["cube_height"])
            success = False
            maybe_sync_viewer(env, viewer, args)

            while not done:
                action, _ = model.predict(obs, deterministic=args.deterministic)
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                rewards.append(float(reward))
                max_height = max(max_height, float(info["cube_height"]))
                success = success or bool(info["success"])

                ee = info["ee_pos"]
                cube = info["cube_pos"]
                goal = info["goal_pos"]
                rows.append(
                    {
                        "episode": ep,
                        "step": step,
                        "reward": float(reward),
                        "success": bool(info["success"]),
                        "is_grasped": bool(info["is_grasped"]),
                        "cube_height": float(info["cube_height"]),
                        "reach_dist": float(info.get("reach_dist", np.nan)),
                        "goal_dist": float(info.get("goal_dist", np.nan)),
                        "ee_x": float(ee[0]),
                        "ee_y": float(ee[1]),
                        "ee_z": float(ee[2]),
                        "cube_x": float(cube[0]),
                        "cube_y": float(cube[1]),
                        "cube_z": float(cube[2]),
                        "goal_x": float(goal[0]),
                        "goal_y": float(goal[1]),
                        "goal_z": float(goal[2]),
                    }
                )
                step += 1
                maybe_sync_viewer(env, viewer, args)

            ep_rewards.append(float(np.sum(rewards)))
            ep_successes.append(float(success))
            ep_max_heights.append(max_height)
    finally:
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)

    csv_path = args.output_dir / "pick_lift_eval.csv"
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    first_ep = [r for r in rows if r["episode"] == 0]
    steps = [int(r["step"]) for r in first_ep]
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    axes[0].plot(steps, [float(r["cube_height"]) for r in first_ep], label="cube height")
    axes[0].axhline(args.lift_height, color="tab:green", linestyle="--", label="target lift")
    axes[0].set_ylabel("height [m]")
    axes[0].legend()
    axes[1].plot(steps, [float(r["reach_dist"]) for r in first_ep], label="ee-cube distance")
    axes[1].plot(steps, [float(r["goal_dist"]) for r in first_ep], label="cube-goal distance")
    axes[1].set_ylabel("distance [m]")
    axes[1].legend()
    axes[2].plot(steps, [float(r["is_grasped"]) for r in first_ep], label="grasped")
    axes[2].plot(steps, [float(r["success"]) for r in first_ep], label="success")
    axes[2].set_ylabel("flag")
    axes[2].set_xlabel("step")
    axes[2].legend()
    fig.tight_layout()
    plot_path = args.output_dir / "pick_lift_eval_episode0.png"
    fig.savefig(plot_path, dpi=160)

    print(f"model={model_path}")
    print(f"episodes={args.episodes}")
    print(f"success_rate={float(np.mean(ep_successes)):.3f}")
    print(f"mean_return={float(np.mean(ep_rewards)):.3f}")
    print(f"mean_max_cube_height={float(np.mean(ep_max_heights)):.4f} m")
    print(f"csv={csv_path}")
    print(f"plot={plot_path}")
    env.close()


if __name__ == "__main__":
    main()

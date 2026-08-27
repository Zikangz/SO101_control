from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import mujoco
import numpy as np
import torch as th
import torch.nn.functional as F
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from so101_tracking import SO101PickLiftEnv  # noqa: E402
from video_utils import write_mp4


class PickLiftMetricsCallback(BaseCallback):
    """Log pick-and-lift task metrics from env info."""

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        if not infos:
            return True
        info = infos[0]
        for key in (
            "success",
            "is_grasped",
            "is_closed_cmd",
            "cube_height",
            "reach_dist",
            "hover_dist",
            "grasp_dist",
            "goal_dist",
            "lift_progress",
        ):
            if key in info:
                self.logger.record(f"pick_lift/{key}", float(info[key]))
        return True


class MujocoViewerCallback(BaseCallback):
    """Synchronize MuJoCo's passive viewer with the training environment."""

    def __init__(
        self,
        sync_every: int = 1,
        stop_if_closed: bool = False,
        real_time: bool = False,
        speed: float = 1.0,
    ) -> None:
        super().__init__()
        self.sync_every = max(1, int(sync_every))
        self.stop_if_closed = stop_if_closed
        self.real_time = real_time
        self.speed = max(float(speed), 1e-6)
        self.viewer = None
        self.base_env = None
        self._last_wall_time = None
        self._last_sim_time = None

    def _unwrap_env(self):
        env = self.training_env.envs[0]
        while hasattr(env, "env"):
            env = env.env
        return env

    def _on_training_start(self) -> None:
        import mujoco.viewer

        self.base_env = self._unwrap_env()
        self.viewer = mujoco.viewer.launch_passive(
            self.base_env.model,
            self.base_env.data,
            show_left_ui=True,
            show_right_ui=True,
        )
        self._last_wall_time = time.perf_counter()
        self._last_sim_time = float(self.base_env.data.time)
        mode = "real-time" if self.real_time else "fast"
        print(f"MuJoCo viewer launched in {mode} mode. Close the window to hide visualization.")

    def _maybe_sleep_to_sim_time(self) -> None:
        if not self.real_time or self.base_env is None:
            return
        now_wall = time.perf_counter()
        now_sim = float(self.base_env.data.time)
        if self._last_wall_time is None or self._last_sim_time is None or now_sim < self._last_sim_time:
            self._last_wall_time = now_wall
            self._last_sim_time = now_sim
            return

        target_wall_delta = (now_sim - self._last_sim_time) / self.speed
        elapsed_wall = now_wall - self._last_wall_time
        sleep_s = target_wall_delta - elapsed_wall
        if sleep_s > 0:
            time.sleep(sleep_s)
            now_wall = time.perf_counter()

        self._last_wall_time = now_wall
        self._last_sim_time = now_sim

    def _on_step(self) -> bool:
        if self.viewer is None:
            return True
        if not self.viewer.is_running():
            return not self.stop_if_closed
        if self.n_calls % self.sync_every == 0:
            self._maybe_sleep_to_sim_time()
            self.viewer.sync()
        return True

    def _on_training_end(self) -> None:
        if self.viewer is not None:
            self.viewer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train SAC on SO-101 MuJoCo pick-and-lift.")
    parser.add_argument("--total-timesteps", type=int, default=300_000)
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
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "pick_lift_sac")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--buffer-size", type=int, default=300_000)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-starts", type=int, default=5_000)
    parser.add_argument("--ent-coef", default="auto_0.01", help="SAC entropy coefficient, e.g. auto_0.01.")
    parser.add_argument(
        "--demo-prefill-episodes",
        type=int,
        default=50,
        help="Number of scripted successful episodes to add to SAC replay buffer before online learning.",
    )
    parser.add_argument("--demo-noise", type=float, default=0.03, help="Gaussian action noise for scripted demos.")
    parser.add_argument("--demo-seed", type=int, default=100_000)
    parser.add_argument("--bc-pretrain-steps", type=int, default=2_000)
    parser.add_argument("--bc-batch-size", type=int, default=256)
    parser.add_argument("--train-freq", type=int, default=1)
    parser.add_argument("--gradient-steps", type=int, default=2)
    parser.add_argument("--gamma", type=float, default=0.97)
    parser.add_argument("--tau", type=float, default=0.02)
    parser.add_argument("--eval-freq", type=int, default=10_000)
    parser.add_argument("--save-freq", type=int, default=50_000)
    parser.add_argument("--viewer", action="store_true", help="Show MuJoCo motion during training.")
    parser.add_argument("--viewer-sync-every", type=int, default=1)
    parser.add_argument("--viewer-real-time", action="store_true")
    parser.add_argument("--viewer-speed", type=float, default=1.0)
    parser.add_argument("--stop-if-viewer-closed", action="store_true")
    parser.add_argument("--record-video", action="store_true", help="Record one post-training evaluation rollout to mp4.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate for --record-video outputs.")
    return parser.parse_args()


def make_env(args: argparse.Namespace, seed: int):
    def _init():
        env = SO101PickLiftEnv(
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
        env = Monitor(env, info_keywords=("success", "cube_height", "is_grasped"))
        env.reset(seed=seed)
        return env

    return _init


def make_raw_env(args: argparse.Namespace) -> SO101PickLiftEnv:
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


def behavior_clone_actor(model: SAC, demo_obs: np.ndarray, demo_actions: np.ndarray, args: argparse.Namespace) -> None:
    if args.bc_pretrain_steps <= 0 or len(demo_obs) == 0:
        return

    actor = model.actor
    actor.set_training_mode(True)
    obs_tensor = th.as_tensor(demo_obs, device=model.device, dtype=th.float32)
    action_tensor = th.as_tensor(demo_actions, device=model.device, dtype=th.float32)
    batch_size = min(args.bc_batch_size, len(demo_obs))
    last_loss = 0.0

    for _ in range(args.bc_pretrain_steps):
        indices = th.randint(0, len(demo_obs), (batch_size,), device=model.device)
        pred_action = actor(obs_tensor[indices], deterministic=True)
        loss = F.mse_loss(pred_action, action_tensor[indices])
        actor.optimizer.zero_grad()
        loss.backward()
        actor.optimizer.step()
        last_loss = float(loss.detach().cpu())

    print(f"BC actor warm start complete: steps={args.bc_pretrain_steps}, final_mse={last_loss:.6f}")


def prefill_replay_buffer(model: SAC, args: argparse.Namespace) -> tuple[int, int]:
    """Seed SAC with scripted successful transitions, mirroring offline demo replay."""
    if args.demo_prefill_episodes <= 0:
        return 0, 0
    if args.control_mode != "ee_delta":
        print("Skipping demo prefill: scripted demos are currently only defined for ee_delta control.")
        return 0, 0

    env = make_raw_env(args)
    transitions = 0
    successes = 0
    demo_obs: list[np.ndarray] = []
    demo_actions: list[np.ndarray] = []
    try:
        for ep in range(args.demo_prefill_episodes):
            obs, _info = env.reset(seed=args.demo_seed + ep)
            done = False
            step = 0
            final_info = {}
            while not done:
                action = env.scripted_action(step=step, noise_std=args.demo_noise)
                next_obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                demo_obs.append(obs.copy())
                demo_actions.append(action.copy())
                model.replay_buffer.add(
                    obs,
                    next_obs,
                    action,
                    np.array([reward], dtype=np.float32),
                    np.array([done], dtype=np.float32),
                    [info],
                )
                obs = next_obs
                final_info = info
                step += 1
                transitions += 1
            successes += int(bool(final_info.get("success", False)))
    finally:
        env.close()

    if demo_obs:
        behavior_clone_actor(model, np.asarray(demo_obs), np.asarray(demo_actions), args)

    print(
        "Demo prefill complete: "
        f"episodes={args.demo_prefill_episodes}, transitions={transitions}, successes={successes}"
    )
    return transitions, successes


def main() -> None:
    args = parse_args()
    run_name = args.run_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = args.output_dir / run_name
    model_dir = run_dir / "models"
    log_dir = run_dir / "tb"
    model_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    train_env = DummyVecEnv([make_env(args, args.seed)])
    eval_env = DummyVecEnv([make_env(args, args.seed + 10_000)])

    model = SAC(
        policy="MlpPolicy",
        env=train_env,
        learning_rate=args.learning_rate,
        buffer_size=args.buffer_size,
        learning_starts=0 if args.demo_prefill_episodes > 0 else args.learning_starts,
        batch_size=args.batch_size,
        tau=args.tau,
        gamma=args.gamma,
        train_freq=args.train_freq,
        gradient_steps=args.gradient_steps,
        ent_coef=args.ent_coef,
        policy_kwargs=dict(net_arch=[256, 256]),
        tensorboard_log=str(log_dir),
        seed=args.seed,
        device=args.device,
        verbose=1,
    )

    prefill_replay_buffer(model, args)

    callback_items = [
        PickLiftMetricsCallback(),
        CheckpointCallback(
            save_freq=args.save_freq,
            save_path=str(model_dir),
            name_prefix="checkpoint",
            save_replay_buffer=False,
            save_vecnormalize=False,
        ),
        EvalCallback(
            eval_env,
            best_model_save_path=str(model_dir),
            log_path=str(run_dir / "eval"),
            eval_freq=args.eval_freq,
            deterministic=True,
            render=False,
            n_eval_episodes=5,
        ),
    ]
    if args.viewer:
        callback_items.insert(
            0,
            MujocoViewerCallback(
                sync_every=args.viewer_sync_every,
                stop_if_closed=args.stop_if_viewer_closed,
                real_time=args.viewer_real_time,
                speed=args.viewer_speed,
            ),
        )
    if args.record_video:
        print("Warning: --record-video is a lightweight viewer capture mode; for a stable export, prefer evaluation scripts.")

    print(f"Run directory: {run_dir}")
    print(f"control_mode={args.control_mode}")
    print(f"virtual_grasp={args.virtual_grasp}")
    print(f"demo_prefill_episodes={args.demo_prefill_episodes}")
    print(f"cube_xy_center={tuple(args.cube_xy_center)} cube_xy_range={tuple(args.cube_xy_range)}")
    model.learn(total_timesteps=args.total_timesteps, callback=CallbackList(callback_items), progress_bar=True)

    final_path = model_dir / "final_model"
    model.save(final_path)
    print(f"Saved final model: {final_path}.zip")
    if args.record_video:
        record_env = SO101PickLiftEnv(
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
            render_mode="rgb_array",
        )
        obs, _ = record_env.reset(seed=args.seed + 20_000)
        frames = []
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _reward, terminated, truncated, _info = record_env.step(action)
            done = terminated or truncated
            frame = record_env.render()
            if frame is not None:
                frames.append(frame)
        record_env.close()
        if frames:
            video_path = run_dir / "pick_lift_demo.mp4"
            write_mp4(frames, video_path, fps=args.video_fps)
            print(f"video={video_path}")
    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()

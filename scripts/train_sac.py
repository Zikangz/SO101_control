from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import mujoco
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from so101_tracking import SO101TrackingEnv  # noqa: E402
from video_utils import write_mp4


class TrackingErrorCallback(BaseCallback):
    """Log tracking error from env info into SB3's logger."""

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        if infos and "tracking_error" in infos[0]:
            self.logger.record("tracking/error_m", infos[0]["tracking_error"])
        return True


class MujocoViewerCallback(BaseCallback):
    """Synchronize MuJoCo's passive viewer with the training environment."""

    def __init__(
        self,
        sync_every: int = 1,
        stop_if_closed: bool = False,
        real_time: bool = False,
        speed: float = 1.0,
    ):
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
    parser = argparse.ArgumentParser(description="Train SAC on SO-101 MuJoCo endpoint tracking.")
    parser.add_argument("--total-timesteps", type=int, default=200_000)
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
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "sac")
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--buffer-size", type=int, default=200_000)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-starts", type=int, default=2_000)
    parser.add_argument("--train-freq", type=int, default=1)
    parser.add_argument("--gradient-steps", type=int, default=1)
    parser.add_argument("--gamma", type=float, default=0.98)
    parser.add_argument("--tau", type=float, default=0.02)
    parser.add_argument("--eval-freq", type=int, default=10_000)
    parser.add_argument("--save-freq", type=int, default=25_000)
    parser.add_argument("--viewer", action="store_true", help="Show MuJoCo motion during training.")
    parser.add_argument("--viewer-sync-every", type=int, default=1, help="Viewer sync period in env steps.")
    parser.add_argument(
        "--viewer-real-time",
        action="store_true",
        help="Throttle training while the viewer is open so motion is visible at MuJoCo sim time.",
    )
    parser.add_argument(
        "--viewer-speed",
        type=float,
        default=1.0,
        help="Viewer playback speed in sim seconds per wall second when --viewer-real-time is set.",
    )
    parser.add_argument(
        "--stop-if-viewer-closed",
        action="store_true",
        help="Stop training when the MuJoCo viewer window is closed.",
    )
    parser.add_argument("--record-video", action="store_true", help="Record one post-training evaluation rollout to mp4.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate for --record-video outputs.")
    return parser.parse_args()


def make_env(args: argparse.Namespace, seed: int):
    def _init():
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
        )
        env = Monitor(env)
        env.reset(seed=seed)
        return env

    return _init


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

    policy_kwargs = dict(net_arch=[256, 256])
    model = SAC(
        policy="MlpPolicy",
        env=train_env,
        learning_rate=args.learning_rate,
        buffer_size=args.buffer_size,
        learning_starts=args.learning_starts,
        batch_size=args.batch_size,
        tau=args.tau,
        gamma=args.gamma,
        train_freq=args.train_freq,
        gradient_steps=args.gradient_steps,
        policy_kwargs=policy_kwargs,
        tensorboard_log=str(log_dir),
        seed=args.seed,
        device=args.device,
        verbose=1,
    )

    callback_items = [
        TrackingErrorCallback(),
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
    callbacks = CallbackList(callback_items)

    print(f"Run directory: {run_dir}")
    print(f"control_mode={args.control_mode}")
    print(f"trajectory_mode={args.trajectory_mode}")
    print(f"frame_skip={args.frame_skip}")
    model.learn(total_timesteps=args.total_timesteps, callback=callbacks, progress_bar=True)
    final_path = model_dir / "final_model"
    model.save(final_path)
    print(f"Saved final model: {final_path}.zip")
    if args.record_video:
        record_env = SO101TrackingEnv(
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
            video_path = run_dir / "tracking_demo.mp4"
            write_mp4(frames, video_path, fps=args.video_fps)
            print(f"video={video_path}")
    train_env.close()
    eval_env.close()


if __name__ == "__main__":
    main()

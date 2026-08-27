from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np


ARM_DOF = 5


class SO101TrackingEnv(gym.Env):
    """Minimal SO-101 end-effector trajectory tracking environment.

    The action controls the five arm joints as small position deltas. The gripper
    actuator is held fixed because this first-stage task is about endpoint motion.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        model_path: str | Path | None = None,
        episode_steps: int = 300,
        frame_skip: int = 5,
        action_scale: float = 0.04,
        control_mode: str = "joint_delta",
        ik_iters: int = 3,
        ik_gain: float = 0.8,
        ik_damping: float = 0.03,
        ik_max_dq: float = 0.06,
        residual_scale: float = 0.015,
        trajectory_mode: str = "lissajous",
        random_segments: int = 6,
        random_center: tuple[float, float, float] | None = None,
        random_half_range: tuple[float, float, float] | None = None,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        if control_mode not in {"joint_delta", "ik_residual"}:
            raise ValueError("control_mode must be 'joint_delta' or 'ik_residual'")
        if trajectory_mode not in {"lissajous", "random"}:
            raise ValueError("trajectory_mode must be 'lissajous' or 'random'")
        root = Path(__file__).resolve().parents[1]
        self.model_path = Path(model_path) if model_path else root / "assets" / "so101" / "so101_new_calib.xml"
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        if self.site_id < 0:
            raise ValueError("site 'gripperframe' not found in SO-101 model")

        self.episode_steps = int(episode_steps)
        self.frame_skip = int(frame_skip)
        self.action_scale = float(action_scale)
        self.control_mode = control_mode
        self.ik_iters = int(ik_iters)
        self.ik_gain = float(ik_gain)
        self.ik_damping = float(ik_damping)
        self.ik_max_dq = float(ik_max_dq)
        self.residual_scale = float(residual_scale)
        self.trajectory_mode = trajectory_mode
        self.random_segments = max(1, int(random_segments))
        self.random_center = np.array(random_center or (0.31, 0.0, 0.20), dtype=np.float64)
        self.random_half_range = np.array(random_half_range or (0.045, 0.055, 0.030), dtype=np.float64)
        self.random_waypoints: np.ndarray | None = None
        self.render_mode = render_mode
        self.step_count = 0
        self.last_action = np.zeros(ARM_DOF, dtype=np.float32)
        self.target_pos = np.zeros(3, dtype=np.float64)
        self._renderer: mujoco.Renderer | None = None

        self.ctrl_min = self.model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_max = self.model.actuator_ctrlrange[:, 1].copy()
        self.qpos_home = np.array([0.0, 0.25, -0.45, 0.45, 0.0, 0.45], dtype=np.float64)
        self.qpos_home = np.clip(self.qpos_home, self.ctrl_min, self.ctrl_max)

        obs_dim = self.model.nq + self.model.nv + 3 + 3 + 2
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(ARM_DOF,), dtype=np.float32)

    @staticmethod
    def _smoothstep(t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    def _sample_random_waypoints(self) -> None:
        start = self.data.site_xpos[self.site_id].copy()
        waypoints = self.random_center + self.np_random.uniform(
            -1.0,
            1.0,
            size=(self.random_segments + 1, 3),
        ) * self.random_half_range
        z_min = self.random_center[2] - self.random_half_range[2]
        z_max = self.random_center[2] + self.random_half_range[2]
        waypoints[:, 2] = np.clip(waypoints[:, 2], z_min, z_max)
        waypoints[0] = start
        self.random_waypoints = waypoints

    def _random_trajectory_target(self) -> np.ndarray:
        if self.random_waypoints is None:
            return self.data.site_xpos[self.site_id].copy()
        progress = min(max(self.step_count / max(1, self.episode_steps), 0.0), 1.0)
        segment_pos = progress * self.random_segments
        segment_idx = min(int(np.floor(segment_pos)), self.random_segments - 1)
        local_t = self._smoothstep(segment_pos - segment_idx)
        start = self.random_waypoints[segment_idx]
        end = self.random_waypoints[segment_idx + 1]
        return start + local_t * (end - start)

    def _trajectory_target(self, phase: float) -> np.ndarray:
        if self.trajectory_mode == "random":
            return self._random_trajectory_target()
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

    def _phase(self) -> float:
        return 2.0 * np.pi * self.step_count / max(1, self.episode_steps)

    def _dls_ik_delta(self, target_pos: np.ndarray) -> np.ndarray:
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.site_id)
        jac = jacp[:, :ARM_DOF]
        err = target_pos - self.data.site_xpos[self.site_id]
        lhs = jac @ jac.T + (self.ik_damping**2) * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, err)
        return np.clip(self.ik_gain * dq, -self.ik_max_dq, self.ik_max_dq)

    def _apply_ik_residual_control(self, action: np.ndarray) -> None:
        target = self.target_pos.copy()
        for _ in range(self.ik_iters):
            dq_ik = self._dls_ik_delta(target)
            residual = self.residual_scale * action
            ctrl = self.data.ctrl.copy()
            ctrl[:ARM_DOF] = np.clip(
                self.data.qpos[:ARM_DOF] + dq_ik + residual,
                self.ctrl_min[:ARM_DOF],
                self.ctrl_max[:ARM_DOF],
            )
            ctrl[ARM_DOF] = self.qpos_home[ARM_DOF]
            self.data.ctrl[:] = ctrl
            for _ in range(self.frame_skip):
                mujoco.mj_step(self.model, self.data)

    def _apply_joint_delta_control(self, action: np.ndarray) -> None:
        ctrl = self.data.ctrl.copy()
        ctrl[:ARM_DOF] = np.clip(
            self.data.qpos[:ARM_DOF] + self.action_scale * action,
            self.ctrl_min[:ARM_DOF],
            self.ctrl_max[:ARM_DOF],
        )
        ctrl[ARM_DOF] = self.qpos_home[ARM_DOF]
        self.data.ctrl[:] = ctrl

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

    def _get_obs(self) -> np.ndarray:
        phase = self._phase()
        ee_pos = self.data.site_xpos[self.site_id].copy()
        obs = np.concatenate(
            [
                self.data.qpos.copy(),
                self.data.qvel.copy(),
                ee_pos,
                self.target_pos,
                np.array([np.sin(phase), np.cos(phase)], dtype=np.float64),
            ]
        )
        return obs.astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self.step_count = 0
        self.last_action.fill(0.0)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.qpos_home
        self.data.ctrl[:] = self.qpos_home
        mujoco.mj_forward(self.model, self.data)
        if self.trajectory_mode == "random":
            self._sample_random_waypoints()
        self.target_pos = self._trajectory_target(self._phase())
        return self._get_obs(), self._info()

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        target_for_reward = self.target_pos.copy()
        if self.control_mode == "ik_residual":
            self._apply_ik_residual_control(action)
        else:
            self._apply_joint_delta_control(action)

        ee_pos = self.data.site_xpos[self.site_id].copy()
        dist = float(np.linalg.norm(ee_pos - target_for_reward))
        action_penalty = float(np.square(action).mean())
        smooth_penalty = float(np.square(action - self.last_action).mean())
        velocity_penalty = float(np.square(self.data.qvel[:ARM_DOF]).mean())
        self.last_action = action.copy()

        reward = -10.0 * dist - 0.02 * action_penalty - 0.01 * smooth_penalty - 0.001 * velocity_penalty
        if dist < 0.025:
            reward += 1.0

        terminated = False
        self.step_count += 1
        truncated = self.step_count >= self.episode_steps
        self.target_pos = self._trajectory_target(self._phase())
        info = self._info(dist, target_pos=target_for_reward)
        return self._get_obs(), float(reward), terminated, truncated, info

    def _info(self, dist: float | None = None, target_pos: np.ndarray | None = None) -> dict:
        ee_pos = self.data.site_xpos[self.site_id].copy()
        target = self.target_pos if target_pos is None else target_pos
        if dist is None:
            dist = float(np.linalg.norm(ee_pos - target))
        return {
            "ee_pos": ee_pos.astype(np.float32),
            "target_pos": target.astype(np.float32),
            "tracking_error": float(dist),
            "step": self.step_count,
        }

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        self._renderer.update_scene(self.data)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

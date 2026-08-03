from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np


ARM_DOF = 5
ROBOT_DOF = 6


class SO101PickLiftEnv(gym.Env):
    """State-based SO-101 MuJoCo pick-and-lift task.

    The default action is Cartesian end-effector delta plus gripper command:
    ``[dx, dy, dz, gripper]``.  The Cartesian command is converted to joint
    position targets with damped-least-squares IK, then MuJoCo's position
    actuators simulate the arm dynamics.

    ``virtual_grasp`` intentionally defaults to True.  The SO-101 mesh model is
    useful for motion learning, but contact-only grasping with the one-moving-jaw
    CAD mesh is brittle.  The virtual grasp keeps the first task focused on the
    RL grasp sequence: reach, close, lift.  Turn it off for contact-only tests.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        model_path: str | Path | None = None,
        episode_steps: int = 200,
        frame_skip: int = 5,
        control_mode: str = "ee_delta",
        ee_action_scale: float = 0.025,
        joint_action_scale: float = 0.04,
        ik_gain: float = 0.8,
        ik_damping: float = 0.03,
        ik_max_dq: float = 0.06,
        cube_xy_center: tuple[float, float] = (0.34, 0.0),
        cube_xy_range: tuple[float, float] = (0.045, 0.045),
        lift_height: float = 0.12,
        virtual_grasp: bool = True,
        grasp_threshold: float = 0.045,
        render_mode: str | None = None,
    ) -> None:
        super().__init__()
        if control_mode not in {"ee_delta", "joint_delta"}:
            raise ValueError("control_mode must be 'ee_delta' or 'joint_delta'")

        root = Path(__file__).resolve().parents[1]
        self.model_path = Path(model_path) if model_path else root / "assets" / "so101" / "scene_pick_lift.xml"
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        self.ee_site_id = self._required_id(mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.cube_site_id = self._required_id(mujoco.mjtObj.mjOBJ_SITE, "cube_site")
        self.goal_site_id = self._required_id(mujoco.mjtObj.mjOBJ_SITE, "lift_goal")
        self.cube_joint_id = self._required_id(mujoco.mjtObj.mjOBJ_JOINT, "cube_free")
        self.cube_qpos_adr = int(self.model.jnt_qposadr[self.cube_joint_id])
        self.cube_qvel_adr = int(self.model.jnt_dofadr[self.cube_joint_id])

        self.episode_steps = int(episode_steps)
        self.frame_skip = int(frame_skip)
        self.control_mode = control_mode
        self.ee_action_scale = float(ee_action_scale)
        self.joint_action_scale = float(joint_action_scale)
        self.ik_gain = float(ik_gain)
        self.ik_damping = float(ik_damping)
        self.ik_max_dq = float(ik_max_dq)
        self.cube_xy_center = np.array(cube_xy_center, dtype=np.float64)
        self.cube_xy_range = np.array(cube_xy_range, dtype=np.float64)
        self.lift_height = float(lift_height)
        self.virtual_grasp = bool(virtual_grasp)
        self.grasp_threshold = float(grasp_threshold)
        self.render_mode = render_mode

        self.ctrl_min = self.model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_max = self.model.actuator_ctrlrange[:, 1].copy()
        self.gripper_closed_ctrl = -0.05
        self.gripper_open_ctrl = 1.20
        self.gripper_release_ctrl = 0.45
        self.cube_half_size = 0.02
        self.ee_workspace_low = np.array([0.24, -0.10, 0.045], dtype=np.float64)
        self.ee_workspace_high = np.array([0.45, 0.10, 0.24], dtype=np.float64)
        self.qpos_home = np.array([0.0, 0.25, -0.45, 0.45, 0.0, self.gripper_open_ctrl], dtype=np.float64)
        self.qpos_home = np.clip(self.qpos_home, self.ctrl_min[:ROBOT_DOF], self.ctrl_max[:ROBOT_DOF])

        self.step_count = 0
        self.success_steps = 0
        self.is_grasped = False
        self.grasp_offset = np.array([0.0, 0.0, -0.030], dtype=np.float64)
        self.cube_start_pos = np.array([0.34, 0.0, self.cube_half_size], dtype=np.float64)
        self.goal_pos = self.cube_start_pos + np.array([0.0, 0.0, self.lift_height], dtype=np.float64)
        self.last_action = np.zeros(self._action_dim(), dtype=np.float32)
        self._renderer: mujoco.Renderer | None = None

        obs_dim = ROBOT_DOF + ROBOT_DOF + 3 + 3 + 3 + 3 + 3 + 3 + 1 + 1 + 2
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(self._action_dim(),), dtype=np.float32)

    def _action_dim(self) -> int:
        return 4 if self.control_mode == "ee_delta" else ROBOT_DOF

    def scripted_action(self, step: int | None = None, noise_std: float = 0.0) -> np.ndarray:
        """Return a scripted successful pick-and-lift action for demos/prefill.

        This is not meant to be the final controller.  It provides successful
        transitions so off-policy RL does not have to discover reach-close-lift
        from random exploration alone.
        """
        if self.control_mode != "ee_delta":
            raise ValueError("scripted_action is only defined for control_mode='ee_delta'")

        step = self.step_count if step is None else int(step)
        progress = step / max(1, self.episode_steps)
        ee_pos = self._ee_pos()
        cube_pos = self._cube_pos()

        if progress < 0.32:
            target = cube_pos + np.array([0.0, 0.0, 0.050], dtype=np.float64)
            gripper = 1.0
        elif progress < 0.55:
            target = cube_pos + np.array([0.0, 0.0, 0.012], dtype=np.float64)
            gripper = 1.0
        elif progress < 0.68:
            target = cube_pos + np.array([0.0, 0.0, 0.012], dtype=np.float64)
            gripper = -1.0
        else:
            # The end effector sits above the cube center when grasped, so command
            # slightly above the cube goal height.
            target = self.goal_pos + np.array([0.0, 0.0, 0.040], dtype=np.float64)
            gripper = -1.0

        delta = np.clip((target - ee_pos) / self.ee_action_scale, -1.0, 1.0)
        action = np.array([delta[0], delta[1], delta[2], gripper], dtype=np.float32)
        if noise_std > 0.0:
            action += self.np_random.normal(0.0, noise_std, size=action.shape).astype(np.float32)
        return np.clip(action, -1.0, 1.0)

    def _required_id(self, obj_type: mujoco.mjtObj, name: str) -> int:
        obj_id = mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError(f"MuJoCo object not found: {name}")
        return int(obj_id)

    def _ee_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.ee_site_id].copy()

    def _cube_pos(self) -> np.ndarray:
        return self.data.site_xpos[self.cube_site_id].copy()

    def _cube_linvel(self) -> np.ndarray:
        return self.data.qvel[self.cube_qvel_adr : self.cube_qvel_adr + 3].copy()

    def _phase(self) -> float:
        return 2.0 * np.pi * self.step_count / max(1, self.episode_steps)

    def _map_gripper_action(self, action_value: float) -> float:
        action_value = float(np.clip(action_value, -1.0, 1.0))
        alpha = 0.5 * (action_value + 1.0)
        return (1.0 - alpha) * self.gripper_closed_ctrl + alpha * self.gripper_open_ctrl

    def _dls_ik_delta(self, target_pos: np.ndarray) -> np.ndarray:
        jacp = np.zeros((3, self.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.ee_site_id)
        jac = jacp[:, :ARM_DOF]
        err = target_pos - self._ee_pos()
        lhs = jac @ jac.T + (self.ik_damping**2) * np.eye(3)
        dq = jac.T @ np.linalg.solve(lhs, err)
        return np.clip(self.ik_gain * dq, -self.ik_max_dq, self.ik_max_dq)

    def _apply_control(self, action: np.ndarray) -> None:
        ctrl = self.data.ctrl.copy()
        if self.control_mode == "ee_delta":
            target_pos = self._ee_pos() + self.ee_action_scale * action[:3]
            target_pos = np.clip(target_pos, self.ee_workspace_low, self.ee_workspace_high)
            dq = self._dls_ik_delta(target_pos)
            ctrl[:ARM_DOF] = np.clip(
                self.data.qpos[:ARM_DOF] + dq,
                self.ctrl_min[:ARM_DOF],
                self.ctrl_max[:ARM_DOF],
            )
            ctrl[5] = self._map_gripper_action(float(action[3]))
        else:
            ctrl[:ARM_DOF] = np.clip(
                self.data.qpos[:ARM_DOF] + self.joint_action_scale * action[:ARM_DOF],
                self.ctrl_min[:ARM_DOF],
                self.ctrl_max[:ARM_DOF],
            )
            ctrl[5] = self._map_gripper_action(float(action[5]))

        self.data.ctrl[:] = ctrl
        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)
            self._update_virtual_grasp()

    def _set_cube_pose(self, pos: np.ndarray) -> None:
        self.data.qpos[self.cube_qpos_adr : self.cube_qpos_adr + 3] = pos
        self.data.qpos[self.cube_qpos_adr + 3 : self.cube_qpos_adr + 7] = np.array(
            [1.0, 0.0, 0.0, 0.0], dtype=np.float64
        )
        self.data.qvel[self.cube_qvel_adr : self.cube_qvel_adr + 6] = 0.0

    def _update_virtual_grasp(self) -> None:
        if not self.virtual_grasp:
            return

        ee_pos = self._ee_pos()
        cube_pos = self._cube_pos()
        gripper_ctrl = float(self.data.ctrl[5])
        is_closed = gripper_ctrl <= self.gripper_release_ctrl

        if self.is_grasped and not is_closed:
            self.is_grasped = False
            return

        if not self.is_grasped:
            reach_dist = float(np.linalg.norm(ee_pos - cube_pos))
            if is_closed and reach_dist <= self.grasp_threshold:
                self.is_grasped = True
                self.grasp_offset = cube_pos - ee_pos
                if np.linalg.norm(self.grasp_offset) > self.grasp_threshold:
                    self.grasp_offset = np.array([0.0, 0.0, -0.030], dtype=np.float64)

        if self.is_grasped:
            new_cube_pos = ee_pos + self.grasp_offset
            new_cube_pos[2] = max(new_cube_pos[2], self.cube_half_size + 0.001)
            self._set_cube_pose(new_cube_pos)
            mujoco.mj_forward(self.model, self.data)

    def _sample_cube_pos(self) -> np.ndarray:
        xy = self.cube_xy_center + self.np_random.uniform(-1.0, 1.0, size=2) * self.cube_xy_range
        return np.array([xy[0], xy[1], self.cube_half_size + 0.002], dtype=np.float64)

    def _set_goal_pos(self, cube_pos: np.ndarray) -> None:
        self.cube_start_pos = cube_pos.copy()
        self.goal_pos = cube_pos + np.array([0.0, 0.0, self.lift_height], dtype=np.float64)
        self.model.site_pos[self.goal_site_id] = self.goal_pos

    def _get_obs(self) -> np.ndarray:
        ee_pos = self._ee_pos()
        cube_pos = self._cube_pos()
        phase = self._phase()
        obs = np.concatenate(
            [
                self.data.qpos[:ROBOT_DOF].copy(),
                self.data.qvel[:ROBOT_DOF].copy(),
                ee_pos,
                cube_pos,
                self._cube_linvel(),
                self.goal_pos,
                cube_pos - ee_pos,
                self.goal_pos - cube_pos,
                np.array([self.data.qpos[5]], dtype=np.float64),
                np.array([float(self.is_grasped)], dtype=np.float64),
                np.array([np.sin(phase), np.cos(phase)], dtype=np.float64),
            ]
        )
        return obs.astype(np.float32)

    def _compute_reward(self, action: np.ndarray) -> tuple[float, dict]:
        ee_pos = self._ee_pos()
        cube_pos = self._cube_pos()
        hover_pos = cube_pos + np.array([0.0, 0.0, 0.050], dtype=np.float64)
        grasp_pos = cube_pos + np.array([0.0, 0.0, 0.012], dtype=np.float64)
        reach_dist = float(np.linalg.norm(ee_pos - cube_pos))
        hover_dist = float(np.linalg.norm(ee_pos - hover_pos))
        grasp_dist = float(np.linalg.norm(ee_pos - grasp_pos))
        goal_dist = float(np.linalg.norm(self.goal_pos - cube_pos))
        lift_progress = float(np.clip((cube_pos[2] - self.cube_start_pos[2]) / self.lift_height, 0.0, 1.5))
        action_penalty = float(np.square(action).mean())
        smooth_penalty = float(np.square(action - self.last_action).mean())
        is_closed_cmd = float(self.data.ctrl[5] <= self.gripper_release_ctrl)
        success = bool(cube_pos[2] >= self.goal_pos[2] - 0.015)

        close_near_bonus = 0.5 if is_closed_cmd and reach_dist <= self.grasp_threshold else 0.0
        close_far_penalty = 0.25 if is_closed_cmd and reach_dist > 0.09 else 0.0
        hover_bonus = 0.25 if hover_dist < 0.035 else 0.0

        reward = (
            -2.0 * reach_dist
            - 3.0 * goal_dist
            - 1.0 * min(hover_dist, grasp_dist)
            + 3.0 * lift_progress
            + (0.75 if self.is_grasped else 0.0)
            + close_near_bonus
            + hover_bonus
            + (8.0 if success else 0.0)
            - 0.02 * action_penalty
            - 0.01 * smooth_penalty
            - close_far_penalty
        )
        metrics = {
            "reach_dist": reach_dist,
            "hover_dist": hover_dist,
            "grasp_dist": grasp_dist,
            "goal_dist": goal_dist,
            "cube_height": float(cube_pos[2] - self.cube_half_size),
            "lift_progress": lift_progress,
            "success": float(success),
            "is_grasped": float(self.is_grasped),
            "is_closed_cmd": is_closed_cmd,
        }
        return float(reward), metrics

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        self.step_count = 0
        self.success_steps = 0
        self.is_grasped = False
        self.grasp_offset = np.array([0.0, 0.0, -0.030], dtype=np.float64)
        self.last_action = np.zeros(self._action_dim(), dtype=np.float32)

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:ROBOT_DOF] = self.qpos_home
        self.data.ctrl[:] = self.qpos_home
        cube_pos = self._sample_cube_pos()
        self._set_cube_pose(cube_pos)
        self._set_goal_pos(cube_pos)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)
        self._set_goal_pos(self._cube_pos())
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), self._info()

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -1.0, 1.0)

        self._apply_control(action)
        reward, metrics = self._compute_reward(action)
        self.last_action = action.copy()

        self.step_count += 1
        if metrics["success"] > 0.5:
            self.success_steps += 1
        else:
            self.success_steps = 0

        terminated = self.success_steps >= 10
        truncated = self.step_count >= self.episode_steps
        info = self._info(metrics=metrics)
        return self._get_obs(), reward, terminated, truncated, info

    def _info(self, metrics: dict | None = None) -> dict:
        ee_pos = self._ee_pos()
        cube_pos = self._cube_pos()
        info = {
            "ee_pos": ee_pos.astype(np.float32),
            "cube_pos": cube_pos.astype(np.float32),
            "goal_pos": self.goal_pos.astype(np.float32),
            "cube_height": float(cube_pos[2] - self.cube_half_size),
            "is_grasped": bool(self.is_grasped),
            "success": False,
            "step": self.step_count,
        }
        if metrics:
            info.update(metrics)
            info["success"] = bool(metrics["success"] > 0.5)
            info["is_grasped"] = bool(metrics["is_grasped"] > 0.5)
        return info

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

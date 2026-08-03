from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from so101_tracking import SO101PickLiftEnv  # noqa: E402


def main() -> None:
    env = SO101PickLiftEnv(episode_steps=180, control_mode="ee_delta", virtual_grasp=True)
    obs, info = env.reset(seed=0)
    rewards = []
    successes = []

    print(f"obs_shape={obs.shape} action_space={env.action_space}")
    print(f"initial_cube_pos={info['cube_pos'].round(4).tolist()}")
    print(f"goal_pos={info['goal_pos'].round(4).tolist()}")

    for step in range(180):
        action = env.scripted_action(step)
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(reward)
        successes.append(float(info["success"]))
        if terminated or truncated:
            break

    print(f"steps={len(rewards)}")
    print(f"mean_reward={float(np.mean(rewards)):.3f}")
    print(f"final_cube_pos={info['cube_pos'].round(4).tolist()}")
    print(f"final_cube_height={float(info['cube_height']):.4f} m")
    print(f"final_reach_dist={float(info.get('reach_dist', np.nan)):.4f} m")
    print(f"is_grasped={info['is_grasped']} success={info['success']}")
    print(f"any_success={bool(np.max(successes) > 0.5)}")
    env.close()


if __name__ == "__main__":
    main()

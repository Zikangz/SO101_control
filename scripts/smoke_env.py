from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from so101_tracking import SO101TrackingEnv  # noqa: E402


def main() -> None:
    env = SO101TrackingEnv(episode_steps=120)
    obs, info = env.reset(seed=0)
    errors = []
    rewards = []
    print(f"obs_shape={obs.shape} action_space={env.action_space}")

    for _ in range(120):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        errors.append(info["tracking_error"])
        rewards.append(reward)
        if terminated or truncated:
            break

    print(f"steps={len(errors)}")
    print(f"mean_tracking_error={float(np.mean(errors)):.4f} m")
    print(f"final_tracking_error={float(errors[-1]):.4f} m")
    print(f"mean_reward={float(np.mean(rewards)):.4f}")
    env.close()


if __name__ == "__main__":
    main()

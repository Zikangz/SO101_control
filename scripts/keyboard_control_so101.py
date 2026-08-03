from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "assets" / "so101" / "scene.xml"
JOINT_HOME = np.array([0.0, 0.25, -0.45, 0.45, 0.0, 0.45], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard-control the SO-101 MuJoCo position actuators.")
    parser.add_argument("--step", type=float, default=0.05, help="Control increment in radians.")
    parser.add_argument("--duration", type=float, default=None, help="Optional max runtime in seconds.")
    return parser.parse_args()


def actuator_names(model: mujoco.MjModel) -> list[str]:
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]


def print_controls(names: list[str]) -> None:
    print("\nSO-101 keyboard controls")
    print("------------------------")
    print("1-6     select actuator")
    print("q / a   increase / decrease selected actuator")
    print("r       reset to home pose")
    print("p       pause / resume physics")
    print("Esc     request exit")
    print("\nActuators:")
    for i, name in enumerate(names, start=1):
        print(f"  {i}: {name}")
    print()


def main() -> None:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    names = actuator_names(model)
    ctrl_min = model.actuator_ctrlrange[:, 0].copy()
    ctrl_max = model.actuator_ctrlrange[:, 1].copy()
    home = np.clip(JOINT_HOME, ctrl_min, ctrl_max)

    selected = 0
    paused = False
    exit_requested = False

    def reset_pose() -> None:
        mujoco.mj_resetData(model, data)
        data.qpos[:] = home
        data.ctrl[:] = home
        mujoco.mj_forward(model, data)

    def show_state() -> None:
        print(
            f"selected={selected + 1}:{names[selected]} "
            f"ctrl={data.ctrl[selected]:+.3f} rad qpos={data.qpos[selected]:+.3f} rad"
        )

    def key_callback(key: int) -> None:
        nonlocal selected, paused, exit_requested
        if key == 256:  # GLFW_KEY_ESCAPE
            exit_requested = True
            print("Exit requested.")
            return
        if key < 0 or key > 255:
            return

        ch = chr(key).lower()
        if ch in "123456":
            idx = int(ch) - 1
            if idx < model.nu:
                selected = idx
                show_state()
        elif ch == "q":
            data.ctrl[selected] = np.clip(data.ctrl[selected] + args.step, ctrl_min[selected], ctrl_max[selected])
            show_state()
        elif ch == "a":
            data.ctrl[selected] = np.clip(data.ctrl[selected] - args.step, ctrl_min[selected], ctrl_max[selected])
            show_state()
        elif ch == "r":
            reset_pose()
            print("Reset to home pose.")
            show_state()
        elif ch == "p":
            paused = not paused
            print(f"paused={paused}")

    reset_pose()
    print_controls(names)
    show_state()

    start = time.time()
    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
        show_left_ui=True,
        show_right_ui=True,
    ) as viewer:
        while viewer.is_running() and not exit_requested:
            if args.duration is not None and time.time() - start >= args.duration:
                break

            step_start = time.time()
            if not paused:
                mujoco.mj_step(model, data)
            viewer.sync()
            sleep_s = model.opt.timestep - (time.time() - step_start)
            if sleep_s > 0:
                time.sleep(sleep_s)


if __name__ == "__main__":
    main()

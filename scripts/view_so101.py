from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = ROOT / "assets" / "so101" / "scene.xml"
ROBOT_PATH = ROOT / "assets" / "so101" / "so101_new_calib.xml"
PICK_LIFT_PATH = ROOT / "assets" / "so101" / "scene_pick_lift.xml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open the SO-101 MuJoCo model in the official viewer.")
    parser.add_argument(
        "--model",
        choices=["scene", "robot", "pick_lift"],
        default="scene",
        help="scene includes floor/light; robot is raw SO-101; pick_lift adds cube and lift target.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional seconds to keep the viewer open. Omit for interactive blocking mode.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model_paths = {
        "scene": SCENE_PATH,
        "robot": ROBOT_PATH,
        "pick_lift": PICK_LIFT_PATH,
    }
    model_path = model_paths[args.model]
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    print(f"Opening: {model_path}")
    print("Tip: in the right UI, open the Control/Actuator section to move position actuators.")

    if args.duration is None:
        mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
        return

    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        end_time = time.time() + max(0.0, args.duration)
        while viewer.is_running() and time.time() < end_time:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()

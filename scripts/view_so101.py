from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco

from video_utils import write_mp4


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
    parser.add_argument("--record-video", action="store_true", help="Record the scene to mp4 for the given duration.")
    parser.add_argument("--video-fps", type=int, default=30, help="Frame rate for --record-video outputs.")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "view_so101")
    return parser.parse_args()


def record_scene(model: mujoco.MjModel, data: mujoco.MjData, args: argparse.Namespace) -> Path:
    duration = 5.0 if args.duration is None else max(0.0, float(args.duration))
    fps = max(1, int(args.video_fps))
    frames = []
    renderer = mujoco.Renderer(model, height=480, width=640)
    frame_dt = 1.0 / fps
    next_frame_t = 0.0
    try:
        while float(data.time) < duration:
            mujoco.mj_step(model, data)
            if float(data.time) + 1e-12 >= next_frame_t:
                renderer.update_scene(data)
                frames.append(renderer.render())
                next_frame_t += frame_dt
    finally:
        renderer.close()

    output_path = args.output_dir / f"{args.model}_view.mp4"
    return write_mp4(frames, output_path, fps=fps)


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

    if args.record_video:
        video_path = record_scene(model, data, args)
        print(f"video={video_path}")
        return

    if args.duration is None:
        from mujoco import viewer as mujoco_viewer

        mujoco_viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
        return

    from mujoco import viewer as mujoco_viewer

    with mujoco_viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        end_time = time.time() + max(0.0, args.duration)
        while viewer.is_running() and time.time() < end_time:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()

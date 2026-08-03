from __future__ import annotations

import argparse
import csv
import itertools
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml


DEFAULT_CONFIG = ROOT / "ros1_ws" / "src" / "so101_ros1_bridge" / "config" / "so101_planar_3dof_gripper.yaml"
DEFAULT_MODEL = ROOT / "assets" / "so101" / "scene.xml"
ARM_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
JOINT_ORDER = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan planar SO101 XZ reachable workspace.")
    parser.add_argument("--model-path", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--limit-profile", choices=["config", "mujoco"], default="config")
    parser.add_argument("--samples-per-joint", type=int, default=35)
    parser.add_argument("--ee-frame", choices=["site_gripperframe", "body_gripper"], default="site_gripperframe")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "planar_workspace")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with args.config.open() as handle:
        config = yaml.safe_load(handle)
    model = mujoco.MjModel.from_xml_path(str(args.model_path))
    data = mujoco.MjData(model)
    qaddr = {}
    joint_range = {}
    for name in JOINT_ORDER:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qaddr[name] = int(model.jnt_qposadr[jid])
        joint_range[name] = tuple(float(v) for v in model.jnt_range[jid])

    if args.ee_frame == "body_gripper":
        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
        read_ee = lambda: data.xpos[ee_id].copy()
    else:
        ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        read_ee = lambda: data.site_xpos[ee_id].copy()

    if args.limit_profile == "mujoco":
        limits = {name: joint_range[name] for name in ARM_JOINTS}
    else:
        limits = {name: tuple(config["limits"][name]) for name in ARM_JOINTS}

    values = [
        np.linspace(limits[name][0], limits[name][1], max(2, int(args.samples_per_joint)))
        for name in ARM_JOINTS
    ]
    rows = []
    for shoulder_lift, elbow_flex, wrist_flex in itertools.product(*values):
        data.qpos[:] = 0.0
        data.qpos[qaddr["shoulder_pan"]] = 0.0
        data.qpos[qaddr["shoulder_lift"]] = shoulder_lift
        data.qpos[qaddr["elbow_flex"]] = elbow_flex
        data.qpos[qaddr["wrist_flex"]] = wrist_flex
        data.qpos[qaddr["wrist_roll"]] = 0.0
        data.qpos[qaddr["gripper"]] = 0.5
        mujoco.mj_forward(model, data)
        ee = read_ee()
        rows.append(
            {
                "shoulder_lift": float(shoulder_lift),
                "elbow_flex": float(elbow_flex),
                "wrist_flex": float(wrist_flex),
                "ee_x": float(ee[0]),
                "ee_y": float(ee[1]),
                "ee_z": float(ee[2]),
            }
        )

    csv_path = args.output_dir / ("%s_%s.csv" % (args.limit_profile, args.ee_frame))
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    xs = np.array([row["ee_x"] for row in rows], dtype=np.float64)
    zs = np.array([row["ee_z"] for row in rows], dtype=np.float64)
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(1, 1, 1)
    ax.scatter(xs, zs, s=2, alpha=0.25)
    ax.axvline(0.0, color="black", linewidth=0.8, linestyle="--")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("SO101 planar workspace: %s / %s" % (args.limit_profile, args.ee_frame))
    fig.tight_layout()
    plot_path = args.output_dir / ("%s_%s.png" % (args.limit_profile, args.ee_frame))
    fig.savefig(plot_path, dpi=160)

    print("limit_profile=%s" % args.limit_profile)
    print("ee_frame=%s" % args.ee_frame)
    print("samples=%d" % len(rows))
    print("x_min=%.6f x_max=%.6f z_min=%.6f z_max=%.6f" % (xs.min(), xs.max(), zs.min(), zs.max()))
    print("csv=%s" % csv_path)
    print("plot=%s" % plot_path)


if __name__ == "__main__":
    main()

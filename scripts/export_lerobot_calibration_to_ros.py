#!/usr/bin/env python3
"""Print LeRobot SO101 calibration as a ROS YAML block.

Typical usage after running lerobot-calibrate:

  python3 scripts/export_lerobot_calibration_to_ros.py --id aerial_so101_follower
"""

import json
import argparse
from pathlib import Path


JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def candidate_paths(robot_id):
    home = Path.home()
    roots = [
        PROJECT_ROOT / "calibration",
        home / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so_follower",
        home / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so101_follower",
        Path("/tmp") / "lerobot" / "calibration" / "robots" / "so_follower",
    ]
    for root in roots:
        yield root / ("%s.json" % robot_id)


def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)
    missing = [joint for joint in JOINT_ORDER if joint not in data]
    if missing:
        raise ValueError("Calibration file is missing joints: %s" % ", ".join(missing))
    return data


def print_yaml(calibration):
    print("hardware_calibration:")
    for joint in JOINT_ORDER:
        item = calibration[joint]
        print(
            "  %s: {range_min: %s, range_max: %s, homing_offset: %s, drive_mode: %s}"
            % (
                joint,
                int(item.get("range_min", 0)),
                int(item.get("range_max", 4095)),
                int(item.get("homing_offset", 0)),
                int(item.get("drive_mode", 0)),
            )
        )


def yaml_block(calibration):
    lines = ["hardware_calibration:"]
    for joint in JOINT_ORDER:
        item = calibration[joint]
        lines.append(
            "  %s: {range_min: %s, range_max: %s, homing_offset: %s, drive_mode: %s}"
            % (
                joint,
                int(item.get("range_min", 0)),
                int(item.get("range_max", 4095)),
                int(item.get("homing_offset", 0)),
                int(item.get("drive_mode", 0)),
            )
        )
    return "\n".join(lines) + "\n"


def write_config(config_path, calibration):
    config_path = Path(config_path).expanduser()
    new_block = yaml_block(calibration)
    if config_path.exists():
        text = config_path.read_text()
        marker = "hardware_calibration:"
        index = text.find(marker)
        if index >= 0:
            prefix = text[:index].rstrip() + "\n\n"
            config_path.write_text(prefix + new_block)
        else:
            config_path.write_text(text.rstrip() + "\n\n" + new_block)
    else:
        config_path.parent.mkdir(parents=True, exist_ok=True)
        config_path.write_text(new_block)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", default="aerial_so101_follower", help="LeRobot robot id used during calibration")
    parser.add_argument("--path", help="Explicit calibration JSON path")
    parser.add_argument("--write-config", help="Write hardware_calibration into the given ROS bridge YAML")
    args = parser.parse_args()

    if args.path:
        path = Path(args.path).expanduser()
    else:
        path = next((p for p in candidate_paths(args.id) if p.is_file()), None)
        if path is None:
            print("Could not find calibration JSON. Checked:")
            for candidate in candidate_paths(args.id):
                print("  %s" % candidate)
            return 1

    calibration = load_calibration(path)
    print("# Source: %s" % path)
    print_yaml(calibration)
    if args.write_config:
        write_config(args.write_config, calibration)
        print("# Updated: %s" % Path(args.write_config).expanduser())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

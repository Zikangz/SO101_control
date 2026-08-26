#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path


JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def env_truthy(name):
    return os.environ.get(name, "0").strip().lower() in {"1", "true", "yes", "on"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "path",
        nargs="?",
        default=str(PROJECT_ROOT / "calibration" / "aerial_so101_follower.json"),
    )
    parser.add_argument("--min-span", type=int, default=80)
    parser.add_argument(
        "--limited-wrist-roll",
        action="store_true",
        default=env_truthy("SO101_WRIST_ROLL_LIMITED"),
        help="Check wrist_roll as a mechanically limited joint instead of a continuous joint.",
    )
    args = parser.parse_args()
    continuous = set() if args.limited_wrist_roll else {"wrist_roll"}

    path = Path(args.path).expanduser()
    if not path.is_file():
        print("[ERROR] Calibration file not found: %s" % path)
        return 1

    data = json.loads(path.read_text())
    ok = True
    print("[SO101] Checking calibration:", path)
    for joint in JOINTS:
        if joint not in data:
            print("[ERROR] Missing joint:", joint)
            ok = False
            continue
        item = data[joint]
        span = int(item.get("range_max", 0)) - int(item.get("range_min", 0))
        homing = int(item.get("homing_offset", -1))
        print(
            "  %-14s id=%s homing=%4d range=[%4d,%4d] span=%4d"
            % (
                joint,
                item.get("id", "?"),
                homing,
                int(item.get("range_min", 0)),
                int(item.get("range_max", 0)),
                span,
            )
        )
        if args.limited_wrist_roll and joint == "wrist_roll" and int(item.get("range_min", 0)) == 0 and int(item.get("range_max", 0)) == 4095:
            print("    [WARN] wrist_roll is limited on this arm, but calibration still has the continuous placeholder [0,4095]. Recalibrate it through the safe usable range.")
            ok = False
        if joint not in continuous and span < args.min_span:
            print("    [WARN] Range span is too small; recalibrate this joint by sweeping its full range.")
            ok = False
        if not (0 <= homing <= 4095):
            print("    [ERROR] homing_offset out of raw servo range.")
            ok = False

    if ok:
        print("[OK] Calibration looks usable.")
        return 0
    print("[WARN] Calibration is not reliable enough for normal motion. Re-run calibration before larger moves.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

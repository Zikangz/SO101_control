#!/usr/bin/env python3
"""Set one or more Feetech servos' current physical pose as raw center 2048.

This writes the servo's factory middle calibration command only. It does not
write PID, speed, acceleration, id, homing offset, or position registers unless
--center-test is explicitly requested.
"""

import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SEEED = ROOT / "third_party" / "Seeed_RoboController"
sys.path.insert(0, str(SEEED))

try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS
except ImportError as exc:
    raise SystemExit(
        "Cannot import scservo_sdk. Run scripts/bootstrap_noetic.sh --with-third-party first."
    ) from exc


ADDR_TORQUE_ENABLE = 40
TORQUE_OFF = 0
TORQUE_ON = 1
CALIBRATE_CURRENT_AS_2048 = 128
RAW_CENTER = 2048

JOINT_BY_ID = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}


def parse_ids(text):
    ids = []
    for item in text.replace(",", " ").split():
        if "-" in item:
            start, end = item.split("-", 1)
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(item))
    return sorted(dict.fromkeys(ids))


def require_success(bus, result, error, action):
    if result != COMM_SUCCESS or error:
        detail = bus.getTxRxResult(result) if hasattr(bus, "getTxRxResult") else result
        raise RuntimeError("%s failed: result=%s error=%s" % (action, detail, error))


def read_position(bus, sid):
    pos, result, error = bus.ReadPos(sid)
    require_success(bus, result, error, "read position ID%d" % sid)
    return int(pos)


def calibrate_one(bus, sid, assume_yes=False, center_test=False):
    joint = JOINT_BY_ID.get(sid, "unknown")
    print("\nID%d %s" % (sid, joint))

    model, result, error = bus.ReadModelNumber(sid)
    require_success(bus, result, error, "read model ID%d" % sid)
    before = read_position(bus, sid)
    print("  model=%s current_raw=%d" % (model, before))

    result, error = bus.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, TORQUE_OFF)
    require_success(bus, result, error, "disable torque ID%d" % sid)
    print("  torque=off. Move this joint to its physical middle/zero pose now.")

    if not assume_yes:
        answer = input("  Type YES after the joint is physically centered: ").strip()
        if answer != "YES":
            print("  skipped")
            return False

    physical_raw = read_position(bus, sid)
    print("  physical_center_raw_before_factory_cal=%d" % physical_raw)

    result, error = bus.unLockEprom(sid)
    require_success(bus, result, error, "unlock EEPROM ID%d" % sid)
    time.sleep(0.1)

    try:
        result, error = bus.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, CALIBRATE_CURRENT_AS_2048)
        require_success(bus, result, error, "factory middle calibration ID%d" % sid)
        time.sleep(0.2)
    finally:
        result, error = bus.LockEprom(sid)
        require_success(bus, result, error, "lock EEPROM ID%d" % sid)

    after = read_position(bus, sid)
    print("  factory_middle_written. raw_after=%d" % after)

    if center_test:
        result, error = bus.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, TORQUE_ON)
        require_success(bus, result, error, "enable torque ID%d" % sid)
        result, error = bus.WritePosEx(sid, RAW_CENTER, 500, 30)
        require_success(bus, result, error, "move center ID%d" % sid)
        time.sleep(1.5)
        print("  center_test_raw=%d" % read_position(bus, sid))

    return True


def main():
    parser = argparse.ArgumentParser(description="SO101 single-servo factory middle calibration")
    parser.add_argument("port", help="Serial port, for example /dev/ttyACM0")
    parser.add_argument("--ids", default="1-6", help="Servo ids, for example 1-6 or '2'")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--yes", action="store_true", help="Do not prompt; only use when joints are already centered")
    parser.add_argument("--center-test", action="store_true", help="After calibration, move each servo to raw 2048")
    args = parser.parse_args()

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise SystemExit("Cannot open serial port: %s" % args.port)
    if not ph.setBaudRate(args.baudrate):
        ph.closePort()
        raise SystemExit("Cannot set baudrate: %s" % args.baudrate)

    bus = sms_sts(ph)
    try:
        changed = 0
        for sid in parse_ids(args.ids):
            if calibrate_one(bus, sid, assume_yes=args.yes, center_test=args.center_test):
                changed += 1
            time.sleep(0.2)
        print("\nDone. calibrated=%d" % changed)
    finally:
        ph.closePort()


if __name__ == "__main__":
    main()

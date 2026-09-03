#!/usr/bin/env python3
"""Read SO101 Feetech servo status and optionally toggle torque only.

This helper intentionally does not write PID, speed, acceleration, homing, or
position registers. It is meant for bring-up checks when the ROS bridge is
started with configure_motors_on_connect:=false.
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
ADDR_OPERATING_MODE = 33
ADDR_P = 21
ADDR_D = 22
ADDR_I = 23
DEFAULT_IDS = [1, 2, 3, 4, 5, 6]


def parse_ids(text):
    ids = []
    for item in text.replace(",", " ").split():
        if "-" in item:
            start, end = item.split("-", 1)
            ids.extend(range(int(start), int(end) + 1))
        else:
            ids.append(int(item))
    return sorted(dict.fromkeys(ids))


def read1(bus, sid, address):
    value, comm, err = bus.read1ByteTxRx(sid, address)
    if comm != COMM_SUCCESS or err:
        return None
    return int(value)


def scan(bus, max_id):
    found = []
    for sid in range(1, max_id + 1):
        _model, comm, _err = bus.ping(sid)
        if comm == COMM_SUCCESS:
            found.append(sid)
        time.sleep(0.01)
    return found


def main():
    parser = argparse.ArgumentParser(description="SO101 servo status / torque-only helper")
    parser.add_argument("port", help="Serial port, for example /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--ids", default="1-6", help="Servo ids, for example 1-6 or '1,2,3'")
    parser.add_argument("--scan", action="store_true", help="Scan id 1..20 and use discovered ids")
    parser.add_argument("--max-id", type=int, default=20)
    parser.add_argument("--torque", choices=["on", "off"], help="Set Torque_Enable only")
    args = parser.parse_args()

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise SystemExit("Cannot open serial port: %s" % args.port)
    if not ph.setBaudRate(args.baudrate):
        ph.closePort()
        raise SystemExit("Cannot set baudrate: %s" % args.baudrate)

    bus = sms_sts(ph)
    try:
        ids = scan(bus, args.max_id) if args.scan else parse_ids(args.ids)
        if not ids:
            raise SystemExit("No servos found")

        if args.torque:
            value = 1 if args.torque == "on" else 0
            for sid in ids:
                comm, err = bus.write1ByteTxRx(sid, ADDR_TORQUE_ENABLE, value)
                ok = comm == COMM_SUCCESS and not err
                print("ID%-3d torque %s: %s" % (sid, args.torque, "ok" if ok else "failed"))
                time.sleep(0.03)

        print("\nID   model  pos   torque  mode  P   I   D   voltage  temp  moving")
        print("--   -----  ----  ------  ----  --  --  --  -------  ----  ------")
        for sid in ids:
            model, comm, err = bus.ReadModelNumber(sid)
            model = model if comm == COMM_SUCCESS and not err else None
            pos, comm, err = bus.ReadPos(sid)
            pos = pos if comm == COMM_SUCCESS and not err else None
            voltage, comm, err = bus.ReadVoltage(sid)
            voltage = (float(voltage) / 10.0) if comm == COMM_SUCCESS and not err else None
            temp, comm, err = bus.ReadTemperature(sid)
            temp = temp if comm == COMM_SUCCESS and not err else None
            moving, comm, err = bus.ReadMoving(sid)
            moving = moving if comm == COMM_SUCCESS and not err else None
            torque = read1(bus, sid, ADDR_TORQUE_ENABLE)
            mode = read1(bus, sid, ADDR_OPERATING_MODE)
            p = read1(bus, sid, ADDR_P)
            d = read1(bus, sid, ADDR_D)
            i = read1(bus, sid, ADDR_I)
            print(
                "%2d   %5s  %4s  %6s  %4s  %2s  %2s  %2s  %7s  %4s  %6s"
                % (
                    sid,
                    "NA" if model is None else model,
                    "NA" if pos is None else pos,
                    "NA" if torque is None else torque,
                    "NA" if mode is None else mode,
                    "NA" if p is None else p,
                    "NA" if i is None else i,
                    "NA" if d is None else d,
                    "NA" if voltage is None else "%.1fV" % voltage,
                    "NA" if temp is None else temp,
                    "NA" if moving is None else moving,
                )
            )
            time.sleep(0.03)
    finally:
        ph.closePort()


if __name__ == "__main__":
    main()

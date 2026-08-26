#!/usr/bin/env python3
"""Randomly sample SO101 MuJoCo joint space and report contact pairs."""

import argparse
import csv
import random
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_XML = str(PROJECT_ROOT / "third_party" / "SO-ARM100-main" / "Simulation" / "SO101" / "so101_new_calib.xml")
DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def _parse_assignment(item):
    if "=" not in item:
        raise argparse.ArgumentTypeError("Expected name=value, got %r" % item)
    name, value = item.split("=", 1)
    return name, float(value)


def _parse_range(item):
    if "=" not in item or ":" not in item:
        raise argparse.ArgumentTypeError("Expected name=lo:hi, got %r" % item)
    name, raw = item.split("=", 1)
    lo, hi = raw.split(":", 1)
    return name, (float(lo), float(hi))


def _geom_name(mujoco, model, geom_id):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
    return name or ("geom_%d" % geom_id)


def _body_name(mujoco, model, geom_id):
    body_id = int(model.geom_bodyid[int(geom_id)])
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return name or ("body_%d" % body_id)


def _joint_ranges(mujoco, model):
    ranges = {}
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if not name:
            continue
        lo, hi = model.jnt_range[joint_id]
        ranges[name] = (float(lo), float(hi))
    return ranges


def _joint_qpos_addr(mujoco, model):
    addresses = {}
    for joint_id in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name:
            addresses[name] = int(model.jnt_qposadr[joint_id])
    return addresses


def run(args):
    try:
        import mujoco
    except ImportError:
        print("MuJoCo Python package is not installed. Install in your env with: python3 -m pip install mujoco", file=sys.stderr)
        return 2

    xml_path = Path(args.xml)
    if not xml_path.exists():
        print("MuJoCo XML not found: %s" % xml_path, file=sys.stderr)
        return 1

    random.seed(args.seed)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    ranges = _joint_ranges(mujoco, model)
    addresses = _joint_qpos_addr(mujoco, model)
    range_overrides = dict(_parse_range(item) for item in args.range)
    for name, value in range_overrides.items():
        if name not in ranges:
            print("Unknown joint in --range: %s" % name, file=sys.stderr)
            return 1
        lo, hi = value
        if hi < lo:
            print("Invalid --range %s=%.4f:%.4f" % (name, lo, hi), file=sys.stderr)
            return 1
        model_lo, model_hi = ranges[name]
        ranges[name] = (max(model_lo, lo), min(model_hi, hi))

    requested = args.joints or [name for name in DEFAULT_JOINTS if name in ranges]
    locked = dict(_parse_assignment(item) for item in args.locked)
    unknown = [name for name in requested if name not in ranges]
    if unknown:
        print("Unknown MuJoCo joint(s): %s" % ", ".join(unknown), file=sys.stderr)
        return 1

    rows = []
    contacts_seen = {}
    body_contacts_seen = {}
    for sample_idx in range(args.samples):
        q = {}
        for name, (lo, hi) in ranges.items():
            if name in locked:
                value = locked[name]
            elif name in requested:
                value = random.uniform(lo, hi)
            else:
                value = 0.0
            q[name] = value
            data.qpos[addresses[name]] = value

        mujoco.mj_forward(model, data)
        pairs = []
        body_pairs = []
        min_dist = ""
        for contact_idx in range(data.ncon):
            contact = data.contact[contact_idx]
            g1 = _geom_name(mujoco, model, contact.geom1)
            g2 = _geom_name(mujoco, model, contact.geom2)
            if args.ignore_floor and ("floor" in (g1, g2)):
                continue
            pair = "%s|%s" % tuple(sorted((g1, g2)))
            body_pair = "%s|%s" % tuple(sorted((_body_name(mujoco, model, contact.geom1), _body_name(mujoco, model, contact.geom2))))
            pairs.append(pair)
            body_pairs.append(body_pair)
            contacts_seen[pair] = contacts_seen.get(pair, 0) + 1
            body_contacts_seen[body_pair] = body_contacts_seen.get(body_pair, 0) + 1
            min_dist = contact.dist if min_dist == "" else min(min_dist, contact.dist)

        row = {
            "sample": sample_idx,
            "n_contacts": len(pairs),
            "min_contact_dist": min_dist,
            "contact_pairs": ";".join(sorted(set(pairs))),
            "contact_body_pairs": ";".join(sorted(set(body_pairs))),
        }
        row.update(q)
        rows.append(row)

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            fieldnames = ["sample", "n_contacts", "min_contact_dist", "contact_pairs", "contact_body_pairs"] + sorted(ranges)
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    colliding = [row for row in rows if int(row["n_contacts"]) > 0]
    print("MuJoCo SO101 contact scan")
    print("  xml:       %s" % xml_path)
    print("  samples:   %d" % len(rows))
    print("  contacts:  %d" % len(colliding))
    print("  joint ranges:")
    for name in sorted(requested):
        if name in ranges:
            print("    %-14s [%.4f, %.4f]" % (name, ranges[name][0], ranges[name][1]))
    print("  contact pairs:")
    for pair, count in sorted(contacts_seen.items(), key=lambda item: (-item[1], item[0]))[:20]:
        print("    %-60s %d" % (pair, count))
    print("  body pairs:")
    for pair, count in sorted(body_contacts_seen.items(), key=lambda item: (-item[1], item[0]))[:20]:
        print("    %-60s %d" % (pair, count))
    if args.csv:
        print("  csv:       %s" % args.csv)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Scan SO101 MuJoCo self-contact over random joint samples")
    parser.add_argument("--xml", default=DEFAULT_XML)
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--joints", nargs="+", default=[])
    parser.add_argument("--locked", nargs="+", default=["shoulder_pan=0.0", "wrist_roll=0.0", "gripper=0.5"])
    parser.add_argument("--range", nargs="+", default=[], help="Override sampled joint ranges, e.g. shoulder_lift=-1.0:1.0")
    parser.add_argument("--csv", default="/tmp/so101_mujoco_contact_scan.csv")
    parser.add_argument("--ignore-floor", action="store_true", default=True)
    parser.add_argument("--include-floor", dest="ignore_floor", action="store_false")
    return parser


def main():
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

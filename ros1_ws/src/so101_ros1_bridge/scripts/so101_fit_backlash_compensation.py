#!/usr/bin/env python3
"""Fit a conservative direction-dependent joint bias profile from sweep CSVs."""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time


def _float(row, key, default=None):
    value = row.get(key, "")
    if value in ("", None):
        return default
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return value


def _read_rows(paths):
    for path in paths:
        with open(path, "r", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                row["_source_csv"] = path
                yield row


def _clip(value, lo, hi):
    return max(float(lo), min(float(hi), float(value)))


def _bin_centers(lo, hi, bins):
    if bins <= 1 or hi <= lo:
        return [0.5 * (lo + hi)]
    step = (hi - lo) / float(bins)
    return [lo + (idx + 0.5) * step for idx in range(bins)]


def _nearest_fill(values):
    if not values:
        return values
    filled = list(values)
    known = [idx for idx, value in enumerate(filled) if value is not None]
    if not known:
        return [0.0 for _ in filled]
    for idx, value in enumerate(filled):
        if value is not None:
            continue
        nearest = min(known, key=lambda known_idx: abs(known_idx - idx))
        filled[idx] = filled[nearest]
    return [float(value) for value in filled]


def _fit_joint(samples, args):
    positions = [item["desired"] for item in samples]
    lo = min(positions)
    hi = max(positions)
    centers = _bin_centers(lo, hi, args.bins)
    if len(centers) == 1:
        width = max(1e-9, hi - lo)
    else:
        width = centers[1] - centers[0]
    raw = {1: [[] for _ in centers], -1: [[] for _ in centers]}
    for item in samples:
        idx = int((item["desired"] - lo) / max(1e-9, width))
        idx = max(0, min(len(centers) - 1, idx))
        raw[item["direction"]][idx].append(item["bias"])

    tables = {}
    counts = {}
    for direction, key in ((1, "positive"), (-1, "negative")):
        values = []
        count_values = []
        for bucket in raw[direction]:
            count_values.append(len(bucket))
            if len(bucket) < args.min_samples:
                values.append(None)
                continue
            median = statistics.median(bucket)
            values.append(_clip(args.scale * median, -args.max_abs_bias, args.max_abs_bias))
        tables[key] = _nearest_fill(values)
        counts[key] = count_values

    return {
        "breakpoints_rad": centers,
        "positive_bias_rad": tables["positive"],
        "negative_bias_rad": tables["negative"],
        "positive_sample_count": counts["positive"],
        "negative_sample_count": counts["negative"],
        "max_abs_bias_rad": float(args.max_abs_bias),
        "source_sample_count": len(samples),
        "source_position_range_rad": [float(lo), float(hi)],
    }


def run(args):
    if not args.csv:
        raise RuntimeError("at least one --csv is required")
    requested = set(args.joints)
    by_joint = {name: [] for name in args.joints}
    previous_wide = {name: None for name in args.joints}
    skipped = 0
    wide_rows = 0
    single_joint_rows = 0
    for row in _read_rows(args.csv):
        joint = row.get("joint", "")
        elapsed = _float(row, "elapsed", 0.0)
        if elapsed < args.warmup:
            skipped += 1
            continue
        if joint in requested:
            single_joint_rows += 1
            measured = _float(row, "measured")
            if measured is None:
                measured = _float(row, joint)
            desired = _float(row, "commanded") if args.prefer_commanded else None
            if desired is None:
                desired = _float(row, "target")
            if measured is None or desired is None:
                skipped += 1
                continue
            direction = int(_float(row, "direction", 0.0) or 0)
            velocity = _float(row, "target_velocity", 0.0) or 0.0
            if direction == 0:
                if velocity > args.direction_velocity_threshold:
                    direction = 1
                elif velocity < -args.direction_velocity_threshold:
                    direction = -1
            if direction not in (-1, 1) or abs(velocity) < args.direction_velocity_threshold:
                skipped += 1
                continue
            error = measured - desired
            by_joint[joint].append(
                {
                    "desired": desired,
                    "measured": measured,
                    "direction": direction,
                    "bias": -error,
                }
            )
            continue

        # Wide runtime CSVs from so101_ee_sine_test.py have one row per time
        # sample with columns like shoulder_lift and commanded_shoulder_lift.
        # Fit those directly so the table covers the actual Cartesian path
        # workspace instead of an unrelated single-joint pose.
        added_from_wide_row = False
        for wide_joint in args.joints:
            measured = _float(row, wide_joint)
            desired = _float(row, "commanded_" + wide_joint) if args.prefer_commanded else None
            if desired is None:
                desired = _float(row, wide_joint)
            if measured is None or desired is None:
                continue
            previous = previous_wide.get(wide_joint)
            previous_wide[wide_joint] = (elapsed, desired)
            if previous is None:
                continue
            prev_elapsed, prev_desired = previous
            dt = elapsed - prev_elapsed
            if dt <= 1e-6:
                continue
            velocity = (desired - prev_desired) / dt
            if abs(velocity) < args.direction_velocity_threshold:
                continue
            direction = 1 if velocity > 0.0 else -1
            error = measured - desired
            by_joint[wide_joint].append(
                {
                    "desired": desired,
                    "measured": measured,
                    "direction": direction,
                    "bias": -error,
                }
            )
            added_from_wide_row = True
        if added_from_wide_row:
            wide_rows += 1
        else:
            skipped += 1

    joints = {}
    for joint, samples in by_joint.items():
        if len(samples) < max(args.min_total_samples, 2 * args.min_samples):
            print("skip %s: only %d usable samples" % (joint, len(samples)), file=sys.stderr)
            continue
        joints[joint] = _fit_joint(samples, args)

    if not joints:
        raise RuntimeError("no usable joint data; make sure the CSV came from timed so101_sine_test.py")

    payload = {
        "version": 1,
        "model": "directional_joint_bias_v1",
        "units": "rad",
        "generated_at_unix": time.time(),
        "source_csv": [os.path.abspath(path) for path in args.csv],
        "source": args.out,
        "prefer_commanded": bool(args.prefer_commanded),
        "scale": float(args.scale),
        "max_abs_bias_rad": float(args.max_abs_bias),
        "bias_slew_rad_s": float(args.bias_slew),
        "velocity_threshold_rad_s": float(args.direction_velocity_threshold),
        "position_hysteresis_rad": float(args.position_hysteresis),
        "limit_margin_rad": float(args.limit_margin),
        "joints": joints,
        "skipped_rows": skipped,
        "source_format": "single_joint" if single_joint_rows and not wide_rows else "wide_runtime" if wide_rows else "mixed",
        "single_joint_rows": single_joint_rows,
        "wide_runtime_rows": wide_rows,
    }
    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("wrote:", args.out)
    print("skipped_rows:", skipped)
    for joint, data in joints.items():
        pos_peak = max(abs(value) for value in data["positive_bias_rad"])
        neg_peak = max(abs(value) for value in data["negative_bias_rad"])
        print(
            "%s samples=%d positive_peak=%.5frad negative_peak=%.5frad"
            % (joint, data["source_sample_count"], pos_peak, neg_peak)
        )
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description="Fit SO101 direction-dependent backlash compensation")
    parser.add_argument("--csv", action="append", default=[], help="CSV produced by timed so101_sine_test.py")
    parser.add_argument("--out", required=True)
    parser.add_argument("--joints", nargs="+", default=["shoulder_lift", "elbow_flex"])
    parser.add_argument("--bins", type=int, default=9)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--min-total-samples", type=int, default=40)
    parser.add_argument("--warmup", type=float, default=1.0)
    parser.add_argument("--scale", type=float, default=0.50, help="Conservative fraction of measured command error to apply")
    parser.add_argument("--max-abs-bias", type=float, default=0.030)
    parser.add_argument("--bias-slew", type=float, default=0.020)
    parser.add_argument("--direction-velocity-threshold", type=float, default=0.002)
    parser.add_argument("--position-hysteresis", type=float, default=0.003)
    parser.add_argument("--limit-margin", type=float, default=0.050)
    parser.add_argument("--prefer-commanded", action="store_true", default=True)
    parser.add_argument("--prefer-target", dest="prefer_commanded", action="store_false")
    return parser


def main():
    try:
        return run(build_parser().parse_args())
    except RuntimeError as exc:
        print("SO101 backlash fit error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Fit a bounded phase-dependent X/Z feedforward profile from bench CSVs.

The profile is tied to one fixed periodic desktop trajectory. It is useful for
learning repeatable geometry error without increasing servo gains, but it is
not a general gravity controller and must be revalidated after payload,
temperature, mounting, or UAV attitude changes.
"""

import argparse
import csv
import json
import math
import statistics

import numpy as np


SUPPORTED_PATTERNS = ["xz_sine", "xz_edge_vertex8", "xz_vertex_diamond", "xz_zigzag"]


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def _infer_pattern(rows):
    patterns = sorted({row.get("pattern", "") for row in rows if row.get("pattern", "")})
    if len(patterns) == 1:
        return patterns[0]
    if not patterns:
        raise RuntimeError("CSV has no pattern column; pass --pattern explicitly")
    raise RuntimeError("CSV contains multiple patterns; pass --pattern explicitly")


def _profile_value(coefficients, phase, harmonics):
    value = float(coefficients[0])
    for k in range(1, harmonics + 1):
        value += float(coefficients[2 * k - 1]) * math.sin(k * phase)
        value += float(coefficients[2 * k]) * math.cos(k * phase)
    return value


def _source_bias(row, axis):
    explicit = _f(row, axis + "_feedforward_bias")
    if explicit is not None:
        return explicit
    command = _f(row, "command_target_" + axis)
    target = _f(row, "target_" + axis)
    if command is not None and target is not None:
        return command - target
    if axis == "z":
        return _f(row, "z_feedforward_bias") or 0.0
    return 0.0


def _design_row(phase, harmonics):
    return [1.0] + [term for k in range(1, harmonics + 1) for term in (math.sin(k * phase), math.cos(k * phase))]


def _fit_axis(samples, harmonics, max_abs_bias):
    matrix = [_design_row(phase, harmonics) for phase, value in samples]
    values = [value for phase, value in samples]
    coefficients, _, _, _ = np.linalg.lstsq(np.asarray(matrix), np.asarray(values), rcond=None)
    coefficients = [float(value) for value in coefficients]

    phases = np.linspace(0.0, 2.0 * math.pi, 2001)
    raw_values = [_profile_value(coefficients, float(phase), harmonics) for phase in phases]
    peak = max(abs(value) for value in raw_values)
    if peak > max_abs_bias:
        scale = max_abs_bias / peak
        coefficients = [value * scale for value in coefficients]
        raw_values = [_profile_value(coefficients, float(phase), harmonics) for phase in phases]
    return coefficients, float(max(abs(value) for value in raw_values))


def _load_usable_rows(paths, args):
    usable = []
    patterns = []
    source_counts = {}
    for path in paths:
        with open(path, newline="") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            continue
        pattern = args.pattern or _infer_pattern(rows)
        patterns.append(pattern)
        elapsed_values = [_f(row, "elapsed") for row in rows]
        elapsed_values = [value for value in elapsed_values if value is not None]
        if not elapsed_values:
            raise RuntimeError("CSV has no numeric elapsed column: %s" % path)
        last_elapsed = max(elapsed_values)
        source_counts[path] = 0
        for row in rows:
            elapsed = _f(row, "elapsed")
            if elapsed is None or elapsed < args.exclude_start or elapsed > last_elapsed - args.exclude_end:
                continue
            ramp_scale = _f(row, "ramp_scale")
            if ramp_scale is not None and ramp_scale < args.min_ramp_scale:
                continue
            target_x = _f(row, "target_x")
            target_z = _f(row, "target_z")
            measured_x = _f(row, "measured_x")
            measured_z = _f(row, "measured_z")
            if None in (target_x, target_z, measured_x, measured_z):
                continue
            phase = 2.0 * math.pi * args.frequency * elapsed
            old_x_bias = _source_bias(row, "x")
            old_z_bias = _source_bias(row, "z")
            error_x = measured_x - target_x
            error_z = measured_z - target_z
            usable.append(
                {
                    "phase": phase,
                    "desired_x": old_x_bias - args.learning_rate * error_x,
                    "desired_z": old_z_bias - args.learning_rate * error_z,
                    "error_x": error_x,
                    "error_z": error_z,
                    "old_x_bias": old_x_bias,
                    "old_z_bias": old_z_bias,
                    "source": path,
                }
            )
            source_counts[path] += 1
    patterns = sorted(set(patterns))
    if len(patterns) != 1:
        raise RuntimeError("CSV pattern mismatch; pass matching CSVs or use --pattern")
    if patterns[0] not in SUPPORTED_PATTERNS:
        raise RuntimeError("unsupported pattern in CSV: %s" % patterns[0])
    return patterns[0], usable, source_counts


def _rmse(values):
    if not values:
        return float("nan")
    return math.sqrt(statistics.fmean([value * value for value in values]))


def main():
    parser = argparse.ArgumentParser(description="Fit a bounded SO101 periodic-path X/Z compensation profile")
    parser.add_argument("csv", nargs="+", help="One or more CSV logs from the same trajectory")
    parser.add_argument("--pattern", choices=SUPPORTED_PATTERNS, default="", help="Trajectory pattern; default: infer from CSV")
    parser.add_argument("--frequency", type=float, required=True, help="Hz used by the recorded test")
    parser.add_argument("--center", nargs=3, type=float, required=True, help="m, same center used by the recorded test")
    parser.add_argument("--x-amplitude", type=float, required=True, help="m, half width")
    parser.add_argument("--z-amplitude", type=float, required=True, help="m, half height")
    parser.add_argument("--exclude-start", type=float, default=10.0)
    parser.add_argument("--exclude-end", type=float, default=5.0)
    parser.add_argument("--min-ramp-scale", type=float, default=0.98)
    parser.add_argument("--harmonics", type=int, default=2)
    parser.add_argument("--learning-rate", type=float, default=0.5, help="0-1 update fraction; keep conservative on hardware")
    parser.add_argument("--max-abs-x-bias", type=float, default=0.006, help="m, hard safety bound")
    parser.add_argument("--max-abs-z-bias", type=float, default=0.012, help="m, hard safety bound")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.frequency <= 0.0 or args.harmonics < 0:
        parser.error("frequency must be positive and harmonics cannot be negative")
    if args.max_abs_x_bias <= 0.0 or args.max_abs_z_bias <= 0.0:
        parser.error("max bias limits must be positive")
    if not 0.0 < args.learning_rate <= 1.0:
        parser.error("learning-rate must be in (0, 1]")

    pattern, usable, source_counts = _load_usable_rows(args.csv, args)
    needed = max(20, 2 * args.harmonics + 3)
    if len(usable) < needed:
        raise RuntimeError("not enough usable rows after settling/ramp exclusion")

    x_coefficients, max_x = _fit_axis(
        [(row["phase"], row["desired_x"]) for row in usable],
        int(args.harmonics),
        float(args.max_abs_x_bias),
    )
    z_coefficients, max_z = _fit_axis(
        [(row["phase"], row["desired_z"]) for row in usable],
        int(args.harmonics),
        float(args.max_abs_z_bias),
    )

    payload = {
        "schema": "so101_phase_xz_compensation_v1",
        "warning": "bench-only; invalidate after payload, temperature, UAV attitude, or motion condition changes",
        "pattern": pattern,
        "frequency_hz": float(args.frequency),
        "center_xyz_m": [float(value) for value in args.center],
        "x_amplitude_m": float(args.x_amplitude),
        "z_amplitude_m": float(args.z_amplitude),
        "harmonics": int(args.harmonics),
        "learning_rate": float(args.learning_rate),
        "coefficients_x_m": x_coefficients,
        "coefficients_z_m": z_coefficients,
        "max_abs_x_bias_m": max_x,
        "max_abs_z_bias_m": max_z,
        "source_csvs": list(args.csv),
        "usable_samples": len(usable),
        "usable_samples_by_csv": source_counts,
        "source_error_x_rmse_m": _rmse([row["error_x"] for row in usable]),
        "source_error_z_rmse_m": _rmse([row["error_z"] for row in usable]),
        "source_x_bias_median_m": float(statistics.median([row["old_x_bias"] for row in usable])),
        "source_z_bias_median_m": float(statistics.median([row["old_z_bias"] for row in usable])),
    }
    with open(args.output, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")

    print("profile:", args.output)
    print("usable_samples:", len(usable))
    print("source_error_x_rmse_m: %.6f" % payload["source_error_x_rmse_m"])
    print("source_error_z_rmse_m: %.6f" % payload["source_error_z_rmse_m"])
    print("max_abs_x_bias_m: %.6f" % payload["max_abs_x_bias_m"])
    print("max_abs_z_bias_m: %.6f" % payload["max_abs_z_bias_m"])
    print("coefficients_x_m:", " ".join("%.7f" % value for value in x_coefficients))
    print("coefficients_z_m:", " ".join("%.7f" % value for value in z_coefficients))


if __name__ == "__main__":
    main()

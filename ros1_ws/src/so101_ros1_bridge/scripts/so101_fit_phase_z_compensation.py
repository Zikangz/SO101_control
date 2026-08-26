#!/usr/bin/env python3
"""Fit a bounded phase-dependent Z feedforward profile from one bench CSV.

The profile is intentionally tied to one xz_sine trajectory. It is a bench
calibration aid, not a general gravity controller and must not be used after
the arm is mounted on a moving or tilted UAV.
"""

import argparse
import csv
import json
import math
import statistics

import numpy as np


def _f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return None


def _profile_value(coefficients, phase, harmonics):
    value = float(coefficients[0])
    for k in range(1, harmonics + 1):
        value += float(coefficients[2 * k - 1]) * math.sin(k * phase)
        value += float(coefficients[2 * k]) * math.cos(k * phase)
    return value


def main():
    parser = argparse.ArgumentParser(description="Fit a bounded SO101 xz_sine Z compensation profile")
    parser.add_argument("csv")
    parser.add_argument("--frequency", type=float, required=True, help="Hz used by the recorded test")
    parser.add_argument("--center", nargs=3, type=float, required=True, help="m, same center used by the recorded test")
    parser.add_argument("--x-amplitude", type=float, required=True, help="m, half width")
    parser.add_argument("--z-amplitude", type=float, required=True, help="m, half height")
    parser.add_argument("--exclude-start", type=float, default=10.0)
    parser.add_argument("--exclude-end", type=float, default=5.0)
    parser.add_argument("--harmonics", type=int, default=3)
    parser.add_argument("--max-abs-bias", type=float, default=0.012, help="m, hard safety bound")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.frequency <= 0.0 or args.harmonics < 0 or args.max_abs_bias <= 0.0:
        parser.error("frequency, max-abs-bias must be positive and harmonics cannot be negative")

    with open(args.csv, newline="") as handle:
        rows = list(csv.DictReader(handle))
    elapsed_values = [_f(row, "elapsed") for row in rows]
    elapsed_values = [value for value in elapsed_values if value is not None]
    if not elapsed_values:
        raise RuntimeError("CSV has no numeric elapsed column")
    last_elapsed = max(elapsed_values)
    usable = []
    source_biases = []
    for row in rows:
        elapsed = _f(row, "elapsed")
        target_z = _f(row, "target_z")
        measured_z = _f(row, "measured_z")
        if elapsed is None or target_z is None or measured_z is None:
            continue
        if elapsed < args.exclude_start or elapsed > last_elapsed - args.exclude_end:
            continue
        ramp_scale = _f(row, "ramp_scale")
        if ramp_scale is not None and ramp_scale < 0.98:
            continue
        source_bias = _f(row, "z_feedforward_bias") or 0.0
        source_biases.append(source_bias)
        phase = 2.0 * math.pi * args.frequency * elapsed
        # measured - target = plant residual at the old command.  Subtracting
        # it from the old bias gives the next command bias for the nominal path.
        usable.append((phase, source_bias - (measured_z - target_z)))
    if len(usable) < max(20, 2 * args.harmonics + 3):
        raise RuntimeError("not enough usable rows after settling/ramp exclusion")

    harmonics = int(args.harmonics)
    matrix = []
    values = []
    for phase, value in usable:
        matrix.append([1.0] + [term for k in range(1, harmonics + 1) for term in (math.sin(k * phase), math.cos(k * phase))])
        values.append(value)
    coefficients, _, _, _ = np.linalg.lstsq(np.asarray(matrix), np.asarray(values), rcond=None)
    coefficients = [float(value) for value in coefficients]

    phases = np.linspace(0.0, 2.0 * math.pi, 2001)
    raw_values = [_profile_value(coefficients, float(phase), harmonics) for phase in phases]
    peak = max(abs(value) for value in raw_values)
    if peak > args.max_abs_bias:
        scale = args.max_abs_bias / peak
        coefficients = [value * scale for value in coefficients]
        raw_values = [_profile_value(coefficients, float(phase), harmonics) for phase in phases]

    payload = {
        "schema": "so101_phase_z_compensation_v1",
        "warning": "bench-only; invalidate after payload, temperature, UAV attitude, or motion condition changes",
        "pattern": "xz_sine",
        "frequency_hz": float(args.frequency),
        "center_xyz_m": [float(value) for value in args.center],
        "x_amplitude_m": float(args.x_amplitude),
        "z_amplitude_m": float(args.z_amplitude),
        "harmonics": harmonics,
        "coefficients_m": coefficients,
        "max_abs_bias_m": float(max(abs(value) for value in raw_values)),
        "source_bias_m": float(statistics.median(source_biases)) if source_biases else 0.0,
        "source_csv": args.csv,
        "usable_samples": len(usable),
    }
    with open(args.output, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print("profile:", args.output)
    print("usable_samples:", len(usable))
    print("max_abs_bias_m: %.6f" % payload["max_abs_bias_m"])
    print("coefficients_m:", " ".join("%.7f" % value for value in coefficients))


if __name__ == "__main__":
    main()

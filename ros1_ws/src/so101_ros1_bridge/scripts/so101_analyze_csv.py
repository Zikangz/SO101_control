#!/usr/bin/env python3
"""Analyze SO101 precision and sine-test CSV files."""

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict


def _unit_for(column):
    if column in ("elapsed", "t"):
        return "s"
    if column in ("target", "measured", "error"):
        return "rad"
    if column.endswith("_x") or column.endswith("_y") or column.endswith("_z"):
        prefixes_m = ("target_", "measured_", "error_", "offset_", "ee_")
        if column.startswith(prefixes_m):
            return "m"
    if column in ("error_norm", "ik_error_m", "mean_error_norm_m", "max_error_norm_m", "rmse_norm_m"):
        return "m"
    if column.startswith("planned_") or column.startswith("commanded_") or column.startswith("joint_tracking_error_"):
        return "rad"
    if column in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
        return "rad"
    if column == "gripper" or column.endswith("_gripper"):
        return "0-1"
    if "current_ma" in column:
        return "mA"
    if "load_raw" in column:
        return "raw"
    if "temperature_c" in column:
        return "C"
    if "voltage_v" in column:
        return "V"
    return "-"


def _read_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def _float_or_none(value):
    try:
        if value is None or value == "":
            return None
        return float(value)
    except ValueError:
        return None


def _numeric_columns(rows):
    columns = sorted({key for row in rows for key in row})
    result = []
    for col in columns:
        values = [_float_or_none(row.get(col)) for row in rows]
        values = [value for value in values if value is not None]
        if values:
            result.append((col, values))
    return result


def _stats(values):
    if not values:
        return {}
    return {
        "min": min(values),
        "max": max(values),
        "range": max(values) - min(values),
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def _values_for(rows, column):
    values = []
    for row in rows:
        value = _float_or_none(row.get(column))
        if value is not None:
            values.append(value)
    return values


def _mean_abs(values):
    if not values:
        return 0.0
    return statistics.fmean(abs(value) for value in values)


def _rms(values):
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(value * value for value in values))


def _max_abs(values):
    if not values:
        return 0.0
    return max(abs(value) for value in values)


def _mean_sample_dt(rows):
    elapsed = _values_for(rows, "elapsed")
    if len(elapsed) < 2:
        return None
    diffs = [elapsed[idx] - elapsed[idx - 1] for idx in range(1, len(elapsed)) if elapsed[idx] > elapsed[idx - 1]]
    if not diffs:
        return None
    return statistics.median(diffs)


def _moving_average_residual(values, window):
    if len(values) < 3:
        return []
    window = max(3, int(window))
    if window % 2 == 0:
        window += 1
    radius = window // 2
    residuals = []
    for idx, value in enumerate(values):
        lo = max(0, idx - radius)
        hi = min(len(values), idx + radius + 1)
        residuals.append(value - statistics.fmean(values[lo:hi]))
    return residuals


def _print_stats(title, stats_by_col, keys=None):
    print(title)
    selected = keys or sorted(stats_by_col)
    for key in selected:
        if key not in stats_by_col:
            continue
        s = stats_by_col[key]
        print(
            "  %-30s unit=%-4s range=% .6f std=% .6f mean=% .6f min=% .6f max=% .6f"
            % (key, _unit_for(key), s["range"], s["std"], s["mean"], s["min"], s["max"])
        )


def _analyze_sine(rows):
    if not rows or "target" not in rows[0] or "measured" not in rows[0]:
        return
    errors = []
    abs_errors = []
    for row in rows:
        target = _float_or_none(row.get("target"))
        measured = _float_or_none(row.get("measured"))
        if target is None or measured is None:
            continue
        err = measured - target
        errors.append(err)
        abs_errors.append(abs(err))
    if not errors:
        return
    rmse = math.sqrt(statistics.fmean([err * err for err in errors]))
    print("sine tracking error")
    print("  samples              %d" % len(errors))
    print("  mean_error           %.6f" % statistics.fmean(errors))
    print("  mean_abs_error       %.6f" % statistics.fmean(abs_errors))
    print("  max_abs_error        %.6f" % max(abs_errors))
    print("  rmse                 %.6f" % rmse)
    print("  suggested review     %s" % _tracking_review(max(abs_errors), rmse))


def _analyze_ee_sine(rows):
    if not rows or "target_x" not in rows[0] or "measured_x" not in rows[0]:
        return
    errors = []
    ik_errors = []
    ik_failures = 0
    ik_approximate = 0
    for row in rows:
        error_norm = _float_or_none(row.get("error_norm"))
        if error_norm is not None:
            errors.append(error_norm)
        ik_error = _float_or_none(row.get("ik_error_m"))
        if ik_error is not None:
            ik_errors.append(ik_error)
        commanded_raw = row.get("ik_commanded", row.get("ik_ok", ""))
        commanded = str(commanded_raw).lower()
        if commanded in ("false", "0"):
            ik_failures += 1
        approximate = str(row.get("ik_approximate", "")).lower()
        if approximate in ("true", "1"):
            ik_approximate += 1
    if not errors:
        return
    rmse = math.sqrt(statistics.fmean([err * err for err in errors]))
    print("end-effector tracking error")
    print("  samples              %d" % len(errors))
    print("  mean_error_norm_m    %.6f" % statistics.fmean(errors))
    print("  max_error_norm_m     %.6f" % max(errors))
    print("  rmse_norm_m          %.6f" % rmse)
    if ik_errors:
        print("  mean_ik_error_m      %.6f" % statistics.fmean(ik_errors))
        print("  max_ik_error_m       %.6f" % max(ik_errors))
    print("  ik_command_failures  %d" % ik_failures)
    print("  ik_approximate_cmds  %d" % ik_approximate)
    if max(errors) <= 0.006 and rmse <= 0.003 and ik_failures == 0:
        review = "good for low-speed desk tests"
    elif max(errors) <= 0.015 and rmse <= 0.008 and ik_failures == 0:
        review = "usable, but retest before aerial integration"
    else:
        review = "needs review: IK reachability, joint tracking, calibration, or command rate"
    print("  suggested review     %s" % review)
    _analyze_cartesian_bias(rows)
    _analyze_command_target_tracking(rows)
    _analyze_joint_tracking(rows)
    _analyze_smoothness_proxy(rows)


def _analyze_cartesian_bias(rows):
    error_x = _values_for(rows, "error_x")
    error_y = _values_for(rows, "error_y")
    error_z = _values_for(rows, "error_z")
    if not error_x or not error_z:
        return
    print("cartesian bias estimate")
    print("  mean_error_x_m       %.6f" % statistics.fmean(error_x))
    if error_y:
        print("  mean_error_y_m       %.6f" % statistics.fmean(error_y))
    print("  mean_error_z_m       %.6f" % statistics.fmean(error_z))
    print("  std_error_z_m        %.6f" % (statistics.pstdev(error_z) if len(error_z) > 1 else 0.0))
    # error_z = measured_z - target_z.  A negative value means the arm runs
    # lower than commanded, so the next command should add -mean(error_z).
    suggested_bias = -statistics.fmean(error_z)
    print("  suggested --z-feedforward-bias %.6f" % suggested_bias)


def _analyze_command_target_tracking(rows):
    """Separate commanded-IK tracking from nominal task-space bias.

    When --z-feedforward-bias is nonzero, error_x/y/z in the CSV intentionally
    remains measured minus the nominal target.  That is useful for evaluating
    task-space bias, but it must not be confused with tracking of the shifted
    Cartesian point given to IK.
    """
    axis_errors = {axis: [] for axis in ("x", "y", "z")}
    norms = []
    for row in rows:
        values = []
        for axis in ("x", "y", "z"):
            measured = _float_or_none(row.get("measured_" + axis))
            target = _float_or_none(row.get("command_target_" + axis))
            if measured is None or target is None:
                values = []
                break
            error = measured - target
            values.append((axis, error))
        if values:
            for axis, error in values:
                axis_errors[axis].append(error)
            norms.append(math.sqrt(sum(error * error for _axis, error in values)))
    if not norms:
        return
    print("shifted IK-command residual")
    print("  note                 command_target may include intentional position feedforward")
    print("  samples              %d" % len(norms))
    print("  mean_error_norm_m    %.6f" % statistics.fmean(norms))
    print("  max_error_norm_m     %.6f" % max(norms))
    print("  rmse_norm_m          %.6f" % _rms(norms))
    for axis in ("x", "y", "z"):
        if axis_errors[axis]:
            print("  mean_error_%s_m       %.6f" % (axis, statistics.fmean(axis_errors[axis])))


def _analyze_joint_tracking(rows):
    joints = ["shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    printed = False
    for joint in joints:
        errors = _values_for(rows, "joint_tracking_error_" + joint)
        if not errors:
            continue
        if not printed:
            print("joint tracking error")
            printed = True
        unit = "0-1" if joint == "gripper" else "rad"
        print(
            "  %-14s unit=%-3s mean=% .6f mean_abs=% .6f rmse=% .6f max_abs=% .6f"
            % (joint, unit, statistics.fmean(errors), _mean_abs(errors), _rms(errors), _max_abs(errors))
        )


def _analyze_smoothness_proxy(rows):
    dt = _mean_sample_dt(rows)
    if dt is None or dt <= 0.0:
        return
    window = max(3, int(round(0.5 / dt)))
    print("smoothness proxy")
    print("  sample_dt_s          %.4f" % dt)
    print("  highpass_window_s    %.2f" % (window * dt))
    for column in ("measured_x", "measured_z", "error_x", "error_z"):
        values = _values_for(rows, column)
        if len(values) < window:
            continue
        residuals = _moving_average_residual(values, window)
        print(
            "  %-20s highpass_rms_m=% .6f highpass_max_abs_m=% .6f"
            % (column, _rms(residuals), _max_abs(residuals))
        )
    for joint in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        for column in ("planned_" + joint, "commanded_" + joint, joint):
            values = _values_for(rows, column)
            if len(values) < window:
                continue
            residuals = _moving_average_residual(values, window)
            print(
                "  %-20s highpass_rms_rad=% .6f highpass_max_abs_rad=% .6f"
                % (column, _rms(residuals), _max_abs(residuals))
            )


def _tracking_review(max_abs_error, rmse):
    if max_abs_error <= 0.03 and rmse <= 0.015:
        return "good for low-speed desk tests"
    if max_abs_error <= 0.06 and rmse <= 0.03:
        return "usable, but retest before aerial integration"
    return "needs review: calibration, load, servo PID, torque/current limit, or command rate"


def _analyze_precision_groups(rows):
    if not rows or "pose" not in rows[0]:
        return
    groups = defaultdict(list)
    for row in rows:
        groups[row.get("pose", "")].append(row)
    keys = ["shoulder_lift", "elbow_flex", "wrist_flex", "gripper", "ee_x", "ee_y", "ee_z"]
    print("precision by pose")
    for pose, group_rows in sorted(groups.items()):
        stats_by_col = {col: _stats(values) for col, values in _numeric_columns(group_rows)}
        print("  pose=%s samples=%d" % (pose, len(group_rows)))
        for key in keys:
            if key in stats_by_col:
                print("    %-14s range=% .6f std=% .6f" % (key, stats_by_col[key]["range"], stats_by_col[key]["std"]))


def _analyze_hold_drift(rows):
    if not rows or "elapsed" in rows[0]:
        return
    numeric = {col: _stats(values) for col, values in _numeric_columns(rows)}
    joint_keys = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    ee_keys = ["ee_x", "ee_y", "ee_z"]
    joint_ranges = [numeric[key]["range"] for key in joint_keys if key in numeric]
    ee_ranges = [numeric[key]["range"] for key in ee_keys if key in numeric]
    if joint_ranges:
        print("hold drift review")
        print("  max_joint_range      %.6f" % max(joint_ranges))
        if ee_ranges:
            print("  max_ee_range_m       %.6f" % max(ee_ranges))
        if max(joint_ranges) <= 0.02:
            print("  suggested review     good hold repeatability")
        elif max(joint_ranges) <= 0.05:
            print("  suggested review     acceptable for slow desk tests; watch loaded poses")
        else:
            print("  suggested review     excessive drift; inspect torque, gravity load, calibration, and servo PID")


def _filter_elapsed(rows, ignore_start=0.0, ignore_end=0.0):
    if not rows:
        return rows
    if ignore_start <= 0.0 and ignore_end <= 0.0:
        return rows
    elapsed = _values_for(rows, "elapsed")
    if not elapsed:
        return rows
    max_elapsed = max(elapsed)
    filtered = []
    for row in rows:
        value = _float_or_none(row.get("elapsed"))
        if value is None:
            filtered.append(row)
            continue
        if value < ignore_start:
            continue
        if ignore_end > 0.0 and value > max_elapsed - ignore_end:
            continue
        filtered.append(row)
    return filtered


def analyze_file(path, ignore_start=0.0, ignore_end=0.0):
    print("file:", path)
    raw_rows = _read_rows(path)
    rows = _filter_elapsed(raw_rows, ignore_start=ignore_start, ignore_end=ignore_end)
    print("rows:", len(rows))
    if len(rows) != len(raw_rows):
        print("rows_raw:", len(raw_rows))
        print("ignored_elapsed_s: start=%.3f end=%.3f" % (ignore_start, ignore_end))
    print("units: Cartesian/EE values are metres; revolute joints and joint errors are radians; gripper is normalized 0-1; servo current is mA; voltage is V; temperature is C.")
    stats_by_col = {col: _stats(values) for col, values in _numeric_columns(rows)}
    preferred = [
        "elapsed",
        "target",
        "target_x",
        "target_y",
        "target_z",
        "measured",
        "measured_x",
        "measured_y",
        "measured_z",
        "error",
        "error_x",
        "error_y",
        "error_z",
        "error_norm",
        "ik_error_m",
        "ik_commanded",
        "ik_converged",
        "ik_approximate",
        "planned_shoulder_lift",
        "planned_elbow_flex",
        "planned_wrist_flex",
        "commanded_shoulder_lift",
        "commanded_elbow_flex",
        "commanded_wrist_flex",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "joint_tracking_error_shoulder_lift",
        "joint_tracking_error_elbow_flex",
        "joint_tracking_error_wrist_flex",
        "gripper",
        "ee_x",
        "ee_y",
        "ee_z",
        "servo_current_ma",
        "servo_load_raw",
        "servo_temperature_c",
        "servo_voltage_v",
        "servo_max_current_ma",
        "servo_max_abs_load_raw",
        "servo_max_temperature_c",
        "servo_min_voltage_v",
    ]
    _print_stats("numeric summary", stats_by_col, preferred)
    _analyze_sine(rows)
    _analyze_ee_sine(rows)
    _analyze_precision_groups(rows)
    _analyze_hold_drift(rows)


def build_parser():
    parser = argparse.ArgumentParser(description="Analyze SO101 CSV output")
    parser.add_argument("--ignore-start", type=float, default=0.0, help="Ignore rows with elapsed less than this many seconds")
    parser.add_argument("--ignore-end", type=float, default=0.0, help="Ignore this many seconds at the end when elapsed is present")
    parser.add_argument("csv", nargs="+")
    return parser


def main():
    args = build_parser().parse_args()
    try:
        for idx, path in enumerate(args.csv):
            if idx:
                print()
            analyze_file(path, ignore_start=max(0.0, args.ignore_start), ignore_end=max(0.0, args.ignore_end))
        return 0
    except OSError as exc:
        print("CSV analysis error: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

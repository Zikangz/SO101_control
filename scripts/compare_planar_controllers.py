from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "outputs" / ".matplotlib"))

import matplotlib.pyplot as plt
import numpy as np


SIM_SCRIPT = ROOT / "scripts" / "mujoco_planar_control_sim.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare planar SO101 MuJoCo trajectory controllers.")
    parser.add_argument(
        "--controllers",
        nargs="+",
        default=["joint_trajectory", "moveit_like", "moveit_ruckig", "argo_like", "argo_external"],
        choices=["joint_trajectory", "cartesian_stream", "moveit_like", "moveit_ruckig", "argo_like", "argo_external"],
    )
    parser.add_argument("--ee-frame", choices=["site_gripperframe", "body_gripper"], default="body_gripper")
    parser.add_argument("--limit-profile", choices=["config", "mujoco"], default="config")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--cycles", type=float, default=1.0)
    parser.add_argument("--frequency", type=float, default=0.05)
    parser.add_argument("--center", nargs=3, type=float, default=None, help="Absolute target center xyz passed to the simulator.")
    parser.add_argument("--x-amplitude", type=float, default=0.03)
    parser.add_argument("--z-amplitude", type=float, default=0.03)
    parser.add_argument("--plan-rate", type=float, default=20.0)
    parser.add_argument("--control-rate", type=float, default=100.0)
    parser.add_argument("--log-rate", type=float, default=100.0)
    parser.add_argument("--start-at-first-target", action="store_true")
    parser.add_argument("--plan-multistart-every-target", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs" / "planar_controller_comparison")
    parser.add_argument("--extra-sim-arg", action="append", default=[], help="Extra argument passed through to mujoco_planar_control_sim.py.")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def summarize(controller: str, csv_path: Path) -> dict[str, float | str]:
    rows = read_rows(csv_path)
    if not rows:
        raise RuntimeError("No rows written for %s" % controller)
    errors = np.array([float(row["tracking_error_xz_m"]) for row in rows], dtype=np.float64)
    joint_errors = []
    for row in rows:
        for key, value in row.items():
            if key.startswith("joint_tracking_error_"):
                joint_errors.append(float(value))
    joint_errors_np = np.array(joint_errors, dtype=np.float64) if joint_errors else np.zeros(1)
    return {
        "controller": controller,
        "samples": len(rows),
        "trajectory_duration_s": max(float(row.get("target_elapsed", row["elapsed"])) for row in rows),
        "total_elapsed_s": float(rows[-1]["elapsed"]),
        "mean_error_xz_m": float(errors.mean()),
        "max_error_xz_m": float(errors.max()),
        "final_error_xz_m": float(errors[-1]),
        "rms_joint_command_error_rad": float(np.sqrt(np.mean(joint_errors_np * joint_errors_np))),
        "csv": str(csv_path),
    }


def write_summary(rows: list[dict[str, float | str]], output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "summary.csv"
    keys = list(rows[0].keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    return path


def plot_comparison(summary_rows: list[dict[str, float | str]], output_dir: Path) -> Path:
    fig = plt.figure(figsize=(10, 7))
    ax_err = fig.add_subplot(2, 1, 1)
    ax_xz = fig.add_subplot(2, 1, 2)
    target_drawn = False
    for summary in summary_rows:
        controller = str(summary["controller"])
        rows = read_rows(Path(str(summary["csv"])))
        t = [float(row["elapsed"]) for row in rows]
        err = [float(row["tracking_error_xz_m"]) for row in rows]
        ax_err.plot(t, err, label=controller)
        if not target_drawn:
            ax_xz.plot(
                [float(row["target_x"]) for row in rows],
                [float(row["target_z"]) for row in rows],
                color="black",
                linestyle="--",
                label="target_xz",
            )
            target_drawn = True
        ax_xz.plot(
            [float(row["ee_x"]) for row in rows],
            [float(row["ee_z"]) for row in rows],
            label=controller,
        )
    ax_err.set_title("Planar SO101 Controller Comparison")
    ax_err.set_xlabel("time [s]")
    ax_err.set_ylabel("XZ error [m]")
    ax_err.legend()
    ax_xz.set_xlabel("x [m]")
    ax_xz.set_ylabel("z [m]")
    ax_xz.axis("equal")
    ax_xz.legend()
    fig.tight_layout()
    path = output_dir / "controller_comparison.png"
    fig.savefig(path, dpi=160)
    return path


def run_controller(args: argparse.Namespace, controller: str) -> Path:
    controller_dir = args.output_dir / controller
    cmd = [
        sys.executable,
        str(SIM_SCRIPT),
        "--controller",
        controller,
        "--cycles",
        str(args.cycles),
        "--frequency",
        str(args.frequency),
        "--x-amplitude",
        str(args.x_amplitude),
        "--z-amplitude",
        str(args.z_amplitude),
        "--ee-frame",
        args.ee_frame,
        "--limit-profile",
        args.limit_profile,
        "--plan-rate",
        str(args.plan_rate),
        "--control-rate",
        str(args.control_rate),
        "--log-rate",
        str(args.log_rate),
        "--output-dir",
        str(controller_dir),
    ]
    if args.duration is not None:
        cmd.extend(["--duration", str(args.duration)])
    if args.center is not None:
        cmd.extend(["--center", str(args.center[0]), str(args.center[1]), str(args.center[2])])
    if args.start_at_first_target:
        cmd.append("--start-at-first-target")
    if args.plan_multistart_every_target:
        cmd.append("--plan-multistart-every-target")
    cmd.extend(args.extra_sim_arg)
    subprocess.run(cmd, check=True)
    return controller_dir / ("%s.csv" % controller)


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = []
    for controller in args.controllers:
        csv_path = run_controller(args, controller)
        summary_rows.append(summarize(controller, csv_path))
    summary_path = write_summary(summary_rows, args.output_dir)
    plot_path = plot_comparison(summary_rows, args.output_dir)
    print("summary=%s" % summary_path)
    print("plot=%s" % plot_path)
    for row in summary_rows:
        print(
            "%s mean=%.6f max=%.6f final=%.6f"
            % (
                row["controller"],
                row["mean_error_xz_m"],
                row["max_error_xz_m"],
                row["final_error_xz_m"],
            )
        )


if __name__ == "__main__":
    main()

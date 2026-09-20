"""Publication-ready plots for the homework 19 evaluation artifacts."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .evaluation import PositionEvaluation


DISPLAY_NAMES = {
    "wheel_only": "Raw wheel odometry",
    "base": "E1 INS baseline (IMU + wheel)",
    "slip": "E2 kinematics + slip",
    "visual": "E3 + stereo VO",
    "full": "E4 + stereo VO + LiDAR",
}


def save_validation_plot(result: PositionEvaluation, output_path: Path) -> None:
    """Plot one aligned trajectory and position error at valid VRS epochs."""
    origin = result.reference_xy_m[0]
    reference = result.reference_xy_m - origin
    estimate = result.aligned_xy_m - origin
    elapsed_s = (result.timestamp_ns - result.timestamp_ns[0]) * 1e-9

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(reference[:, 0], reference[:, 1], label="VRS-GPS RTK", linewidth=2)
    axes[0].plot(estimate[:, 0], estimate[:, 1], label="Best fused, SE(2) aligned")
    axes[0].set_xlabel("UTM east offset (m)")
    axes[0].set_ylabel("UTM north offset (m)")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(elapsed_s, result.error_m, label="2D position error")
    axes[1].axhline(result.rmse_m, color="tab:red", linestyle="--", label="RMSE")
    axes[1].set_xlabel("Time since first matched RTK fix (s)")
    axes[1].set_ylabel("Position error (m)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def save_comparison_plots(
    evaluations: dict[str, PositionEvaluation], output_directory: Path
) -> None:
    """Write trajectory, error-over-time and RMSE comparison screenshots."""
    output_directory.mkdir(parents=True, exist_ok=True)
    first = next(iter(evaluations.values()))
    origin = first.reference_xy_m[0]

    figure, axis = plt.subplots(figsize=(8, 6.5))
    reference = first.reference_xy_m - origin
    axis.plot(
        reference[:, 0],
        reference[:, 1],
        color="black",
        linewidth=2.5,
        label="VRS-GPS RTK reference",
        zorder=10,
    )
    for name, evaluation in evaluations.items():
        estimate = evaluation.aligned_xy_m - origin
        axis.plot(
            estimate[:, 0],
            estimate[:, 1],
            linewidth=1.3,
            label=f"{DISPLAY_NAMES.get(name, name)} ({evaluation.rmse_m:.2f} m)",
        )
    axis.set_xlabel("UTM east offset (m)")
    axis.set_ylabel("UTM north offset (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / "trajectory_comparison.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 5))
    for name, evaluation in evaluations.items():
        elapsed_s = (evaluation.timestamp_ns - evaluation.timestamp_ns[0]) * 1e-9
        axis.plot(
            elapsed_s,
            evaluation.error_m,
            linewidth=1.2,
            label=DISPLAY_NAMES.get(name, name),
        )
    axis.set_xlabel("Time since first matched RTK fix (s)")
    axis.set_ylabel("2D ATE error (m)")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output_directory / "error_over_time.png", dpi=180)
    plt.close(figure)

    names = list(evaluations)
    values = [evaluations[name].rmse_m for name in names]
    figure, axis = plt.subplots(figsize=(8, 4.5))
    bars = axis.bar(
        [DISPLAY_NAMES.get(name, name) for name in names],
        values,
        color=plt.cm.viridis(np.linspace(0.15, 0.85, len(names))),
    )
    axis.bar_label(bars, fmt="%.2f m", padding=3)
    axis.set_ylabel("2D ATE RMSE (m)")
    axis.set_title("Whole-trajectory accuracy after rigid SE(2) alignment")
    axis.tick_params(axis="x", rotation=18)
    axis.grid(True, axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(output_directory / "rmse_comparison.png", dpi=180)
    plt.close(figure)


def _numeric_column(rows: list[dict[str, str]], name: str) -> tuple[np.ndarray, np.ndarray]:
    timestamps: list[int] = []
    values: list[float] = []
    for row in rows:
        value = row.get(name, "")
        if value:
            timestamps.append(int(row["timestamp_ns"]))
            values.append(float(value))
    return np.asarray(timestamps, dtype=np.int64), np.asarray(values)


def save_consistency_plot(
    output_root: Path,
    mode: str,
    *,
    wheel_speed_threshold: float,
    wheel_yaw_threshold: float,
    relative_speed_threshold: float,
    relative_yaw_threshold: float,
) -> None:
    """Plot all available one-dimensional NIS series and their gates."""
    wheel_rows = list(
        csv.DictReader(
            (output_root / f"diagnostics_{mode}.csv").open(encoding="utf-8")
        )
    )
    relative_rows = list(
        csv.DictReader(
            (output_root / f"diagnostics_relative_{mode}.csv").open(encoding="utf-8")
        )
    )
    series = [
        (*_numeric_column(wheel_rows, "speed_nis"), "Wheel speed NIS", wheel_speed_threshold),
        (*_numeric_column(wheel_rows, "yaw_rate_nis"), "Wheel yaw-rate NIS", wheel_yaw_threshold),
        (*_numeric_column(relative_rows, "speed_nis"), "VO/LiDAR speed NIS", relative_speed_threshold),
        (*_numeric_column(relative_rows, "yaw_rate_nis"), "VO/LiDAR yaw-rate NIS", relative_yaw_threshold),
    ]
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=False)
    for axis, (timestamps, values, label, threshold) in zip(axes.flat, series):
        if len(values):
            elapsed = (timestamps - timestamps[0]) * 1e-9
            axis.plot(elapsed, values, linewidth=0.7, label=label)
        axis.axhline(threshold, color="tab:red", linestyle="--", label="NIS gate")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("NIS (-)")
        axis.set_ylim(0.0, max(10.0, threshold * 2.0))
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle(f"Filter consistency diagnostics: {DISPLAY_NAMES.get(mode, mode)}")
    figure.tight_layout()
    figure.savefig(output_root / "screenshots" / "filter_consistency.png", dpi=180)
    plt.close(figure)


def save_slip_plot(output_root: Path, mode: str = "slip") -> None:
    """Visualize labelled encoder fault and debounced detector output."""
    rows = list(
        csv.DictReader(
            (output_root / f"diagnostics_{mode}.csv").open(encoding="utf-8")
        )
    )
    if not rows or not any(row["fault_injected"] == "True" for row in rows):
        return
    timestamps = np.array([int(row["timestamp_ns"]) for row in rows])
    elapsed = (timestamps - timestamps[0]) * 1e-9
    yaw_rate = np.array([float(row["wheel_yaw_rate_rad_s"]) for row in rows])
    injected = np.array([row["fault_injected"] == "True" for row in rows])
    detected = np.array([row["slip_active"] == "True" for row in rows])
    figure, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axes[0].plot(elapsed, yaw_rate, linewidth=0.8, label="Wheel yaw rate")
    axes[0].fill_between(
        elapsed,
        yaw_rate.min(),
        yaw_rate.max(),
        where=injected,
        alpha=0.2,
        color="tab:red",
        label="Injected right-wheel fault",
    )
    axes[0].set_ylabel("Yaw rate (rad/s)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].step(elapsed, injected.astype(int), where="post", label="Fault label")
    axes[1].step(elapsed, detected.astype(int), where="post", label="Slip detected")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Active (-)")
    axes[1].set_yticks((0, 1))
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    figure.tight_layout()
    screenshot_dir = output_root / "screenshots"
    screenshot_dir.mkdir(parents=True, exist_ok=True)
    figure.savefig(screenshot_dir / "slip_detection.png", dpi=180)
    plt.close(figure)


def save_metrics_table(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    """Render the metrics as a compact submission screenshot."""
    rows = []
    baseline = evaluations.get("wheel_only")
    for name, result in evaluations.items():
        improvement = "-"
        if baseline is not None and name != "wheel_only":
            improvement = f"{100.0 * (baseline.rmse_m - result.rmse_m) / baseline.rmse_m:.1f}%"
        rows.append(
            (
                DISPLAY_NAMES.get(name, name),
                f"{result.rmse_m:.3f}",
                f"{result.median_m:.3f}",
                f"{result.p95_m:.3f}",
                improvement,
            )
        )
    figure, axis = plt.subplots(figsize=(9.5, 0.55 * len(rows) + 1.8))
    axis.axis("off")
    table = axis.table(
        cellText=rows,
        colLabels=("Configuration", "RMSE (m)", "Median (m)", "P95 (m)", "vs raw wheel"),
        cellLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.45)
    axis.set_title("Homework 19: whole-trajectory VRS-GPS evaluation", pad=15)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(figure)

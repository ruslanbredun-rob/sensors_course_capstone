"""Publication-ready plots for the homework 19 evaluation artifacts."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.evaluation.metrics import PositionEvaluation


DISPLAY_NAMES = {
    "base": "INS baseline (IMU + wheel)",
    "visual": "Baseline + stereo VO",
    "lidar": "Baseline + LiDAR (no camera)",
    "full": "Baseline + stereo VO + LiDAR",
}


def _save_figure(figure: plt.Figure, output_path: Path, *, tight: bool = True) -> None:
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight" if tight else None)
    plt.close(figure)


def save_validation_plot(result: PositionEvaluation, output_path: Path) -> None:
    """Plot one aligned trajectory and position error at valid VRS epochs."""
    origin = result.reference_xy_m[0]
    reference = result.reference_xy_m - origin
    estimate = result.start_aligned_xy_m - origin
    elapsed_s = (result.timestamp_ns - result.timestamp_ns[0]) * 1e-9

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(reference[:, 0], reference[:, 1], label="VRS-GPS RTK", linewidth=2)
    axes[0].plot(estimate[:, 0], estimate[:, 1], label="Best fused, initial pose aligned")
    axes[0].set_xlabel("UTM east offset (m)")
    axes[0].set_ylabel("UTM north offset (m)")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(elapsed_s, result.start_error_m, label="2D position error")
    axes[1].axhline(
        result.start_rmse_m, color="tab:red", linestyle="--", label="RMSE"
    )
    axes[1].set_xlabel("Time since first matched RTK fix (s)")
    axes[1].set_ylabel("Position error (m)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    _save_figure(figure, output_path, tight=False)


def _save_trajectory_comparison(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    first = next(iter(evaluations.values()))
    origin = first.reference_xy_m[0]
    figure, axis = plt.subplots(figsize=(8, 6.5))
    reference = first.reference_xy_m - origin
    axis.plot(
        reference[:, 0],
        reference[:, 1],
        color="black",
        linewidth=2.0,
        linestyle="--",
        alpha=0.75,
        label="VRS-GPS RTK reference",
        zorder=1,
    )
    for name, result in evaluations.items():
        estimate = result.start_aligned_xy_m - origin
        axis.plot(
            estimate[:, 0],
            estimate[:, 1],
            linewidth=1.5,
            label=(
                f"{DISPLAY_NAMES.get(name, name)} "
                f"(ATE {result.rmse_m:.2f} m)"
            ),
        )
    axis.scatter(
        0.0,
        0.0,
        marker="*",
        s=180,
        color="black",
        edgecolor="white",
        linewidth=0.9,
        zorder=10,
        label="Common start",
    )
    axis.annotate(
        "start (0, 0)",
        (0.0, 0.0),
        xytext=(10, 10),
        textcoords="offset points",
        fontsize=8,
    )
    axis.set(xlabel="UTM east offset (m)", ylabel="UTM north offset (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.set_title("All trajectories aligned to the common initial pose (20 m heading)")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)
    _save_figure(figure, output_path)


def _save_error_comparison(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    figure, axis = plt.subplots(figsize=(9, 5))
    for name, result in evaluations.items():
        elapsed_s = (result.timestamp_ns - result.timestamp_ns[0]) * 1e-9
        axis.plot(
            elapsed_s,
            result.start_error_m,
            linewidth=1.2,
            label=DISPLAY_NAMES.get(name, name),
        )
    axis.set(
        xlabel="Time since first matched RTK fix (s)",
        ylabel="Initial-pose-aligned 2D error (m)",
    )
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)
    _save_figure(figure, output_path)


def _save_rmse_comparison(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    names = list(evaluations)
    global_values = [evaluations[name].rmse_m for name in names]
    start_values = [evaluations[name].start_rmse_m for name in names]
    figure, axis = plt.subplots(figsize=(8, 4.5))
    positions = np.arange(len(names))
    width = 0.38
    global_bars = axis.bar(
        positions - width / 2, global_values, width, label="Global SE(2) ATE"
    )
    start_bars = axis.bar(
        positions + width / 2, start_values, width, label="Initial pose aligned"
    )
    axis.bar_label(global_bars, fmt="%.1f", padding=3, fontsize=8)
    axis.bar_label(start_bars, fmt="%.1f", padding=3, fontsize=8)
    axis.set_ylabel("2D ATE RMSE (m)")
    axis.set_title("Whole-trajectory accuracy")
    axis.set_xticks(positions, [DISPLAY_NAMES.get(name, name) for name in names])
    axis.tick_params(axis="x", rotation=18)
    axis.legend()
    axis.grid(True, axis="y", alpha=0.3)
    _save_figure(figure, output_path)


def save_comparison_plots(
    evaluations: dict[str, PositionEvaluation], output_directory: Path
) -> None:
    """Write trajectory, error-over-time and RMSE comparison screenshots."""
    output_directory.mkdir(parents=True, exist_ok=True)
    _save_trajectory_comparison(
        evaluations, output_directory / "trajectory_comparison.png"
    )
    _save_error_comparison(evaluations, output_directory / "error_over_time.png")
    _save_rmse_comparison(evaluations, output_directory / "rmse_comparison.png")


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
    relative_pose_threshold: float,
    max_covariance_scale: float,
) -> None:
    """Plot wheel gates and adaptive relative-pose consistency diagnostics."""
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
        (
            *_numeric_column(wheel_rows, "speed_nis"),
            "Wheel speed NIS",
            wheel_speed_threshold,
        ),
        (
            *_numeric_column(wheel_rows, "yaw_rate_nis"),
            "Wheel yaw-rate NIS",
            wheel_yaw_threshold,
        ),
        (
            *_numeric_column(relative_rows, "pose_nis"),
            "VO/LiDAR pose NIS after adaptation",
            relative_pose_threshold,
        ),
        (
            *_numeric_column(relative_rows, "covariance_scale"),
            "Adaptive covariance scale",
            max_covariance_scale,
        ),
    ]
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=False)
    for axis, (timestamps, values, label, threshold) in zip(axes.flat, series):
        if len(values):
            elapsed = (timestamps - timestamps[0]) * 1e-9
            axis.plot(elapsed, values, linewidth=0.7, label=label)
        axis.axhline(threshold, color="tab:red", linestyle="--", label="NIS gate")
        axis.set_xlabel("Time (s)")
        axis.set_ylabel("NIS (-)")
        upper = max(10.0, threshold * 2.0)
        if len(values):
            upper = min(upper, max(10.0, float(np.percentile(values, 99)) * 1.2))
        axis.set_ylim(0.0, upper)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize=8)
    figure.suptitle(f"Filter consistency diagnostics: {DISPLAY_NAMES.get(mode, mode)}")
    figure.tight_layout()
    figure.savefig(output_root / "screenshots" / "filter_consistency.png", dpi=180)
    plt.close(figure)


def save_metrics_table(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    """Render the metrics as a compact submission screenshot."""
    rows = []
    baseline = evaluations.get("base")
    for name, result in evaluations.items():
        improvement = "-"
        if baseline is not None and name != "base":
            improvement = f"{100.0 * (baseline.rmse_m - result.rmse_m) / baseline.rmse_m:.1f}%"
        rows.append(
            (
                DISPLAY_NAMES.get(name, name),
                f"{result.rmse_m:.3f}",
                f"{result.start_rmse_m:.3f}",
                f"{result.median_m:.3f}",
                f"{result.p95_m:.3f}",
                improvement,
            )
        )
    figure, axis = plt.subplots(figsize=(11.5, 0.55 * len(rows) + 1.8))
    axis.axis("off")
    table = axis.table(
        cellText=rows,
        colLabels=(
            "Configuration",
            "Global RMSE (m)",
            "Initial-pose RMSE (m)",
            "Median (m)",
            "P95 (m)",
            "vs E1",
        ),
        colWidths=(0.30, 0.14, 0.14, 0.12, 0.12, 0.12),
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

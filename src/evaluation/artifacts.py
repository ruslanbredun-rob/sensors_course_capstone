"""Export evaluation tables and figures."""

from __future__ import annotations

import csv
from pathlib import Path

from src.common.config import RunConfig
from src.evaluation.metrics import PositionEvaluation
from src.evaluation.visualization import (
    save_comparison_plots,
    save_consistency_plot,
    save_metrics_table,
    save_slip_plot,
    save_validation_plot,
)


def write_metrics_csv(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "configuration",
                "matched_vrs_epochs",
                "rmse_m",
                "median_m",
                "p95_m",
                "final_error_m",
                "start_rmse_m",
                "start_median_m",
                "start_p95_m",
                "start_final_error_m",
            )
        )
        for name, result in evaluations.items():
            writer.writerow(
                (
                    name,
                    result.matched_epochs,
                    f"{result.rmse_m:.6f}",
                    f"{result.median_m:.6f}",
                    f"{result.p95_m:.6f}",
                    f"{result.final_error_m:.6f}",
                    f"{result.start_rmse_m:.6f}",
                    f"{result.start_median_m:.6f}",
                    f"{result.start_p95_m:.6f}",
                    f"{result.start_final_error_m:.6f}",
                )
            )


def write_validation_pairs_csv(
    evaluations: dict[str, PositionEvaluation], output_path: Path
) -> None:
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "configuration",
                "timestamp_ns",
                "estimate_time_offset_ms",
                "vrs_utm_easting_m",
                "vrs_utm_northing_m",
                "aligned_est_easting_m",
                "aligned_est_northing_m",
                "error_m",
                "start_aligned_est_easting_m",
                "start_aligned_est_northing_m",
                "start_error_m",
            )
        )
        for name, result in evaluations.items():
            for index, timestamp in enumerate(result.timestamp_ns):
                writer.writerow(
                    (
                        name,
                        timestamp,
                        f"{result.time_offset_ms[index]:.3f}",
                        f"{result.reference_xy_m[index, 0]:.4f}",
                        f"{result.reference_xy_m[index, 1]:.4f}",
                        f"{result.aligned_xy_m[index, 0]:.4f}",
                        f"{result.aligned_xy_m[index, 1]:.4f}",
                        f"{result.error_m[index]:.4f}",
                        f"{result.start_aligned_xy_m[index, 0]:.4f}",
                        f"{result.start_aligned_xy_m[index, 1]:.4f}",
                        f"{result.start_error_m[index]:.4f}",
                    )
                )


def _save_result_plots(
    evaluations: dict[str, PositionEvaluation], output: Path
) -> None:
    screenshots = output / "screenshots"
    screenshots.mkdir(parents=True, exist_ok=True)
    save_comparison_plots(evaluations, screenshots)
    save_metrics_table(evaluations, screenshots / "metrics_summary.png")
    best = min(evaluations.values(), key=lambda result: result.rmse_m)
    save_validation_plot(best, output / "trajectory_validation.png")


def _save_diagnostic_plots(
    config: RunConfig, evaluations: dict[str, PositionEvaluation]
) -> None:
    mode = next(
        name
        for name in ("full", "lidar", "visual", "slip", "base")
        if name in evaluations
    )
    output = config.general.output
    save_consistency_plot(
        output,
        mode,
        wheel_speed_threshold=config.wheel.speed_nis_threshold,
        wheel_yaw_threshold=config.wheel.yaw_nis_threshold,
        relative_pose_threshold=config.fusion.relative_pose_nis_threshold,
        max_covariance_scale=config.fusion.relative_pose_max_covariance_scale,
    )
    save_slip_plot(output, mode)


def export_validation_artifacts(
    config: RunConfig, evaluations: dict[str, PositionEvaluation]
) -> None:
    """Write numeric tables and plots without generating narrative reports."""
    if not evaluations:
        return
    output = config.general.output
    write_metrics_csv(evaluations, output / "comparison_metrics.csv")
    write_validation_pairs_csv(evaluations, output / "validation_pairs.csv")
    _save_result_plots(evaluations, output)
    _save_diagnostic_plots(config, evaluations)

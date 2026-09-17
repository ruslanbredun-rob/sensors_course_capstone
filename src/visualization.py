"""Plots for trajectory validation and later experiment comparisons."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .evaluation import PositionEvaluation
from .models import Estimate, PositionReference


def save_validation_plot(result: PositionEvaluation, output_path: Path) -> None:
    """Plot aligned trajectory and position error at valid VRS epochs."""
    origin = result.reference_xy_m[0]
    reference = result.reference_xy_m - origin
    estimate = result.aligned_xy_m - origin
    elapsed_s = (result.timestamp_ns - result.timestamp_ns[0]) * 1e-9

    figure, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(reference[:, 0], reference[:, 1], label="VRS-GPS RTK", linewidth=2)
    axes[0].plot(estimate[:, 0], estimate[:, 1], label="EKF, SE(2) aligned")
    axes[0].set_xlabel("UTM east offset (m)")
    axes[0].set_ylabel("UTM north offset (m)")
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(elapsed_s, result.error_m, label="2D position error")
    axes[1].axhline(result.rmse_m, color="tab:red", linestyle="--", label="RMSE")
    axes[1].set_xlabel("Time since first matched RTK fix (s)")
    axes[1].set_ylabel("Position error (m)")
    axes[1].legend()
    axes[1].grid(True)

    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    plt.close(figure)


def save_trajectory_plot(
    baseline: Iterable[Estimate],
    fused: Iterable[Estimate],
    reference: Iterable[PositionReference],
    output_path: Path,
) -> None:
    """Save trajectory PNG with axes in metres and legend."""
    raise NotImplementedError("Implement after evaluation frames are fixed")

"""Result plots for homework 19."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

from .models import Estimate, PositionReference


def save_trajectory_plot(
    baseline: Iterable[Estimate],
    fused: Iterable[Estimate],
    reference: Iterable[PositionReference],
    output_path: Path,
) -> None:
    """Save trajectory PNG with axes in metres and legend."""
    raise NotImplementedError("Implement after evaluation frames are fixed")

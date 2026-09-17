"""Orchestrates sources, estimator and phase-specific outputs."""

from __future__ import annotations

from .config import RunConfig


def run(config: RunConfig, mode: str) -> None:
    """Run Base for homework 18; extend with evaluation and E2-E4 for homework 19."""
    if not config.dataset.is_dir():
        raise FileNotFoundError(
            f"Dataset directory missing: {config.dataset}. See data/README.md."
        )
    raise NotImplementedError(
        "Project scaffold only: dataset readers and EKF are not implemented yet. "
        "See docs/roadmap.md."
    )

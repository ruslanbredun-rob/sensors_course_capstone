"""VRS-only trajectory validation; reference data never enter the EKF."""

from __future__ import annotations

from bisect import bisect_left
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from .models import Estimate, PositionReference


@dataclass(frozen=True)
class PositionEvaluation:
    """SE(2)-aligned trajectory error, with no scale adjustment."""

    timestamp_ns: np.ndarray
    time_offset_ms: np.ndarray
    reference_xy_m: np.ndarray
    aligned_xy_m: np.ndarray
    error_m: np.ndarray
    valid_fix_epochs: int
    matched_epochs: int
    rmse_m: float
    median_m: float
    p95_m: float
    final_error_m: float
    alignment_yaw_rad: float


def evaluate_position(
    estimates: Iterable[Estimate],
    reference: Iterable[PositionReference],
    *,
    valid_fix_state: int,
    tolerance_ns: int,
) -> PositionEvaluation:
    """Match valid VRS epochs to nearest estimates and compute aligned 2D ATE.

    The rigid alignment uses all matched points because the GPS-denied local
    trajectory has arbitrary origin and yaw. VRS data are used only here.
    """
    if tolerance_ns <= 0:
        raise ValueError("tolerance_ns must be positive")
    states = list(estimates)
    if len(states) < 2:
        raise ValueError("At least two estimated states are required")
    times = [state.timestamp_ns for state in states]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Estimate timestamps must be strictly increasing")

    reference_valid = [sample for sample in reference if sample.fix_state == valid_fix_state]
    pairs: list[tuple[PositionReference, Estimate, int]] = []
    for sample in reference_valid:
        insertion = bisect_left(times, sample.timestamp_ns)
        candidates = [
            index for index in (insertion - 1, insertion) if 0 <= index < len(states)
        ]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda index: abs(times[index] - sample.timestamp_ns))
        offset_ns = times[nearest] - sample.timestamp_ns
        if abs(offset_ns) <= tolerance_ns:
            pairs.append((sample, states[nearest], offset_ns))
    if len(pairs) < 3:
        raise ValueError(
            f"Only {len(pairs)} valid VRS epochs matched within "
            f"{tolerance_ns / 1e6:.1f} ms; at least 3 are required"
        )

    estimated_xy = np.array([[state.x_m, state.y_m] for _, state, _ in pairs])
    reference_xy = np.array([[fix.x_m, fix.y_m] for fix, _, _ in pairs])
    estimated_center = estimated_xy.mean(axis=0)
    reference_center = reference_xy.mean(axis=0)
    centered_estimate = estimated_xy - estimated_center
    centered_reference = reference_xy - reference_center
    if np.linalg.norm(centered_estimate) < 1.0:
        raise ValueError("Estimated trajectory has insufficient spatial extent")

    # Row-vector Kabsch alignment: no scale fit, reflection forbidden.
    u, _, vt = np.linalg.svd(centered_estimate.T @ centered_reference)
    correction = np.diag([1.0, np.linalg.det(u @ vt)])
    rotation = u @ correction @ vt
    aligned_xy = centered_estimate @ rotation + reference_center
    error = np.linalg.norm(aligned_xy - reference_xy, axis=1)
    return PositionEvaluation(
        timestamp_ns=np.array([fix.timestamp_ns for fix, _, _ in pairs], dtype=np.int64),
        time_offset_ms=np.array([offset / 1e6 for _, _, offset in pairs]),
        reference_xy_m=reference_xy,
        aligned_xy_m=aligned_xy,
        error_m=error,
        valid_fix_epochs=len(reference_valid),
        matched_epochs=len(pairs),
        rmse_m=float(np.sqrt(np.mean(error**2))),
        median_m=float(np.median(error)),
        p95_m=float(np.percentile(error, 95)),
        final_error_m=float(error[-1]),
        alignment_yaw_rad=float(np.arctan2(rotation[0, 1], rotation[0, 0])),
    )


def position_rmse(
    estimates: Iterable[Estimate],
    reference: Iterable[PositionReference],
    *,
    valid_fix_state: int,
    tolerance_ns: int,
) -> float:
    """Return 2D RMSE after rigid SE(2) alignment at matched valid VRS epochs."""
    return evaluate_position(
        estimates,
        reference,
        valid_fix_state=valid_fix_state,
        tolerance_ns=tolerance_ns,
    ).rmse_m

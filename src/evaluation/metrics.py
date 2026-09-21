"""VRS-only trajectory validation; reference data never enter the EKF."""

from __future__ import annotations

import math
from bisect import bisect_left
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np

from src.common.models import Estimate, PositionReference


@dataclass(frozen=True)
class PositionEvaluation:
    """SE(2)-aligned trajectory error, with no scale adjustment."""

    timestamp_ns: np.ndarray
    time_offset_ms: np.ndarray
    reference_xy_m: np.ndarray
    aligned_xy_m: np.ndarray
    error_m: np.ndarray
    start_aligned_xy_m: np.ndarray
    start_error_m: np.ndarray
    valid_fix_epochs: int
    matched_epochs: int
    rmse_m: float
    median_m: float
    p95_m: float
    final_error_m: float
    start_rmse_m: float
    start_median_m: float
    start_p95_m: float
    start_final_error_m: float
    alignment_yaw_rad: float
    start_alignment_yaw_rad: float
    start_alignment_baseline_m: float


@dataclass(frozen=True)
class RtkSegmentEvaluation:
    segment_id: int
    start_timestamp_ns: int
    end_timestamp_ns: int
    epochs: int
    duration_s: float
    global_rmse_m: float
    initial_pose_rmse_m: float
    global_final_error_m: float
    initial_pose_final_error_m: float


def evaluate_rtk_segments(
    result: PositionEvaluation,
    *,
    max_gap_ns: int,
    min_epochs: int,
) -> list[RtkSegmentEvaluation]:
    """Summarize errors on continuous matched RTK intervals."""
    if max_gap_ns <= 0 or min_epochs < 2:
        raise ValueError("Invalid RTK segment thresholds")
    boundaries = [0]
    boundaries.extend(
        int(index + 1)
        for index, gap in enumerate(np.diff(result.timestamp_ns))
        if gap > max_gap_ns
    )
    boundaries.append(len(result.timestamp_ns))
    segments: list[RtkSegmentEvaluation] = []
    for start, end in zip(boundaries, boundaries[1:]):
        if end - start < min_epochs:
            continue
        global_error = result.error_m[start:end]
        initial_error = result.start_error_m[start:end]
        segments.append(
            RtkSegmentEvaluation(
                segment_id=len(segments) + 1,
                start_timestamp_ns=int(result.timestamp_ns[start]),
                end_timestamp_ns=int(result.timestamp_ns[end - 1]),
                epochs=end - start,
                duration_s=float(
                    (result.timestamp_ns[end - 1] - result.timestamp_ns[start]) * 1e-9
                ),
                global_rmse_m=float(np.sqrt(np.mean(global_error**2))),
                initial_pose_rmse_m=float(np.sqrt(np.mean(initial_error**2))),
                global_final_error_m=float(global_error[-1]),
                initial_pose_final_error_m=float(initial_error[-1]),
            )
        )
    return segments


def _initial_pose_alignment(
    estimated_xy: np.ndarray,
    reference_xy: np.ndarray,
    *,
    target_baseline_m: float = 20.0,
) -> tuple[np.ndarray, float, float]:
    """Align origin and initial direction using the first stable motion segment."""
    estimated_offset = estimated_xy - estimated_xy[0]
    reference_offset = reference_xy - reference_xy[0]
    estimated_distance = np.linalg.norm(estimated_offset, axis=1)
    reference_distance = np.linalg.norm(reference_offset, axis=1)
    maximum_reference_distance = float(np.max(reference_distance))
    if maximum_reference_distance <= 0.0:
        raise ValueError("Reference trajectory has insufficient spatial extent")

    # Twenty metres suppresses RTK jitter while the vehicle is stationary. For
    # short synthetic/test trajectories, use one quarter of their extent.
    requested_baseline = min(target_baseline_m, 0.25 * maximum_reference_distance)
    minimum_estimated_distance = max(0.1, 0.1 * requested_baseline)
    candidates = np.flatnonzero(
        (reference_distance >= requested_baseline)
        & (estimated_distance >= minimum_estimated_distance)
    )
    if not len(candidates):
        raise ValueError("No stable initial-motion segment is available for alignment")
    heading_index = int(candidates[0])
    estimated_heading = math.atan2(
        estimated_offset[heading_index, 1], estimated_offset[heading_index, 0]
    )
    reference_heading = math.atan2(
        reference_offset[heading_index, 1], reference_offset[heading_index, 0]
    )
    yaw = math.atan2(
        math.sin(reference_heading - estimated_heading),
        math.cos(reference_heading - estimated_heading),
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array(((cosine, sine), (-sine, cosine)))
    aligned_xy = estimated_offset @ rotation + reference_xy[0]
    return aligned_xy, yaw, float(reference_distance[heading_index])


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
    # Align the initial pose independently of the global ATE fit. The first VRS
    # samples can be stationary, so estimate heading over a stable motion
    # baseline rather than from the noisy first pair of positions.
    (
        start_aligned_xy,
        start_alignment_yaw,
        start_alignment_baseline,
    ) = _initial_pose_alignment(
        estimated_xy,
        reference_xy,
    )
    start_error = np.linalg.norm(start_aligned_xy - reference_xy, axis=1)
    return PositionEvaluation(
        timestamp_ns=np.array([fix.timestamp_ns for fix, _, _ in pairs], dtype=np.int64),
        time_offset_ms=np.array([offset / 1e6 for _, _, offset in pairs]),
        reference_xy_m=reference_xy,
        aligned_xy_m=aligned_xy,
        error_m=error,
        start_aligned_xy_m=start_aligned_xy,
        start_error_m=start_error,
        valid_fix_epochs=len(reference_valid),
        matched_epochs=len(pairs),
        rmse_m=float(np.sqrt(np.mean(error**2))),
        median_m=float(np.median(error)),
        p95_m=float(np.percentile(error, 95)),
        final_error_m=float(error[-1]),
        start_rmse_m=float(np.sqrt(np.mean(start_error**2))),
        start_median_m=float(np.median(start_error)),
        start_p95_m=float(np.percentile(start_error, 95)),
        start_final_error_m=float(start_error[-1]),
        alignment_yaw_rad=float(np.arctan2(rotation[0, 1], rotation[0, 0])),
        start_alignment_yaw_rad=start_alignment_yaw,
        start_alignment_baseline_m=start_alignment_baseline,
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

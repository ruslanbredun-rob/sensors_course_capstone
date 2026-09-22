"""Sparse commercial GPS correction of an existing VIO trajectory."""

from __future__ import annotations

import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.common.config import RunConfig
from src.common.models import Estimate, GpsMeasurement


@dataclass(frozen=True)
class GpsTrajectoryFusionResult:
    states: list[Estimate]
    updates: int
    rejected_updates: int
    available_updates: int


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate_state(states: list[Estimate], times: list[int], timestamp_ns: int) -> Estimate:
    insertion = bisect_left(times, timestamp_ns)
    if insertion == 0:
        return states[0]
    if insertion == len(states):
        return states[-1]
    previous, following = states[insertion - 1], states[insertion]
    ratio = (timestamp_ns - previous.timestamp_ns) / (
        following.timestamp_ns - previous.timestamp_ns
    )
    yaw_delta = _wrap(following.yaw_rad - previous.yaw_rad)
    return Estimate(
        timestamp_ns,
        previous.x_m + ratio * (following.x_m - previous.x_m),
        previous.y_m + ratio * (following.y_m - previous.y_m),
        _wrap(previous.yaw_rad + ratio * yaw_delta),
        previous.speed_m_s + ratio * (following.speed_m_s - previous.speed_m_s),
    )


def _antenna_position(state: Estimate, offset_xy_m: np.ndarray) -> np.ndarray:
    cosine, sine = math.cos(state.yaw_rad), math.sin(state.yaw_rad)
    rotation = np.array(((cosine, -sine), (sine, cosine)))
    return np.array((state.x_m, state.y_m)) + rotation @ offset_xy_m


def _initial_alignment(
    gps: list[GpsMeasurement],
    base: list[Estimate],
    offset_xy_m: np.ndarray,
    distance_m: float,
) -> tuple[int, np.ndarray, np.ndarray, np.ndarray]:
    base_antenna = np.array(
        [_antenna_position(state, offset_xy_m) for state in base]
    )
    gps_xy = np.array(
        [(measurement.easting_m, measurement.northing_m) for measurement in gps]
    )
    base_distance = np.linalg.norm(base_antenna - base_antenna[0], axis=1)
    gps_distance = np.linalg.norm(gps_xy - gps_xy[0], axis=1)
    candidates = np.flatnonzero(
        (base_distance >= distance_m) & (gps_distance >= 0.5 * distance_m)
    )
    if not len(candidates):
        raise ValueError("Sparse GPS has no segment for initial frame alignment")
    alignment_index = int(candidates[0])
    global_points = gps_xy[: alignment_index + 1]
    local_points = base_antenna[: alignment_index + 1]
    global_center = global_points.mean(axis=0)
    local_center = local_points.mean(axis=0)
    u, _, vt = np.linalg.svd(
        (global_points - global_center).T @ (local_points - local_center)
    )
    correction = np.diag((1.0, np.linalg.det(u @ vt)))
    row_rotation = u @ correction @ vt
    return alignment_index, global_center, local_center, row_rotation


def fuse_sparse_gps_with_vio(
    config: RunConfig,
    states: list[Estimate],
    gps: list[GpsMeasurement],
    antenna_offset_xy_m: np.ndarray,
) -> GpsTrajectoryFusionResult:
    """Estimate a slowly varying SE(2) correction over the VIO trajectory."""
    if len(states) < 2 or not gps:
        raise ValueError("VIO states and sparse GPS measurements are required")
    times = [state.timestamp_ns for state in states]
    gps = [
        measurement
        for measurement in gps
        if times[0] <= measurement.timestamp_ns <= times[-1]
    ]
    base_at_gps = [
        _interpolate_state(states, times, measurement.timestamp_ns)
        for measurement in gps
    ]
    alignment_index, global_center, local_center, row_rotation = _initial_alignment(
        gps,
        base_at_gps,
        antenna_offset_xy_m,
        config.gps.initial_alignment_distance_m,
    )
    origin = np.array((states[0].x_m, states[0].y_m))
    correction = np.zeros(3, dtype=float)
    initial_yaw_std = config.gps.min_position_std_m / max(
        config.gps.initial_alignment_distance_m, 1.0
    )
    covariance = np.diag(
        (
            config.gps.min_position_std_m**2,
            config.gps.min_position_std_m**2,
            initial_yaw_std**2,
        )
    )
    next_gps = alignment_index
    last_update_ns = gps[alignment_index].timestamp_ns
    updates = rejected = 0
    diagnostics: list[tuple] = []
    corrected_states: list[Estimate] = []

    for state in states:
        while next_gps < len(gps) and gps[next_gps].timestamp_ns <= state.timestamp_ns:
            measurement = gps[next_gps]
            base_state = base_at_gps[next_gps]
            dt_s = max(0.0, (measurement.timestamp_ns - last_update_ns) * 1e-9)
            covariance += np.diag(
                (
                    config.gps.trajectory_translation_walk_m_sqrt_s**2 * dt_s,
                    config.gps.trajectory_translation_walk_m_sqrt_s**2 * dt_s,
                    config.gps.trajectory_yaw_walk_rad_sqrt_s**2 * dt_s,
                )
            )
            gps_global = np.array((measurement.easting_m, measurement.northing_m))
            measured_local = (gps_global - global_center) @ row_rotation + local_center
            base_antenna = _antenna_position(base_state, antenna_offset_xy_m)
            relative = base_antenna - origin
            cosine, sine = math.cos(correction[2]), math.sin(correction[2])
            rotation = np.array(((cosine, -sine), (sine, cosine)))
            predicted = origin + rotation @ relative + correction[:2]
            residual = measured_local - predicted
            h = np.array(
                (
                    (1.0, 0.0, -sine * relative[0] - cosine * relative[1]),
                    (0.0, 1.0, cosine * relative[0] - sine * relative[1]),
                )
            )
            minimum_variance = config.gps.min_position_std_m**2
            covariance_xx = max(
                measurement.covariance_xx_m2, minimum_variance
            )
            covariance_yy = max(
                measurement.covariance_yy_m2, minimum_variance
            )
            measurement_covariance = np.array(
                (
                    (covariance_xx, measurement.covariance_xy_m2),
                    (measurement.covariance_xy_m2, covariance_yy),
                )
            )
            measurement_covariance = row_rotation.T @ measurement_covariance @ row_rotation
            innovation = h @ covariance @ h.T + measurement_covariance
            raw_nis = float(residual.T @ np.linalg.solve(innovation, residual))
            scale = max(1.0, raw_nis / config.gps.position_nis_threshold)
            accepted = scale <= config.gps.max_covariance_scale
            if accepted:
                scaled_measurement = measurement_covariance * scale
                innovation = h @ covariance @ h.T + scaled_measurement
                gain = np.linalg.solve(innovation.T, (covariance @ h.T).T).T
                correction += gain @ residual
                correction[2] = _wrap(float(correction[2]))
                identity_minus_kh = np.eye(3) - gain @ h
                covariance = (
                    identity_minus_kh @ covariance @ identity_minus_kh.T
                    + gain @ scaled_measurement @ gain.T
                )
                covariance = 0.5 * (covariance + covariance.T)
                updates += 1
            else:
                rejected += 1
            diagnostics.append(
                (measurement.timestamp_ns, raw_nis, scale, accepted, *correction)
            )
            last_update_ns = measurement.timestamp_ns
            next_gps += 1

        relative = np.array((state.x_m, state.y_m)) - origin
        cosine, sine = math.cos(correction[2]), math.sin(correction[2])
        rotation = np.array(((cosine, -sine), (sine, cosine)))
        corrected_position = origin + rotation @ relative + correction[:2]
        corrected_states.append(
            Estimate(
                state.timestamp_ns,
                float(corrected_position[0]),
                float(corrected_position[1]),
                _wrap(state.yaw_rad + float(correction[2])),
                state.speed_m_s,
            )
        )

    path = config.general.output / "diagnostics_gps_gps_sparse.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "timestamp_ns",
                "position_nis",
                "covariance_scale",
                "accepted",
                "correction_x_m",
                "correction_y_m",
                "correction_yaw_rad",
            )
        )
        writer.writerows(diagnostics)
    return GpsTrajectoryFusionResult(
        corrected_states,
        updates,
        rejected,
        len(gps) - alignment_index,
    )

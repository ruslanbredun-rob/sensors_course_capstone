"""Simplified sliding-window visual-inertial odometry.

Stereo VO supplies metric relative-motion factors. IMU samples are preintegrated
between camera epochs. A small robust least-squares window jointly estimates
node speeds, per-edge yaw increments, gyro bias and forward-acceleration bias.
This is a motion-factor VIO prototype, not feature-level bundle adjustment.
"""

from __future__ import annotations

import csv
import math
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from src.camera.visual_odometry import VisualOdometryResult
from src.common.config import RunConfig
from src.common.models import ImuSample, RelativeMotion, RelativePoseEpoch


@dataclass(frozen=True)
class ImuPreintegration:
    dt_s: float
    mean_gyro_rad_s: float
    mean_accel_m_s2: float


@dataclass(frozen=True)
class VioOdometryResult:
    motions: list[RelativeMotion]
    epochs: list[RelativePoseEpoch]
    attempted_pairs: int
    rejected_pairs: int


@dataclass(frozen=True)
class _WindowEdge:
    motion: RelativeMotion
    imu: ImuPreintegration
    start_speed_prior_m_s: float


def preintegrate_imu(
    samples: list[ImuSample],
    timestamps: list[int],
    start_timestamp_ns: int,
    end_timestamp_ns: int,
) -> ImuPreintegration:
    """Integrate piecewise-constant IMU readings over one camera interval."""
    if end_timestamp_ns <= start_timestamp_ns:
        raise ValueError("IMU preintegration interval must be positive")
    if not samples or len(samples) != len(timestamps):
        raise ValueError("IMU samples and timestamps must be non-empty and aligned")
    if start_timestamp_ns < timestamps[0] or end_timestamp_ns > timestamps[-1]:
        raise ValueError("Camera interval lies outside the IMU stream")

    index = max(0, bisect_right(timestamps, start_timestamp_ns) - 1)
    cursor_ns = start_timestamp_ns
    gyro_integral = 0.0
    accel_integral = 0.0
    while cursor_ns < end_timestamp_ns:
        next_ns = end_timestamp_ns
        if index + 1 < len(samples):
            next_ns = min(next_ns, timestamps[index + 1])
        dt_s = (next_ns - cursor_ns) * 1e-9
        gyro_integral += samples[index].yaw_rate_rad_s * dt_s
        accel_integral += samples[index].forward_accel_m_s2 * dt_s
        cursor_ns = next_ns
        if index + 1 < len(samples) and cursor_ns >= timestamps[index + 1]:
            index += 1
    interval_s = (end_timestamp_ns - start_timestamp_ns) * 1e-9
    return ImuPreintegration(
        dt_s=interval_s,
        mean_gyro_rad_s=gyro_integral / interval_s,
        mean_accel_m_s2=accel_integral / interval_s,
    )


class SlidingWindowVio:
    """Robust local optimizer over visual motion and preintegrated IMU factors."""

    def __init__(self, config: RunConfig) -> None:
        self.config = config
        self.edges: deque[_WindowEdge] = deque(maxlen=config.vio.window_size)
        self.gyro_bias_rad_s = 0.0
        self.accel_bias_m_s2 = 0.0
        self.last_speed_m_s: float | None = None
        self.last_timestamp_ns: int | None = None

    def add(
        self, visual: RelativeMotion, imu: ImuPreintegration
    ) -> RelativeMotion:
        start_timestamp_ns = visual.timestamp_ns - round(visual.dt_s * 1e9)
        if (
            self.last_timestamp_ns is not None
            and abs(start_timestamp_ns - self.last_timestamp_ns) > 5_000_000
        ):
            self.edges.clear()
            self.last_speed_m_s = None
        visual_speed = visual.dx_m / visual.dt_s
        start_speed = (
            self.last_speed_m_s
            if self.last_speed_m_s is not None
            else visual_speed
        )
        self.edges.append(_WindowEdge(visual, imu, start_speed))
        optimized, residual_rms = self._optimize()
        count = len(self.edges)
        velocities = optimized[: count + 1]
        yaw_increments = optimized[count + 1 : count + 1 + count]
        self.gyro_bias_rad_s = float(optimized[-2])
        self.accel_bias_m_s2 = float(optimized[-1])
        self.last_speed_m_s = float(velocities[-1])
        self.last_timestamp_ns = visual.timestamp_ns

        fused_distance = 0.5 * (velocities[-2] + velocities[-1]) * visual.dt_s
        if abs(visual.dx_m) > 0.05:
            raw_scale = fused_distance / visual.dx_m
            limit = self.config.vio.max_scale_correction
            scale = max(1.0 / limit, min(limit, raw_scale))
        else:
            scale = 1.0
        quality = visual.quality * math.exp(-0.1 * min(residual_rms, 20.0))
        visual_yaw_std = max(
            visual.yaw_std_rad, self.config.vio.visual_yaw_std_rad
        )
        imu_yaw_std = self.config.vio.imu_gyro_std_rad_s * imu.dt_s
        yaw_std = math.sqrt(
            1.0 / (1.0 / visual_yaw_std**2 + 1.0 / imu_yaw_std**2)
        )
        return RelativeMotion(
            timestamp_ns=visual.timestamp_ns,
            dt_s=visual.dt_s,
            dx_m=float(visual.dx_m * scale),
            dy_m=float(visual.dy_m * scale),
            dyaw_rad=float(yaw_increments[-1]),
            source="vio",
            translation_std_m=max(
                visual.translation_std_m,
                self.config.vio.visual_speed_std_m_s * visual.dt_s,
            ),
            yaw_std_rad=max(0.003, yaw_std),
            quality=max(0.05, min(1.0, quality)),
        )

    def _optimize(self) -> tuple[np.ndarray, float]:
        edges = list(self.edges)
        count = len(edges)
        visual_speeds = np.array(
            [edge.motion.dx_m / edge.motion.dt_s for edge in edges]
        )
        velocities = np.empty(count + 1)
        velocities[0] = edges[0].start_speed_prior_m_s
        velocities[1:] = visual_speeds
        yaw = np.array([edge.motion.dyaw_rad for edge in edges])
        initial = np.concatenate(
            (velocities, yaw, (self.gyro_bias_rad_s, self.accel_bias_m_s2))
        )
        bias_prior = initial[-2:].copy()
        vio = self.config.vio

        def residuals(values: np.ndarray) -> np.ndarray:
            speed_values = values[: count + 1]
            yaw_values = values[count + 1 : count + 1 + count]
            gyro_bias, accel_bias = values[-2:]
            result = [
                (speed_values[0] - edges[0].start_speed_prior_m_s)
                / vio.visual_speed_std_m_s,
                (gyro_bias - bias_prior[0]) / vio.gyro_bias_prior_std_rad_s,
                (accel_bias - bias_prior[1]) / vio.accel_bias_prior_std_m_s2,
            ]
            for index, edge in enumerate(edges):
                dt_s = edge.imu.dt_s
                quality = max(0.1, edge.motion.quality)
                visual_speed = edge.motion.dx_m / edge.motion.dt_s
                result.extend(
                    (
                        (
                            0.5 * (speed_values[index] + speed_values[index + 1])
                            - visual_speed
                        )
                        / (vio.visual_speed_std_m_s / quality),
                        (
                            (speed_values[index + 1] - speed_values[index]) / dt_s
                            - (edge.imu.mean_accel_m_s2 - accel_bias)
                        )
                        / vio.imu_accel_std_m_s2,
                        math.atan2(
                            math.sin(yaw_values[index] - edge.motion.dyaw_rad),
                            math.cos(yaw_values[index] - edge.motion.dyaw_rad),
                        )
                        / (vio.visual_yaw_std_rad / quality),
                        (
                            yaw_values[index] / dt_s
                            - (edge.imu.mean_gyro_rad_s - gyro_bias)
                        )
                        / vio.imu_gyro_std_rad_s,
                    )
                )
            return np.asarray(result)

        lower = np.concatenate(
            (np.full(count + 1, -10.0), np.full(count, -0.6), (-0.5, -5.0))
        )
        upper = np.concatenate(
            (np.full(count + 1, 50.0), np.full(count, 0.6), (0.5, 5.0))
        )
        solution = least_squares(
            residuals,
            initial,
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=1.0,
            max_nfev=40,
        )
        residual_rms = float(np.sqrt(np.mean(residuals(solution.x) ** 2)))
        return solution.x, residual_rms


def _write_motions(path: Path, motions: list[RelativeMotion]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "timestamp_ns",
                "dt_s",
                "dx_m",
                "dy_m",
                "dyaw_rad",
                "translation_std_m",
                "yaw_std_rad",
                "quality",
            )
        )
        for motion in motions:
            writer.writerow(
                (
                    motion.timestamp_ns,
                    f"{motion.dt_s:.6f}",
                    f"{motion.dx_m:.6f}",
                    f"{motion.dy_m:.6f}",
                    f"{motion.dyaw_rad:.8f}",
                    f"{motion.translation_std_m:.6f}",
                    f"{motion.yaw_std_rad:.8f}",
                    f"{motion.quality:.6f}",
                )
            )


def compute_vio_odometry(
    config: RunConfig,
    imu: list[ImuSample],
    visual: VisualOdometryResult,
) -> VioOdometryResult:
    """Fuse accepted stereo factors with IMU preintegration and cache them."""
    imu_timestamps = [sample.timestamp_ns for sample in imu]
    estimator = SlidingWindowVio(config)
    motions: list[RelativeMotion] = []
    rejected = visual.rejected_pairs
    for motion in visual.motions:
        start_ns = motion.timestamp_ns - round(motion.dt_s * 1e9)
        try:
            preintegrated = preintegrate_imu(
                imu, imu_timestamps, start_ns, motion.timestamp_ns
            )
            motions.append(estimator.add(motion, preintegrated))
        except (ValueError, np.linalg.LinAlgError):
            rejected += 1
    _write_motions(config.general.output / "vio_odometry.csv", motions)
    epochs = [RelativePoseEpoch(epoch.timestamp_ns, "vio") for epoch in visual.epochs]
    return VioOdometryResult(
        motions=motions,
        epochs=epochs,
        attempted_pairs=visual.attempted_pairs,
        rejected_pairs=rejected,
    )


def read_vio_odometry(
    config: RunConfig, visual: VisualOdometryResult
) -> VioOdometryResult:
    """Read cached VIO factors and recover camera epochs."""
    motions: list[RelativeMotion] = []
    path = config.general.output / "vio_odometry.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            motions.append(
                RelativeMotion(
                    int(row["timestamp_ns"]),
                    float(row["dt_s"]),
                    float(row["dx_m"]),
                    float(row["dy_m"]),
                    float(row["dyaw_rad"]),
                    "vio",
                    float(row["translation_std_m"]),
                    float(row["yaw_std_rad"]),
                    float(row["quality"]),
                )
            )
    epochs = [RelativePoseEpoch(epoch.timestamp_ns, "vio") for epoch in visual.epochs]
    return VioOdometryResult(
        motions=motions,
        epochs=epochs,
        attempted_pairs=visual.attempted_pairs,
        rejected_pairs=max(0, visual.attempted_pairs - len(motions)),
    )

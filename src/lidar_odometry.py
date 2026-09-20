"""Planar LiDAR odometry from the calibrated left VLP-16 point clouds."""

from __future__ import annotations

import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .calibration import read_rigid_transform
from .config import RunConfig
from .models import RelativeMotion, WheelMeasurement


@dataclass(frozen=True)
class LidarOdometryResult:
    motions: list[RelativeMotion]
    attempted_pairs: int
    rejected_pairs: int


@dataclass(frozen=True)
class IcpResult:
    translation_m: np.ndarray
    yaw_rad: float
    rmse_m: float
    inlier_ratio: float
    iterations: int


def rigid_fit_2d(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fit target = source @ rotation.T + translation without scale."""
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError("source and target must be same-size Nx2 arrays")
    if len(source) < 3:
        raise ValueError("At least three correspondences are required")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    u, _, vt = np.linalg.svd(
        (source - source_center).T @ (target - target_center)
    )
    row_rotation = u @ np.diag([1.0, np.linalg.det(u @ vt)]) @ vt
    rotation = row_rotation.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def icp_2d(
    previous: np.ndarray,
    current: np.ndarray,
    *,
    initial_translation: np.ndarray,
    initial_yaw_rad: float,
    max_correspondence_m: float,
    max_iterations: int = 20,
) -> IcpResult | None:
    """Align current scan into the previous vehicle frame."""
    if len(previous) < 50 or len(current) < 50:
        return None
    tree = cKDTree(previous)
    cosine, sine = math.cos(initial_yaw_rad), math.sin(initial_yaw_rad)
    rotation = np.array(((cosine, -sine), (sine, cosine)))
    translation = initial_translation.astype(float).copy()
    used_iterations = 0
    for iteration in range(max_iterations):
        transformed = current @ rotation.T + translation
        distances, indices = tree.query(transformed, k=1)
        valid = distances < max_correspondence_m
        if int(valid.sum()) < 50:
            return None
        trim_limit = float(np.quantile(distances[valid], 0.75))
        valid &= distances <= trim_limit
        new_rotation, new_translation = rigid_fit_2d(
            current[valid], previous[indices[valid]]
        )
        delta_translation = float(np.linalg.norm(new_translation - translation))
        delta_rotation = new_rotation @ rotation.T
        delta_yaw = abs(math.atan2(delta_rotation[1, 0], delta_rotation[0, 0]))
        rotation, translation = new_rotation, new_translation
        used_iterations = iteration + 1
        if delta_translation < 1e-3 and delta_yaw < 1e-5:
            break

    transformed = current @ rotation.T + translation
    distances, _ = tree.query(transformed, k=1)
    inliers = distances < max_correspondence_m
    if int(inliers.sum()) < 50:
        return None
    return IcpResult(
        translation_m=translation,
        yaw_rad=math.atan2(rotation[1, 0], rotation[0, 0]),
        rmse_m=float(np.sqrt(np.mean(distances[inliers] ** 2))),
        inlier_ratio=float(np.mean(inliers)),
        iterations=used_iterations,
    )


class _LidarFrontend:
    def __init__(self, config: RunConfig) -> None:
        self.config = config
        self.rotation, self.translation = read_rigid_transform(
            config.dataset / "calibration" / "Vehicle2LeftVLP.txt"
        )

    def read_scan(self, path: Path) -> np.ndarray:
        values = np.fromfile(path, dtype=np.float32)
        if values.size == 0 or values.size % 4:
            raise ValueError(f"{path}: expected float32 [x, y, z, intensity] records")
        sensor_xyz = values.reshape(-1, 4)[:, :3]
        vehicle_xyz = sensor_xyz @ self.rotation.T + self.translation
        radius = np.linalg.norm(vehicle_xyz[:, :2], axis=1)
        keep = (
            np.all(np.isfinite(vehicle_xyz), axis=1)
            & (radius > 2.0)
            & (radius < 50.0)
            & (vehicle_xyz[:, 2] > -0.8)
            & (vehicle_xyz[:, 2] < 2.5)
        )
        xy = vehicle_xyz[keep, :2]
        cells = np.floor(xy / self.config.lidar_voxel_m).astype(np.int64)
        _, indices = np.unique(cells, axis=0, return_index=True)
        return xy[np.sort(indices)]


def _timestamped_scans(directory: Path) -> list[tuple[int, Path]]:
    scans: list[tuple[int, Path]] = []
    for path in directory.glob("*.bin"):
        try:
            scans.append((int(path.stem), path))
        except ValueError as exc:
            raise ValueError(f"{path}: filename must be a nanosecond timestamp") from exc
    scans.sort()
    return scans


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


def compute_lidar_odometry(
    config: RunConfig, wheels: list[WheelMeasurement] | None = None
) -> LidarOdometryResult:
    """Run scan-to-scan ICP and persist accepted relative motions."""
    scans = _timestamped_scans(
        config.dataset / "sensor_data" / "VLP_left"
    )[:: config.lidar_frame_step]
    if len(scans) < 2:
        raise FileNotFoundError("At least two left VLP-16 scans are required")
    frontend = _LidarFrontend(config)
    previous_timestamp, previous_path = scans[0]
    previous_points = frontend.read_scan(previous_path)
    prior_speed = config.lidar_initial_speed_m_s
    prior_yaw_rate = 0.0
    wheel_hints = wheels or []
    wheel_timestamps = [sample.timestamp_ns for sample in wheel_hints]
    motions: list[RelativeMotion] = []
    rejected = 0
    for timestamp, path in scans[1:]:
        dt_s = (timestamp - previous_timestamp) * 1e-9
        if dt_s <= 0.0:
            raise ValueError("LiDAR timestamps are not strictly increasing")
        current_points = frontend.read_scan(path)
        if wheel_hints:
            position = bisect_left(wheel_timestamps, timestamp)
            candidates = [
                index
                for index in (position - 1, position)
                if 0 <= index < len(wheel_hints)
            ]
            if candidates:
                nearest = min(
                    candidates,
                    key=lambda index: abs(wheel_timestamps[index] - timestamp),
                )
                hint = wheel_hints[nearest]
                prior_speed = hint.speed_m_s
                prior_yaw_rate = hint.yaw_rate_rad_s
        initial_translation = np.array((prior_speed * dt_s, 0.0))
        initial_yaw = prior_yaw_rate * dt_s
        result = icp_2d(
            previous_points,
            current_points,
            initial_translation=initial_translation,
            initial_yaw_rad=initial_yaw,
            max_correspondence_m=config.lidar_max_correspondence_m,
        )
        accepted = result is not None
        if result is not None:
            speed = float(result.translation_m[0] / dt_s)
            quality = min(
                1.0,
                result.inlier_ratio
                * math.exp(-result.rmse_m / config.lidar_max_rmse_m),
            )
            accepted = (
                result.rmse_m <= config.lidar_max_rmse_m
                and result.inlier_ratio >= config.lidar_min_inlier_ratio
                and -5.0 <= speed <= 45.0
                and abs(result.translation_m[1] / dt_s) <= 12.0
                and abs(result.yaw_rad / dt_s) <= 1.5
                and abs(result.translation_m[0] - initial_translation[0])
                <= max(1.0, 0.5 * abs(initial_translation[0]))
                and abs(result.yaw_rad - initial_yaw) <= 0.15
                and quality >= config.lidar_min_quality
            )
        if accepted and result is not None:
            motion = RelativeMotion(
                timestamp_ns=timestamp,
                dt_s=dt_s,
                dx_m=float(result.translation_m[0]),
                dy_m=float(result.translation_m[1]),
                dyaw_rad=result.yaw_rad,
                source="lidar",
                # Scan-to-scan translation is useful, while VLP-16 rolling
                # acquisition makes the planar yaw increment visibly biased
                # on this fast sequence. Keep yaw as a weak consistency cue.
                translation_std_m=max(0.75, 2.0 * result.rmse_m),
                yaw_std_rad=0.3,
                quality=quality,
            )
            motions.append(motion)
            if not wheel_hints:
                prior_speed = speed
                prior_yaw_rate = result.yaw_rad / dt_s
        else:
            rejected += 1
        previous_timestamp = timestamp
        previous_points = current_points
    _write_motions(config.output / "lidar_odometry.csv", motions)
    return LidarOdometryResult(motions, len(scans) - 1, rejected)

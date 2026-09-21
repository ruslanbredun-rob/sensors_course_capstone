"""Planar LiDAR odometry from the calibrated left VLP-16 point clouds."""

from __future__ import annotations

import csv
import math
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from src.common.config import RunConfig
from src.common.models import (
    ImuSample,
    RelativeMotion,
    RelativePoseEpoch,
    WheelMeasurement,
)
from src.dataset.calibration import read_rigid_transform


@dataclass(frozen=True)
class LidarOdometryResult:
    motions: list[RelativeMotion]
    epochs: list[RelativePoseEpoch]
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
    target: np.ndarray,
    current: np.ndarray,
    *,
    initial_translation: np.ndarray,
    initial_yaw_rad: float,
    max_correspondence_m: float,
    max_iterations: int = 20,
    yaw_correction_weight: float = 1.0,
) -> IcpResult | None:
    """Align current points into the target frame."""
    if len(target) < 50 or len(current) < 50:
        return None
    tree = cKDTree(target)
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
            current[valid], target[indices[valid]]
        )
        delta_translation = float(np.linalg.norm(new_translation - translation))
        delta_rotation = new_rotation @ rotation.T
        delta_yaw = abs(math.atan2(delta_rotation[1, 0], delta_rotation[0, 0]))
        rotation, translation = new_rotation, new_translation
        used_iterations = iteration + 1
        if delta_translation < 1e-3 and delta_yaw < 1e-5:
            break

    fitted_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    yaw_delta = math.atan2(
        math.sin(fitted_yaw - initial_yaw_rad),
        math.cos(fitted_yaw - initial_yaw_rad),
    )
    regularized_yaw = initial_yaw_rad + yaw_correction_weight * yaw_delta
    rotation = _rotation_2d(regularized_yaw)
    # With yaw regularized by the IMU prior, refit translation for the last
    # robust correspondence set instead of keeping the unconstrained ICP value.
    fitted_points = current @ rotation.T
    provisional = fitted_points + translation
    distances, indices = tree.query(provisional, k=1)
    valid = distances < max_correspondence_m
    if int(valid.sum()) >= 50:
        trim_limit = float(np.quantile(distances[valid], 0.75))
        valid &= distances <= trim_limit
        translation = np.mean(target[indices[valid]] - fitted_points[valid], axis=0)

    transformed = current @ rotation.T + translation
    distances, _ = tree.query(transformed, k=1)
    inliers = distances < max_correspondence_m
    if int(inliers.sum()) < 50:
        return None
    return IcpResult(
        translation_m=translation,
        yaw_rad=regularized_yaw,
        rmse_m=float(np.sqrt(np.mean(distances[inliers] ** 2))),
        inlier_ratio=float(np.mean(inliers)),
        iterations=used_iterations,
    )


class _LidarFrontend:
    def __init__(self, config: RunConfig) -> None:
        self.config = config
        self.rotation, self.translation = read_rigid_transform(
            config.general.dataset / "calibration" / "Vehicle2LeftVLP.txt"
        )

    def read_scan(
        self,
        path: Path,
        *,
        speed_m_s: float,
        yaw_rate_rad_s: float,
    ) -> np.ndarray:
        values = np.fromfile(path, dtype=np.float32)
        if values.size == 0 or values.size % 4:
            raise ValueError(f"{path}: expected float32 [x, y, z, intensity] records")
        sensor_xyz = values.reshape(-1, 4)[:, :3]
        vehicle_xyz = sensor_xyz @ self.rotation.T + self.translation
        vehicle_xyz[:, :2] = deskew_points_2d(
            vehicle_xyz[:, :2],
            speed_m_s=speed_m_s,
            yaw_rate_rad_s=yaw_rate_rad_s,
            scan_period_s=self.config.lidar_odometry.deskew_scan_period_s,
        )
        radius = np.linalg.norm(vehicle_xyz[:, :2], axis=1)
        keep = (
            np.all(np.isfinite(vehicle_xyz), axis=1)
            & (radius > 2.0)
            & (radius < 50.0)
            & (vehicle_xyz[:, 2] > -0.8)
            & (vehicle_xyz[:, 2] < 2.5)
        )
        xy = vehicle_xyz[keep, :2]
        return _voxel_downsample(xy, self.config.lidar_odometry.voxel_m)


def deskew_points_2d(
    points: np.ndarray,
    *,
    speed_m_s: float,
    yaw_rate_rad_s: float,
    scan_period_s: float,
) -> np.ndarray:
    """Move ordered VLP-16 points to the scan-end vehicle frame."""
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must be an Nx2 array")
    if scan_period_s <= 0.0 or not len(points):
        return points.copy()
    relative_time = (np.arange(len(points), dtype=float) + 1.0) / len(points) - 1.0
    relative_time *= scan_period_s
    yaw = yaw_rate_rad_s * relative_time
    cosine, sine = np.cos(yaw), np.sin(yaw)
    if abs(yaw_rate_rad_s) > 1e-8:
        translation_x = speed_m_s / yaw_rate_rad_s * np.sin(yaw)
        translation_y = speed_m_s / yaw_rate_rad_s * (1.0 - np.cos(yaw))
    else:
        translation_x = speed_m_s * relative_time
        translation_y = np.zeros_like(relative_time)
    deskewed = np.empty_like(points, dtype=float)
    deskewed[:, 0] = (
        cosine * points[:, 0] - sine * points[:, 1] + translation_x
    )
    deskewed[:, 1] = (
        sine * points[:, 0] + cosine * points[:, 1] + translation_y
    )
    return deskewed


def _voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if not len(points):
        return points
    cells = np.floor(points / voxel_m).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(indices)]


def _rotation_2d(yaw_rad: float) -> np.ndarray:
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array(((cosine, -sine), (sine, cosine)))


def _nearest_sample(samples: list, timestamps: list[int], timestamp_ns: int):
    if not samples:
        return None
    position = bisect_left(timestamps, timestamp_ns)
    candidates = [
        index for index in (position - 1, position) if 0 <= index < len(samples)
    ]
    return samples[
        min(candidates, key=lambda index: abs(timestamps[index] - timestamp_ns))
    ]


def _motion_hint(
    timestamp_ns: int,
    wheels: list[WheelMeasurement],
    wheel_timestamps: list[int],
    imu: list[ImuSample],
    imu_timestamps: list[int],
    default_speed_m_s: float,
) -> tuple[float, float]:
    wheel = _nearest_sample(wheels, wheel_timestamps, timestamp_ns)
    imu_sample = _nearest_sample(imu, imu_timestamps, timestamp_ns)
    speed = wheel.speed_m_s if wheel is not None else default_speed_m_s
    yaw_rate = imu_sample.yaw_rate_rad_s if imu_sample is not None else 0.0
    return speed, yaw_rate


def _local_map(scans: deque[np.ndarray], voxel_m: float) -> np.ndarray:
    return _voxel_downsample(np.concatenate(tuple(scans)), voxel_m)


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


def read_lidar_odometry(config: RunConfig) -> LidarOdometryResult:
    """Load cached accepted increments and recover all scan epochs."""
    path = config.general.output / "lidar_odometry.csv"
    motions: list[RelativeMotion] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            motions.append(
                RelativeMotion(
                    int(row["timestamp_ns"]),
                    float(row["dt_s"]),
                    float(row["dx_m"]),
                    float(row["dy_m"]),
                    float(row["dyaw_rad"]),
                    "lidar",
                    float(row["translation_std_m"]),
                    float(row["yaw_std_rad"]),
                    float(row["quality"]),
                )
            )
    scans = _timestamped_scans(
        config.general.dataset / "sensor_data" / "VLP_left"
    )[:: config.lidar_odometry.frame_step]
    epochs = [RelativePoseEpoch(timestamp, "lidar") for timestamp, _ in scans]
    attempted = max(0, len(scans) - 1)
    return LidarOdometryResult(motions, epochs, attempted, attempted - len(motions))


def compute_lidar_odometry(
    config: RunConfig,
    wheels: list[WheelMeasurement] | None = None,
    imu: list[ImuSample] | None = None,
) -> LidarOdometryResult:
    """Run IMU-deskewed scan-to-local-map ICP and persist pose increments."""
    scans = _timestamped_scans(
        config.general.dataset / "sensor_data" / "VLP_left"
    )[:: config.lidar_odometry.frame_step]
    if len(scans) < 2:
        raise FileNotFoundError("At least two left VLP-16 scans are required")
    frontend = _LidarFrontend(config)
    wheel_hints = wheels or []
    imu_hints = imu or []
    wheel_timestamps = [sample.timestamp_ns for sample in wheel_hints]
    imu_timestamps = [sample.timestamp_ns for sample in imu_hints]
    previous_timestamp, previous_path = scans[0]
    prior_speed, prior_yaw_rate = _motion_hint(
        previous_timestamp,
        wheel_hints,
        wheel_timestamps,
        imu_hints,
        imu_timestamps,
        config.lidar_odometry.initial_speed_m_s,
    )
    previous_points = frontend.read_scan(
        previous_path,
        speed_m_s=prior_speed,
        yaw_rate_rad_s=prior_yaw_rate,
    )
    previous_rotation = np.eye(2)
    previous_translation = np.zeros(2)
    map_scans: deque[np.ndarray] = deque(
        (previous_points.copy(),),
        maxlen=config.lidar_odometry.local_map_scans,
    )
    motions: list[RelativeMotion] = []
    rejected = 0
    for timestamp, path in scans[1:]:
        dt_s = (timestamp - previous_timestamp) * 1e-9
        if dt_s <= 0.0:
            raise ValueError("LiDAR timestamps are not strictly increasing")
        prior_speed, prior_yaw_rate = _motion_hint(
            timestamp,
            wheel_hints,
            wheel_timestamps,
            imu_hints,
            imu_timestamps,
            prior_speed,
        )
        current_points = frontend.read_scan(
            path,
            speed_m_s=prior_speed,
            yaw_rate_rad_s=prior_yaw_rate,
        )
        initial_relative_translation = np.array((prior_speed * dt_s, 0.0))
        initial_relative_yaw = prior_yaw_rate * dt_s
        initial_relative_rotation = _rotation_2d(initial_relative_yaw)
        initial_rotation = previous_rotation @ initial_relative_rotation
        initial_translation = (
            previous_translation
            + previous_rotation @ initial_relative_translation
        )
        target_map = _local_map(map_scans, config.lidar_odometry.map_voxel_m)
        map_result = icp_2d(
            target_map,
            current_points,
            initial_translation=initial_translation,
            initial_yaw_rad=math.atan2(initial_rotation[1, 0], initial_rotation[0, 0]),
            max_correspondence_m=config.lidar_odometry.max_correspondence_m,
            max_iterations=max(4, config.lidar_odometry.max_iterations // 2),
            yaw_correction_weight=config.lidar_odometry.map_yaw_correction_weight,
        )
        scan_result = icp_2d(
            previous_points,
            current_points,
            initial_translation=initial_relative_translation,
            initial_yaw_rad=initial_relative_yaw,
            max_correspondence_m=config.lidar_odometry.max_correspondence_m,
            max_iterations=config.lidar_odometry.max_iterations,
        )

        if map_result is not None:
            current_rotation = _rotation_2d(map_result.yaw_rad)
            current_translation = map_result.translation_m
            map_relative_rotation = previous_rotation.T @ current_rotation
            map_relative_translation = previous_rotation.T @ (
                current_translation - previous_translation
            )
            map_relative_yaw = math.atan2(
                map_relative_rotation[1, 0], map_relative_rotation[0, 0]
            )
        else:
            current_rotation = initial_rotation
            current_translation = initial_translation
            map_relative_translation = initial_relative_translation
            map_relative_yaw = initial_relative_yaw

        accepted = scan_result is not None
        if scan_result is not None:
            map_weight = (
                config.lidar_odometry.map_measurement_weight
                if map_result is not None
                else 0.0
            )
            relative_translation = (
                (1.0 - map_weight) * scan_result.translation_m
                + map_weight * map_relative_translation
            )
            yaw_difference = math.atan2(
                math.sin(map_relative_yaw - scan_result.yaw_rad),
                math.cos(map_relative_yaw - scan_result.yaw_rad),
            )
            relative_yaw = scan_result.yaw_rad + map_weight * yaw_difference
            speed = float(relative_translation[0] / dt_s)
            quality = min(
                1.0,
                scan_result.inlier_ratio
                * math.exp(-scan_result.rmse_m / config.lidar_odometry.max_rmse_m),
            )
            accepted = (
                scan_result.rmse_m <= config.lidar_odometry.max_rmse_m
                and scan_result.inlier_ratio
                >= config.lidar_odometry.min_inlier_ratio
                and -5.0 <= speed <= 45.0
                and abs(relative_translation[1] / dt_s) <= 12.0
                and abs(relative_yaw / dt_s) <= 1.5
                and abs(relative_translation[0] - initial_relative_translation[0])
                <= max(1.0, 0.5 * abs(initial_relative_translation[0]))
                and abs(relative_yaw - initial_relative_yaw) <= 0.15
                and quality >= config.lidar_odometry.min_quality
            )
        if accepted and scan_result is not None:
            shared_map_scale = 1.0 + 9.0 * config.lidar_odometry.map_measurement_weight
            motion_scale = max(1.0, abs(speed) / 4.0)
            uncertainty_scale = shared_map_scale * motion_scale
            motion = RelativeMotion(
                timestamp_ns=timestamp,
                dt_s=dt_s,
                dx_m=float(relative_translation[0]),
                dy_m=float(relative_translation[1]),
                dyaw_rad=relative_yaw,
                source="lidar",
                # The map term shares geometry across consecutive 10 Hz edges;
                # scale covariance by its weight and by motion during a scan.
                translation_std_m=max(0.08, 0.5 * scan_result.rmse_m)
                * uncertainty_scale,
                yaw_std_rad=max(0.003, 0.015 * (1.0 - quality))
                * uncertainty_scale,
                quality=quality,
            )
            motions.append(motion)
        else:
            rejected += 1
        # Keep the local map moving with either scan-to-map or the wheel/IMU
        # prior so one weak scan cannot make the map stale.
        map_scans.append(current_points @ current_rotation.T + current_translation)
        previous_timestamp = timestamp
        previous_points = current_points
        previous_rotation = current_rotation
        previous_translation = current_translation
    _write_motions(config.general.output / "lidar_odometry.csv", motions)
    epochs = [RelativePoseEpoch(timestamp, "lidar") for timestamp, _ in scans]
    return LidarOdometryResult(motions, epochs, len(scans) - 1, rejected)

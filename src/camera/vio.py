"""Feature-level planar stereo visual-inertial odometry."""

from __future__ import annotations

import csv
import json
import math
from bisect import bisect_right
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from src.camera.visual_odometry import (
    _StereoFrame,
    _StereoFrontend,
    _timestamped_files,
)
from src.common.config import RunConfig
from src.common.models import (
    Estimate,
    ImuSample,
    RelativeMotion,
    RelativePoseEpoch,
    WheelMeasurement,
)


@dataclass(frozen=True)
class ImuPreintegration:
    dt_s: float
    mean_gyro_rad_s: float
    mean_accel_m_s2: float


@dataclass(frozen=True)
class WheelPreintegration:
    dt_s: float
    mean_speed_m_s: float
    mean_yaw_rate_rad_s: float


@dataclass(frozen=True)
class FeatureFactor:
    points_previous_camera_m: np.ndarray
    points_current_px: np.ndarray
    initial_delta_vehicle_m: np.ndarray
    initial_dyaw_rad: float
    quality: float


@dataclass(frozen=True)
class VioTrajectoryResult:
    states: list[Estimate]
    attempted_pairs: int
    visual_factors: int
    rejected_pairs: int


def relative_measurements_from_vio(
    states: list[Estimate],
    config: RunConfig,
) -> tuple[list[RelativeMotion], list[RelativePoseEpoch]]:
    """Convert the VIO trajectory into body-frame pose factors for the EKF."""
    motions: list[RelativeMotion] = []
    epochs: list[RelativePoseEpoch] = []
    for previous, current in zip(states, states[1:]):
        dt_s = (current.timestamp_ns - previous.timestamp_ns) * 1e-9
        if dt_s <= 0.0:
            raise ValueError("VIO states must be strictly time ordered")
        world_dx = current.x_m - previous.x_m
        world_dy = current.y_m - previous.y_m
        cosine, sine = math.cos(previous.yaw_rad), math.sin(previous.yaw_rad)
        motions.append(
            RelativeMotion(
                timestamp_ns=current.timestamp_ns,
                dt_s=dt_s,
                dx_m=cosine * world_dx + sine * world_dy,
                dy_m=-sine * world_dx + cosine * world_dy,
                dyaw_rad=_wrap(current.yaw_rad - previous.yaw_rad),
                source="vio",
                translation_std_m=config.fusion.vio_translation_std_m,
                yaw_std_rad=config.fusion.vio_yaw_std_rad,
            )
        )
        epochs.append(RelativePoseEpoch(previous.timestamp_ns, "vio"))
    return motions, epochs


@dataclass
class _Node:
    timestamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float


@dataclass(frozen=True)
class _Edge:
    imu: ImuPreintegration
    wheel: WheelPreintegration
    visual: FeatureFactor | None


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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
    gyro_integral = accel_integral = 0.0
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
        interval_s, gyro_integral / interval_s, accel_integral / interval_s
    )


def preintegrate_wheels(
    samples: list[WheelMeasurement],
    timestamps: list[int],
    start_timestamp_ns: int,
    end_timestamp_ns: int,
) -> WheelPreintegration:
    """Average wheel speed and yaw rate over one camera interval."""
    if end_timestamp_ns <= start_timestamp_ns:
        raise ValueError("Wheel preintegration interval must be positive")
    if not samples or len(samples) != len(timestamps):
        raise ValueError("Wheel samples and timestamps must be non-empty and aligned")
    if start_timestamp_ns < timestamps[0] or end_timestamp_ns > timestamps[-1]:
        raise ValueError("Camera interval lies outside the wheel stream")
    inside = [
        index
        for index in range(
            bisect_right(timestamps, start_timestamp_ns),
            bisect_right(timestamps, end_timestamp_ns),
        )
    ]
    integration_times = np.array(
        [start_timestamp_ns, *(timestamps[index] for index in inside), end_timestamp_ns],
        dtype=np.int64,
    )
    source_times = np.asarray(timestamps, dtype=np.int64)
    speed = np.interp(
        integration_times,
        source_times,
        [sample.speed_m_s for sample in samples],
    )
    yaw_rate = np.interp(
        integration_times,
        source_times,
        [sample.yaw_rate_rad_s for sample in samples],
    )
    relative_time_s = (integration_times - start_timestamp_ns) * 1e-9
    interval_s = (end_timestamp_ns - start_timestamp_ns) * 1e-9
    return WheelPreintegration(
        interval_s,
        float(np.trapezoid(speed, relative_time_s) / interval_s),
        float(np.trapezoid(yaw_rate, relative_time_s) / interval_s),
    )


def _feature_factor(
    frontend: _StereoFrontend,
    previous: _StereoFrame,
    current: _StereoFrame,
    config: RunConfig,
) -> FeatureFactor | None:
    if previous.descriptors is None or current.descriptors is None:
        return None
    matches = frontend.matcher.knnMatch(
        previous.descriptors, current.descriptors, k=2
    )
    good = [
        pair[0]
        for pair in matches
        if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance
    ]
    if len(good) < config.vio.min_tracks:
        return None
    previous_px = np.float32([previous.keypoints[m.queryIdx].pt for m in good])
    current_px = np.float32([current.keypoints[m.trainIdx].pt for m in good])
    pixels = np.rint(previous_px).astype(int)
    width, height = frontend.size
    valid = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    pixels, previous_px, current_px = (
        pixels[valid], previous_px[valid], current_px[valid]
    )
    disparity = previous.disparity[pixels[:, 1], pixels[:, 0]]
    valid = np.isfinite(disparity) & (disparity > 1.0)
    disparity, previous_px, current_px = (
        disparity[valid], previous_px[valid], current_px[valid]
    )
    if len(disparity) < config.vio.min_tracks:
        return None
    depth = frontend.focal_px * frontend.baseline_m / disparity
    valid = (depth > 1.0) & (depth < 80.0)
    depth, previous_px, current_px = depth[valid], previous_px[valid], current_px[valid]
    if len(depth) < config.vio.min_tracks:
        return None
    xyz = np.column_stack(
        (
            (previous_px[:, 0] - frontend.k[0, 2]) * depth / frontend.focal_px,
            (previous_px[:, 1] - frontend.k[1, 2]) * depth / frontend.focal_px,
            depth,
        )
    ).astype(np.float64)
    success, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        xyz,
        current_px.astype(np.float64),
        frontend.k,
        None,
        iterationsCount=100,
        reprojectionError=config.vio.reprojection_std_px * 2.0,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not success or inliers is None or len(inliers) < config.vio.min_tracks:
        return None
    indices = inliers.ravel()
    if len(indices) > config.vio.max_tracks:
        indices = indices[np.linspace(0, len(indices) - 1, config.vio.max_tracks).astype(int)]
    xyz, current_px = xyz[indices], current_px[indices]
    camera_rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_delta = -camera_rotation.T @ translation.ravel()
    vehicle_rotation = (
        frontend.rectified_to_vehicle
        @ camera_rotation.T
        @ frontend.rectified_to_vehicle.T
    )
    sensor_delta = frontend.rectified_to_vehicle @ camera_delta
    vehicle_delta = (
        sensor_delta
        + frontend.sensor_origin_in_vehicle
        - vehicle_rotation @ frontend.sensor_origin_in_vehicle
    )
    yaw = math.atan2(vehicle_rotation[1, 0], vehicle_rotation[0, 0])
    return FeatureFactor(
        xyz,
        current_px.astype(float),
        vehicle_delta,
        yaw,
        min(1.0, len(indices) / max(len(good), 1)),
    )


def _visual_consistent_with_wheels(
    visual: FeatureFactor,
    wheel: WheelPreintegration,
    config: RunConfig,
) -> bool:
    wheel_delta = np.array((wheel.mean_speed_m_s * wheel.dt_s, 0.0))
    translation_error = np.linalg.norm(
        visual.initial_delta_vehicle_m[:2] - wheel_delta
    )
    yaw_error = abs(
        _wrap(
            visual.initial_dyaw_rad
            - wheel.mean_yaw_rate_rad_s * wheel.dt_s
        )
    )
    return (
        translation_error <= config.vio.wheel_translation_gate_m
        and yaw_error <= config.vio.wheel_yaw_gate_rad
    )


def _visual_wheel_weight(
    visual: FeatureFactor,
    wheel: WheelPreintegration,
    config: RunConfig,
) -> float:
    wheel_delta = np.array((wheel.mean_speed_m_s * wheel.dt_s, 0.0))
    translation_ratio = (
        np.linalg.norm(visual.initial_delta_vehicle_m[:2] - wheel_delta)
        / config.vio.wheel_translation_gate_m
    )
    yaw_ratio = abs(
        _wrap(
            visual.initial_dyaw_rad
            - wheel.mean_yaw_rate_rad_s * wheel.dt_s
        )
    ) / config.vio.wheel_yaw_gate_rad
    return max(0.05, math.exp(-2.0 * (translation_ratio**2 + yaw_ratio**2)))


class FeatureWindowVio:
    """Jointly optimize planar poses, speeds and IMU biases."""

    def __init__(self, config: RunConfig, frontend: _StereoFrontend) -> None:
        self.config = config
        self.frontend = frontend
        self.nodes: list[_Node] = []
        self.edges: list[_Edge] = []
        self.gyro_bias_rad_s = 0.0
        self.accel_bias_m_s2 = 0.0

    def initialize(self, timestamp_ns: int) -> Estimate:
        node = _Node(timestamp_ns, 0.0, 0.0, 0.0, 0.0)
        self.nodes = [node]
        return Estimate(timestamp_ns, 0.0, 0.0, 0.0, 0.0)

    def add(
        self,
        timestamp_ns: int,
        imu: ImuPreintegration,
        wheel: WheelPreintegration,
        visual: FeatureFactor | None,
    ) -> Estimate:
        previous = self.nodes[-1]
        dx = wheel.mean_speed_m_s * wheel.dt_s
        dy = 0.0
        dyaw = wheel.mean_yaw_rate_rad_s * wheel.dt_s
        speed = wheel.mean_speed_m_s
        if len(self.nodes) == 1 and not self.edges:
            previous.speed_m_s = speed
        cosine, sine = math.cos(previous.yaw_rad), math.sin(previous.yaw_rad)
        self.nodes.append(
            _Node(
                timestamp_ns,
                previous.x_m + cosine * dx - sine * dy,
                previous.y_m + sine * dx + cosine * dy,
                _wrap(previous.yaw_rad + dyaw),
                speed,
            )
        )
        self.edges.append(_Edge(imu, wheel, visual))
        if len(self.nodes) > self.config.vio.window_size:
            self.nodes.pop(0)
            self.edges.pop(0)
        self._optimize()
        current = self.nodes[-1]
        return Estimate(
            current.timestamp_ns,
            current.x_m,
            current.y_m,
            current.yaw_rad,
            current.speed_m_s,
        )

    def _optimize(self) -> None:
        count = len(self.nodes)
        if count < 2:
            return
        state_values = [
            value
            for node in self.nodes
            for value in (node.x_m, node.y_m, node.yaw_rad, node.speed_m_s)
        ]
        initial = np.array(
            state_values + [self.gyro_bias_rad_s, self.accel_bias_m_s2]
        )
        anchor = initial[:4].copy()
        bias_prior = initial[-2:].copy()
        vio = self.config.vio
        r_cv = self.frontend.rectified_to_vehicle
        t_cv = self.frontend.sensor_origin_in_vehicle
        k = self.frontend.k

        def residuals(values: np.ndarray) -> np.ndarray:
            states = values[:-2].reshape(count, 4)
            gyro_bias, accel_bias = values[-2:]
            result = list((states[0] - anchor) / np.array((0.02, 0.02, 0.002, 1.0)))
            result.extend(
                (
                    (gyro_bias - bias_prior[0]) / vio.gyro_bias_prior_std_rad_s,
                    (accel_bias - bias_prior[1]) / vio.accel_bias_prior_std_m_s2,
                )
            )
            for index, edge in enumerate(self.edges):
                first, second = states[index], states[index + 1]
                dt_s = edge.imu.dt_s
                accel = edge.imu.mean_accel_m_s2 - accel_bias
                yaw_delta = _wrap(second[2] - first[2])
                expected_yaw = (edge.imu.mean_gyro_rad_s - gyro_bias) * dt_s
                distance = first[3] * dt_s + 0.5 * accel * dt_s**2
                yaw_mid = first[2] + 0.5 * expected_yaw
                expected_xy = np.array((math.cos(yaw_mid), math.sin(yaw_mid))) * distance
                result.extend(
                    (
                        (second[0] - first[0] - expected_xy[0]) / vio.imu_position_std_m,
                        (second[1] - first[1] - expected_xy[1]) / vio.imu_position_std_m,
                        _wrap(yaw_delta - expected_yaw) / (vio.imu_gyro_std_rad_s * dt_s),
                        (second[3] - first[3] - accel * dt_s) / (vio.imu_accel_std_m_s2 * dt_s),
                    )
                )
                wheel = edge.wheel
                result.extend(
                    (
                        (0.5 * (first[3] + second[3]) - wheel.mean_speed_m_s)
                        / vio.wheel_speed_std_m_s,
                        _wrap(
                            yaw_delta - wheel.mean_yaw_rate_rad_s * wheel.dt_s
                        )
                        / (vio.wheel_yaw_rate_std_rad_s * wheel.dt_s),
                    )
                )
                if edge.visual is None:
                    continue
                factor = edge.visual
                c0, s0 = math.cos(first[2]), math.sin(first[2])
                c1, s1 = math.cos(second[2]), math.sin(second[2])
                rotation0 = np.array(((c0, -s0, 0.0), (s0, c0, 0.0), (0.0, 0.0, 1.0)))
                rotation1_t = np.array(((c1, s1, 0.0), (-s1, c1, 0.0), (0.0, 0.0, 1.0)))
                points_vehicle0 = factor.points_previous_camera_m @ r_cv.T + t_cv
                points_world = points_vehicle0 @ rotation0.T + np.array((first[0], first[1], 0.0))
                position1 = np.array((second[0], second[1], 0.0))
                points_vehicle1 = (points_world - position1) @ rotation1_t.T
                points_camera1 = (points_vehicle1 - t_cv) @ r_cv
                valid_depth = np.maximum(points_camera1[:, 2], 0.5)
                predicted = np.column_stack(
                    (
                        k[0, 0] * points_camera1[:, 0] / valid_depth + k[0, 2],
                        k[1, 1] * points_camera1[:, 1] / valid_depth + k[1, 2],
                    )
                )
                # Tracks from one image pair share calibration, depth and motion
                # errors. Normalize the group so many correlated pixels cannot
                # overwhelm the IMU and wheel factors.
                group_scale = math.sqrt(2.0 * len(factor.points_current_px))
                reprojection = (
                    math.sqrt(vio.visual_factor_weight * factor.quality)
                    * (predicted - factor.points_current_px)
                    / (vio.reprojection_std_px * group_scale)
                )
                result.extend(reprojection.ravel())
            return np.asarray(result)

        lower = np.full_like(initial, -np.inf)
        upper = np.full_like(initial, np.inf)
        for index in range(count):
            lower[4 * index + 3] = -10.0
            upper[4 * index + 3] = 50.0
        lower[-2:] = (-0.5, -5.0)
        upper[-2:] = (0.5, 5.0)
        solution = least_squares(
            residuals,
            initial,
            bounds=(lower, upper),
            loss="huber",
            f_scale=1.0,
            max_nfev=vio.max_iterations,
        )
        states = solution.x[:-2].reshape(count, 4)
        for node, state in zip(self.nodes, states):
            node.x_m, node.y_m, node.yaw_rad, node.speed_m_s = map(float, state)
            node.yaw_rad = _wrap(node.yaw_rad)
        self.gyro_bias_rad_s = float(solution.x[-2])
        self.accel_bias_m_s2 = float(solution.x[-1])


def _write_states(path: Path, states: list[Estimate]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("timestamp_ns", "x_m", "y_m", "yaw_rad", "speed_m_s"))
        for state in states:
            writer.writerow(
                (
                    state.timestamp_ns,
                    state.x_m,
                    state.y_m,
                    state.yaw_rad,
                    state.speed_m_s,
                )
            )


def _cache_signature(config: RunConfig, timestamps: list[int]) -> dict:
    calibration = config.general.dataset / "calibration"
    sources = [
        config.general.dataset / "sensor_data" / "xsens_imu.csv",
        config.general.dataset / "sensor_data" / "encoder.csv",
        calibration / "EncoderParameter.txt",
        calibration / "left.yaml",
        calibration / "right.yaml",
        calibration / "Vehicle2Stereo.txt",
    ]
    return {
        "schema_version": 3,
        "dataset": str(config.general.dataset.resolve()),
        "frame_step": config.visual_odometry.frame_step,
        "image_scale": config.visual_odometry.image_scale,
        "vio": asdict(config.vio),
        "image_count": len(timestamps),
        "first_timestamp_ns": timestamps[0],
        "last_timestamp_ns": timestamps[-1],
        "source_files": {
            str(path.relative_to(config.general.dataset)): {
                "size": path.stat().st_size,
                "mtime_ns": path.stat().st_mtime_ns,
            }
            for path in sources
        },
    }


def _stereo_timestamps(
    config: RunConfig,
) -> tuple[dict[int, Path], dict[int, Path], list[int]]:
    dataset = config.general.dataset
    left = _timestamped_files(dataset / "image" / "stereo_left", "png")
    right = _timestamped_files(dataset / "image" / "stereo_right", "png")
    timestamps = sorted(set(left).intersection(right))[
        :: config.visual_odometry.frame_step
    ]
    if len(timestamps) < 2:
        raise FileNotFoundError("At least two timestamp-matched stereo pairs are required")
    return left, right, timestamps


def read_vio_trajectory(config: RunConfig) -> VioTrajectoryResult:
    _, _, timestamps = _stereo_timestamps(config)
    manifest_path = config.general.output / "vio_manifest.json"
    if not manifest_path.exists():
        raise ValueError("VIO cache has no manifest; run once without --reuse-frontends")
    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("signature") != _cache_signature(config, timestamps):
        raise ValueError("VIO cache does not match the dataset or current configuration")
    states: list[Estimate] = []
    trajectory_path = config.general.output / "vio_trajectory.csv"
    with trajectory_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            states.append(
                Estimate(
                    int(row["timestamp_ns"]),
                    float(row["x_m"]),
                    float(row["y_m"]),
                    float(row["yaw_rad"]),
                    float(row["speed_m_s"]),
                )
            )
    stats = manifest.get("stats", {})
    attempted_pairs = int(stats.get("attempted_pairs", max(0, len(states) - 1)))
    if len(states) != attempted_pairs + 1 or any(
        first.timestamp_ns >= second.timestamp_ns
        for first, second in zip(states, states[1:])
    ):
        raise ValueError("VIO trajectory cache is incomplete or unordered")
    return VioTrajectoryResult(
        states,
        attempted_pairs,
        int(stats.get("visual_factors", 0)),
        int(stats.get("rejected_pairs", 0)),
    )


def compute_vio_trajectory(
    config: RunConfig,
    imu: list[ImuSample],
    wheels: list[WheelMeasurement],
) -> VioTrajectoryResult:
    left, right, all_timestamps = _stereo_timestamps(config)
    if not imu:
        raise ValueError("VIO requires a non-empty IMU stream")
    if not wheels:
        raise ValueError("VIO requires a non-empty wheel stream")
    timestamps = [
        timestamp
        for timestamp in all_timestamps
        if max(imu[0].timestamp_ns, wheels[0].timestamp_ns)
        <= timestamp
        <= min(imu[-1].timestamp_ns, wheels[-1].timestamp_ns)
    ]
    if len(timestamps) < 2:
        raise ValueError("Fewer than two stereo pairs overlap the IMU stream")
    frontend = _StereoFrontend(config)
    estimator = FeatureWindowVio(config, frontend)
    imu_timestamps = [sample.timestamp_ns for sample in imu]
    wheel_timestamps = [sample.timestamp_ns for sample in wheels]
    previous = frontend.read_frame(timestamps[0], left[timestamps[0]], right[timestamps[0]])
    states = [estimator.initialize(previous.timestamp_ns)]
    visual_factors = rejected = 0
    for timestamp in timestamps[1:]:
        current = frontend.read_frame(timestamp, left[timestamp], right[timestamp])
        imu_factor = preintegrate_imu(
            imu, imu_timestamps, previous.timestamp_ns, timestamp
        )
        wheel_factor = preintegrate_wheels(
            wheels, wheel_timestamps, previous.timestamp_ns, timestamp
        )
        try:
            visual_factor = _feature_factor(frontend, previous, current, config)
        except (cv2.error, np.linalg.LinAlgError):
            visual_factor = None
        if visual_factor is not None and not _visual_consistent_with_wheels(
            visual_factor, wheel_factor, config
        ):
            visual_factor = None
        elif visual_factor is not None:
            visual_factor = replace(
                visual_factor,
                quality=visual_factor.quality
                * _visual_wheel_weight(visual_factor, wheel_factor, config),
            )
        visual_factors += int(visual_factor is not None)
        rejected += int(visual_factor is None)
        states.append(
            estimator.add(timestamp, imu_factor, wheel_factor, visual_factor)
        )
        previous = current
    _write_states(config.general.output / "vio_trajectory.csv", states)
    with (config.general.output / "vio_manifest.json").open("w", encoding="utf-8") as stream:
        json.dump(
            {
                "signature": _cache_signature(config, all_timestamps),
                "stats": {
                    "attempted_pairs": len(timestamps) - 1,
                    "visual_factors": visual_factors,
                    "rejected_pairs": rejected,
                },
            },
            stream,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
    return VioTrajectoryResult(states, len(timestamps) - 1, visual_factors, rejected)

"""Quality-gated metric stereo visual odometry for the urban35 sequence."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .calibration import read_rigid_transform
from .config import RunConfig
from .models import RelativeMotion


@dataclass(frozen=True)
class VisualOdometryResult:
    motions: list[RelativeMotion]
    attempted_pairs: int
    rejected_pairs: int


@dataclass(frozen=True)
class _StereoFrame:
    timestamp_ns: int
    keypoints: tuple
    descriptors: np.ndarray | None
    disparity: np.ndarray


def _opencv_matrix(path: Path, key: str) -> np.ndarray:
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError(f"Cannot open OpenCV calibration {path}")
    value = storage.getNode(key).mat()
    storage.release()
    if value is None:
        raise ValueError(f"{path}: missing {key}")
    return value


class _StereoFrontend:
    def __init__(self, config: RunConfig) -> None:
        calibration = config.dataset / "calibration"
        left_path = calibration / "left.yaml"
        right_path = calibration / "right.yaml"
        width = int(1280 * config.visual_image_scale)
        height = int(560 * config.visual_image_scale)
        self.size = (width, height)

        left_k = _opencv_matrix(left_path, "camera_matrix")
        left_d = _opencv_matrix(left_path, "distortion_coefficients")
        left_r = _opencv_matrix(left_path, "rectification_matrix")
        left_p = _opencv_matrix(left_path, "projection_matrix")
        right_k = _opencv_matrix(right_path, "camera_matrix")
        right_d = _opencv_matrix(right_path, "distortion_coefficients")
        right_r = _opencv_matrix(right_path, "rectification_matrix")
        right_p = _opencv_matrix(right_path, "projection_matrix")

        scale = config.visual_image_scale
        self.k = left_p[:, :3].copy()
        self.k[:2] *= scale
        right_new_k = right_p[:, :3].copy()
        right_new_k[:2] *= scale
        self.left_maps = cv2.initUndistortRectifyMap(
            left_k, left_d, left_r, self.k, self.size, cv2.CV_32FC1
        )
        self.right_maps = cv2.initUndistortRectifyMap(
            right_k, right_d, right_r, right_new_k, self.size, cv2.CV_32FC1
        )
        self.focal_px = float(self.k[0, 0])
        self.baseline_m = float(-right_p[0, 3] / right_p[0, 0])
        sensor_to_vehicle, _ = read_rigid_transform(
            calibration / "Vehicle2Stereo.txt"
        )
        # Rectification maps original-camera coordinates into rectified ones.
        self.rectified_to_vehicle = sensor_to_vehicle @ left_r.T
        self.orb = cv2.ORB_create(
            nfeatures=1800, scaleFactor=1.2, nlevels=8, fastThreshold=15
        )
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        disparities = max(64, int(math.ceil(96 * scale / 16)) * 16)
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=disparities,
            blockSize=5,
            P1=8 * 25,
            P2=32 * 25,
            disp12MaxDiff=1,
            uniquenessRatio=8,
            speckleWindowSize=40,
            speckleRange=2,
        )

    def read_frame(self, timestamp_ns: int, left: Path, right: Path) -> _StereoFrame:
        left_image = cv2.imread(str(left), cv2.IMREAD_GRAYSCALE)
        right_image = cv2.imread(str(right), cv2.IMREAD_GRAYSCALE)
        if left_image is None or right_image is None:
            raise ValueError(f"Cannot read stereo pair at {timestamp_ns}")
        left_rectified = cv2.remap(
            left_image, *self.left_maps, interpolation=cv2.INTER_LINEAR
        )
        right_rectified = cv2.remap(
            right_image, *self.right_maps, interpolation=cv2.INTER_LINEAR
        )
        keypoints, descriptors = self.orb.detectAndCompute(left_rectified, None)
        disparity = self.stereo.compute(left_rectified, right_rectified).astype(
            np.float32
        ) / 16.0
        return _StereoFrame(
            timestamp_ns, tuple(keypoints), descriptors, disparity
        )

    def estimate(
        self, previous: _StereoFrame, current: _StereoFrame, config: RunConfig
    ) -> RelativeMotion | None:
        if previous.descriptors is None or current.descriptors is None:
            return None
        matches = self.matcher.knnMatch(
            previous.descriptors, current.descriptors, k=2
        )
        good = [first for first, second in matches if first.distance < 0.75 * second.distance]
        if len(good) < config.visual_min_matches:
            return None
        points_previous = np.float32(
            [previous.keypoints[match.queryIdx].pt for match in good]
        )
        points_current = np.float32(
            [current.keypoints[match.trainIdx].pt for match in good]
        )
        essential, inlier_mask = cv2.findEssentialMat(
            points_previous,
            points_current,
            self.k,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )
        if essential is None or inlier_mask is None:
            return None
        inliers, rotation_21, direction_21, pose_mask = cv2.recoverPose(
            essential,
            points_previous,
            points_current,
            self.k,
            mask=inlier_mask,
        )
        if inliers < config.visual_min_matches // 2:
            return None

        valid = pose_mask.ravel() > 0
        points_previous = points_previous[valid]
        points_current = points_current[valid]
        pixels = np.rint(points_previous).astype(int)
        width, height = self.size
        inside = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < width)
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < height)
        )
        pixels = pixels[inside]
        points_previous = points_previous[inside]
        points_current = points_current[inside]
        disparity = previous.disparity[pixels[:, 1], pixels[:, 0]]
        depth_valid = (disparity > 1.0) & np.isfinite(disparity)
        disparity = disparity[depth_valid]
        points_previous = points_previous[depth_valid]
        points_current = points_current[depth_valid]
        if len(disparity) < 20:
            return None

        depth = self.focal_px * self.baseline_m / disparity
        x = (points_previous[:, 0] - self.k[0, 2]) * depth / self.focal_px
        y = (points_previous[:, 1] - self.k[1, 2]) * depth / self.focal_px
        xyz = np.column_stack((x, y, depth))
        bearings = np.column_stack(
            (
                (points_current[:, 0] - self.k[0, 2]) / self.focal_px,
                (points_current[:, 1] - self.k[1, 2]) / self.focal_px,
                np.ones(len(points_current)),
            )
        )
        bearings /= np.linalg.norm(bearings, axis=1)[:, None]
        rotated = (rotation_21 @ xyz.T).T
        direction = direction_21.ravel()
        projected_direction = direction - (
            np.sum(bearings * direction, axis=1)[:, None] * bearings
        )
        projected_points = rotated - (
            np.sum(bearings * rotated, axis=1)[:, None] * bearings
        )
        denominator = np.sum(projected_direction**2, axis=1)
        scales = -np.sum(projected_direction * projected_points, axis=1) / denominator
        scales = scales[np.isfinite(scales) & (np.abs(scales) < 20.0)]
        if len(scales) < 20:
            return None
        metric_scale = float(np.median(np.abs(scales)))
        scale_mad = float(1.4826 * np.median(np.abs(np.abs(scales) - metric_scale)))

        camera_delta = -rotation_21.T @ (direction * metric_scale)
        vehicle_delta = self.rectified_to_vehicle @ camera_delta
        # urban35 is a forward-driving run. Essential-matrix translation sign
        # becomes ambiguous for far scenes, so use the known motion direction.
        if vehicle_delta[0] < 0.0:
            vehicle_delta *= -1.0
        vehicle_rotation = (
            self.rectified_to_vehicle
            @ rotation_21.T
            @ self.rectified_to_vehicle.T
        )
        yaw = math.atan2(vehicle_rotation[1, 0], vehicle_rotation[0, 0])
        dt_s = (current.timestamp_ns - previous.timestamp_ns) * 1e-9
        if dt_s <= 0.0:
            raise ValueError("Stereo timestamps are not strictly increasing")
        speed = float(vehicle_delta[0] / dt_s)
        if (
            speed <= 0.0
            or speed > config.visual_max_speed_m_s
            or abs(vehicle_delta[1]) > max(1.5, 0.5 * vehicle_delta[0])
            or abs(yaw / dt_s) > 1.5
        ):
            return None
        quality = min(1.0, inliers / max(len(good), 1))
        return RelativeMotion(
            timestamp_ns=current.timestamp_ns,
            dt_s=dt_s,
            dx_m=float(vehicle_delta[0]),
            dy_m=float(vehicle_delta[1]),
            dyaw_rad=float(yaw),
            source="visual",
            translation_std_m=max(0.5, min(3.0, scale_mad)),
            yaw_std_rad=max(0.01, 0.04 * (1.0 - quality)),
            quality=quality,
        )


def _timestamped_files(directory: Path, suffix: str) -> dict[int, Path]:
    files: dict[int, Path] = {}
    for path in directory.glob(f"*.{suffix}"):
        try:
            timestamp = int(path.stem)
        except ValueError as exc:
            raise ValueError(f"{path}: filename must be a nanosecond timestamp") from exc
        files[timestamp] = path
    return files


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


def compute_visual_odometry(config: RunConfig) -> VisualOdometryResult:
    """Run stereo VO and persist accepted relative motions for inspection."""
    left = _timestamped_files(config.dataset / "image" / "stereo_left", "png")
    right = _timestamped_files(config.dataset / "image" / "stereo_right", "png")
    timestamps = sorted(set(left).intersection(right))[:: config.visual_frame_step]
    if len(timestamps) < 2:
        raise FileNotFoundError("At least two timestamp-matched stereo pairs are required")
    frontend = _StereoFrontend(config)
    previous = frontend.read_frame(timestamps[0], left[timestamps[0]], right[timestamps[0]])
    motions: list[RelativeMotion] = []
    rejected = 0
    for timestamp in timestamps[1:]:
        current = frontend.read_frame(timestamp, left[timestamp], right[timestamp])
        try:
            motion = frontend.estimate(previous, current, config)
        except cv2.error:
            motion = None
        if motion is None:
            rejected += 1
        else:
            motions.append(motion)
        previous = current
    _write_motions(config.output / "visual_odometry.csv", motions)
    return VisualOdometryResult(motions, len(timestamps) - 1, rejected)

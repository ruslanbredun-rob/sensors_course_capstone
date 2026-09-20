"""Typed, repository-relative configuration grouped by subsystem."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class GeneralConfig:
    dataset: Path
    output: Path
    max_dt_s: float


@dataclass(frozen=True)
class EvaluationConfig:
    reference_fix_state: int
    reference_tolerance_ns: int


@dataclass(frozen=True)
class ImuConfig:
    gyro_std_rad_s: float
    accel_std_m_s2: float
    gyro_bias_walk_rad_s_sqrt_s: float
    accel_bias_walk_m_s2_sqrt_s: float


@dataclass(frozen=True)
class WheelConfig:
    speed_std_m_s: float
    yaw_rate_std_rad_s: float
    speed_nis_threshold: float
    yaw_nis_threshold: float


@dataclass(frozen=True)
class SlipConfig:
    yaw_threshold_rad_s: float
    accel_threshold_m_s2: float
    enter_count: int
    exit_count: int
    fault_start_s: float
    fault_duration_s: float
    fault_right_scale: float


@dataclass(frozen=True)
class VisualOdometryConfig:
    frame_step: int
    image_scale: float
    min_matches: int
    min_quality: float
    max_speed_m_s: float
    min_fusion_coverage: float


@dataclass(frozen=True)
class LidarOdometryConfig:
    frame_step: int
    voxel_m: float
    max_correspondence_m: float
    max_rmse_m: float
    min_inlier_ratio: float
    min_quality: float
    initial_speed_m_s: float
    min_fusion_coverage: float


@dataclass(frozen=True)
class FusionConfig:
    relative_speed_nis_threshold: float
    relative_yaw_nis_threshold: float
    relative_pose_nis_threshold: float
    relative_pose_max_covariance_scale: float
    relative_fallback_window_s: float


@dataclass(frozen=True)
class RunConfig:
    general: GeneralConfig
    evaluation: EvaluationConfig
    imu: ImuConfig
    wheel: WheelConfig
    slip: SlipConfig
    visual_odometry: VisualOdometryConfig
    lidar_odometry: LidarOdometryConfig
    fusion: FusionConfig


def _section(values: dict[str, Any], name: str, config_path: Path) -> dict[str, Any]:
    section = values.get(name)
    if not isinstance(section, dict):
        raise ValueError(f"{config_path}: expected object section '{name}'")
    return section


def _positive(
    section: dict[str, Any],
    section_name: str,
    keys: tuple[str, ...],
    config_path: Path,
) -> None:
    for key in keys:
        try:
            value = float(section[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{config_path}: invalid {section_name}.{key}") from exc
        if value <= 0:
            raise ValueError(f"{config_path}: {section_name}.{key} must be positive")


def load_config(
    config_path: Path, *, dataset: Path | None = None, output: Path | None = None
) -> RunConfig:
    """Load the nested JSON schema and apply optional CLI path overrides."""
    with config_path.open(encoding="utf-8") as stream:
        values = json.load(stream)
    if not isinstance(values, dict):
        raise ValueError(f"{config_path}: top-level value must be an object")

    general = _section(values, "general", config_path)
    evaluation = _section(values, "evaluation", config_path)
    imu = _section(values, "imu", config_path)
    wheel = _section(values, "wheel", config_path)
    slip = _section(values, "slip", config_path)
    visual = _section(values, "visual_odometry", config_path)
    lidar = _section(values, "lidar_odometry", config_path)
    fusion = _section(values, "fusion", config_path)

    _positive(general, "general", ("max_dt_s",), config_path)
    _positive(evaluation, "evaluation", ("reference_tolerance_ms",), config_path)
    _positive(
        imu,
        "imu",
        (
            "gyro_std_rad_s",
            "accel_std_m_s2",
            "gyro_bias_walk_rad_s_sqrt_s",
            "accel_bias_walk_m_s2_sqrt_s",
        ),
        config_path,
    )
    _positive(
        wheel,
        "wheel",
        (
            "speed_std_m_s",
            "yaw_rate_std_rad_s",
            "speed_nis_threshold",
            "yaw_nis_threshold",
        ),
        config_path,
    )
    _positive(
        slip,
        "slip",
        (
            "yaw_threshold_rad_s",
            "accel_threshold_m_s2",
            "enter_count",
            "exit_count",
            "fault_duration_s",
            "fault_right_scale",
        ),
        config_path,
    )
    _positive(
        visual,
        "visual_odometry",
        (
            "frame_step",
            "image_scale",
            "min_matches",
            "min_quality",
            "max_speed_m_s",
            "min_fusion_coverage",
        ),
        config_path,
    )
    _positive(
        lidar,
        "lidar_odometry",
        (
            "frame_step",
            "voxel_m",
            "max_correspondence_m",
            "max_rmse_m",
            "min_inlier_ratio",
            "min_quality",
            "initial_speed_m_s",
            "min_fusion_coverage",
        ),
        config_path,
    )
    for section_name, section in (
        ("visual_odometry", visual),
        ("lidar_odometry", lidar),
    ):
        if float(section["min_fusion_coverage"]) > 1.0:
            raise ValueError(
                f"{config_path}: {section_name}.min_fusion_coverage must be <= 1"
            )
    _positive(
        fusion,
        "fusion",
        (
            "relative_speed_nis_threshold",
            "relative_yaw_nis_threshold",
            "relative_pose_nis_threshold",
            "relative_pose_max_covariance_scale",
            "relative_fallback_window_s",
        ),
        config_path,
    )

    def resolve(value: str | Path) -> Path:
        path = Path(value).expanduser()
        return path.resolve() if path.is_absolute() else (PROJECT_ROOT / path).resolve()

    return RunConfig(
        general=GeneralConfig(
            dataset=resolve(dataset or general["dataset"]),
            output=resolve(output or general["output"]),
            max_dt_s=float(general["max_dt_s"]),
        ),
        evaluation=EvaluationConfig(
            reference_fix_state=int(evaluation["reference_fix_state"]),
            reference_tolerance_ns=int(evaluation["reference_tolerance_ms"] * 1_000_000),
        ),
        imu=ImuConfig(
            gyro_std_rad_s=float(imu["gyro_std_rad_s"]),
            accel_std_m_s2=float(imu["accel_std_m_s2"]),
            gyro_bias_walk_rad_s_sqrt_s=float(imu["gyro_bias_walk_rad_s_sqrt_s"]),
            accel_bias_walk_m_s2_sqrt_s=float(imu["accel_bias_walk_m_s2_sqrt_s"]),
        ),
        wheel=WheelConfig(
            speed_std_m_s=float(wheel["speed_std_m_s"]),
            yaw_rate_std_rad_s=float(wheel["yaw_rate_std_rad_s"]),
            speed_nis_threshold=float(wheel["speed_nis_threshold"]),
            yaw_nis_threshold=float(wheel["yaw_nis_threshold"]),
        ),
        slip=SlipConfig(
            yaw_threshold_rad_s=float(slip["yaw_threshold_rad_s"]),
            accel_threshold_m_s2=float(slip["accel_threshold_m_s2"]),
            enter_count=int(slip["enter_count"]),
            exit_count=int(slip["exit_count"]),
            fault_start_s=float(slip["fault_start_s"]),
            fault_duration_s=float(slip["fault_duration_s"]),
            fault_right_scale=float(slip["fault_right_scale"]),
        ),
        visual_odometry=VisualOdometryConfig(
            frame_step=int(visual["frame_step"]),
            image_scale=float(visual["image_scale"]),
            min_matches=int(visual["min_matches"]),
            min_quality=float(visual["min_quality"]),
            max_speed_m_s=float(visual["max_speed_m_s"]),
            min_fusion_coverage=float(visual["min_fusion_coverage"]),
        ),
        lidar_odometry=LidarOdometryConfig(
            frame_step=int(lidar["frame_step"]),
            voxel_m=float(lidar["voxel_m"]),
            max_correspondence_m=float(lidar["max_correspondence_m"]),
            max_rmse_m=float(lidar["max_rmse_m"]),
            min_inlier_ratio=float(lidar["min_inlier_ratio"]),
            min_quality=float(lidar["min_quality"]),
            initial_speed_m_s=float(lidar["initial_speed_m_s"]),
            min_fusion_coverage=float(lidar["min_fusion_coverage"]),
        ),
        fusion=FusionConfig(
            relative_speed_nis_threshold=float(fusion["relative_speed_nis_threshold"]),
            relative_yaw_nis_threshold=float(fusion["relative_yaw_nis_threshold"]),
            relative_pose_nis_threshold=float(fusion["relative_pose_nis_threshold"]),
            relative_pose_max_covariance_scale=float(
                fusion["relative_pose_max_covariance_scale"]
            ),
            relative_fallback_window_s=float(fusion["relative_fallback_window_s"]),
        ),
    )

"""Repository-relative configuration and CLI overrides."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class RunConfig:
    dataset: Path
    output: Path
    reference_fix_state: int
    reference_tolerance_ns: int
    wheel_speed_std_m_s: float
    wheel_yaw_rate_std_rad_s: float
    gyro_std_rad_s: float
    accel_std_m_s2: float
    gyro_bias_walk_rad_s_sqrt_s: float
    accel_bias_walk_m_s2_sqrt_s: float
    wheel_nis_threshold: float
    wheel_yaw_nis_threshold: float
    slip_yaw_threshold_rad_s: float
    slip_accel_threshold_m_s2: float
    slip_enter_count: int
    slip_exit_count: int
    slip_fault_start_s: float
    slip_fault_duration_s: float
    slip_fault_right_scale: float
    visual_frame_step: int
    visual_image_scale: float
    visual_min_matches: int
    visual_max_speed_m_s: float
    lidar_frame_step: int
    lidar_voxel_m: float
    lidar_max_correspondence_m: float
    lidar_max_rmse_m: float
    lidar_min_inlier_ratio: float
    lidar_initial_speed_m_s: float
    relative_speed_nis_threshold: float
    relative_yaw_nis_threshold: float
    max_dt_s: float


def load_config(
    config_path: Path, *, dataset: Path | None = None, output: Path | None = None
) -> RunConfig:
    with config_path.open(encoding="utf-8") as stream:
        values = json.load(stream)

    def resolve(value: str | Path) -> Path:
        path = Path(value).expanduser()
        return path.resolve() if path.is_absolute() else (PROJECT_ROOT / path).resolve()

    positive = (
        "wheel_speed_std_m_s",
        "wheel_yaw_rate_std_rad_s",
        "gyro_std_rad_s",
        "accel_std_m_s2",
        "gyro_bias_walk_rad_s_sqrt_s",
        "accel_bias_walk_m_s2_sqrt_s",
        "wheel_nis_threshold",
        "wheel_yaw_nis_threshold",
        "slip_yaw_threshold_rad_s",
        "slip_accel_threshold_m_s2",
        "slip_enter_count",
        "slip_exit_count",
        "slip_fault_duration_s",
        "slip_fault_right_scale",
        "visual_frame_step",
        "visual_image_scale",
        "visual_min_matches",
        "visual_max_speed_m_s",
        "lidar_frame_step",
        "lidar_voxel_m",
        "lidar_max_correspondence_m",
        "lidar_max_rmse_m",
        "lidar_min_inlier_ratio",
        "lidar_initial_speed_m_s",
        "relative_speed_nis_threshold",
        "relative_yaw_nis_threshold",
        "max_dt_s",
    )
    for key in positive:
        if float(values[key]) <= 0:
            raise ValueError(f"{config_path}: {key} must be positive")

    return RunConfig(
        dataset=resolve(dataset or values["dataset"]),
        output=resolve(output or values["output"]),
        reference_fix_state=int(values["reference_fix_state"]),
        reference_tolerance_ns=int(values["reference_tolerance_ms"] * 1_000_000),
        wheel_speed_std_m_s=float(values["wheel_speed_std_m_s"]),
        wheel_yaw_rate_std_rad_s=float(values["wheel_yaw_rate_std_rad_s"]),
        gyro_std_rad_s=float(values["gyro_std_rad_s"]),
        accel_std_m_s2=float(values["accel_std_m_s2"]),
        gyro_bias_walk_rad_s_sqrt_s=float(values["gyro_bias_walk_rad_s_sqrt_s"]),
        accel_bias_walk_m_s2_sqrt_s=float(values["accel_bias_walk_m_s2_sqrt_s"]),
        wheel_nis_threshold=float(values["wheel_nis_threshold"]),
        wheel_yaw_nis_threshold=float(values["wheel_yaw_nis_threshold"]),
        slip_yaw_threshold_rad_s=float(values["slip_yaw_threshold_rad_s"]),
        slip_accel_threshold_m_s2=float(values["slip_accel_threshold_m_s2"]),
        slip_enter_count=int(values["slip_enter_count"]),
        slip_exit_count=int(values["slip_exit_count"]),
        slip_fault_start_s=float(values["slip_fault_start_s"]),
        slip_fault_duration_s=float(values["slip_fault_duration_s"]),
        slip_fault_right_scale=float(values["slip_fault_right_scale"]),
        visual_frame_step=int(values["visual_frame_step"]),
        visual_image_scale=float(values["visual_image_scale"]),
        visual_min_matches=int(values["visual_min_matches"]),
        visual_max_speed_m_s=float(values["visual_max_speed_m_s"]),
        lidar_frame_step=int(values["lidar_frame_step"]),
        lidar_voxel_m=float(values["lidar_voxel_m"]),
        lidar_max_correspondence_m=float(values["lidar_max_correspondence_m"]),
        lidar_max_rmse_m=float(values["lidar_max_rmse_m"]),
        lidar_min_inlier_ratio=float(values["lidar_min_inlier_ratio"]),
        lidar_initial_speed_m_s=float(values["lidar_initial_speed_m_s"]),
        relative_speed_nis_threshold=float(values["relative_speed_nis_threshold"]),
        relative_yaw_nis_threshold=float(values["relative_yaw_nis_threshold"]),
        max_dt_s=float(values["max_dt_s"]),
    )

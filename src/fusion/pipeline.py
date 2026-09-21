"""Experiment orchestration, result export and independent validation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from src.camera.visual_odometry import (
    VisualOdometryResult,
    compute_visual_odometry,
    read_visual_odometry,
)
from src.camera.vio import (
    VioOdometryResult,
    compute_vio_odometry,
    read_vio_odometry,
)
from src.common.config import RunConfig
from src.common.models import (
    Estimate,
    ImuSample,
    RelativeMotion,
    RelativePoseEpoch,
    WheelMeasurement,
)
from src.common.synchronization import ordered_sensor_events
from src.dataset.readers import read_encoders, read_vrs_reference
from src.evaluation.artifacts import export_validation_artifacts
from src.evaluation.metrics import PositionEvaluation, evaluate_position
from src.fusion.ekf import VehicleEKF
from src.imu.reader import read_imu
from src.lidar.odometry import (
    LidarOdometryResult,
    compute_lidar_odometry,
    read_lidar_odometry,
)
from src.wheel.odometry import (
    wheel_measurements,
)


@dataclass
class ExperimentResult:
    name: str
    states: list[Estimate]
    wheel_updates: int
    rejected_wheel_updates: int
    relative_updates: int = 0
    rejected_relative_updates: int = 0
    available_relative_updates: int = 0
    evaluation: PositionEvaluation | None = None


def _require_inputs(dataset: Path, *, mode: str, validate: bool) -> None:
    required = [
        dataset / "sensor_data" / "encoder.csv",
        dataset / "sensor_data" / "xsens_imu.csv",
        dataset / "calibration" / "EncoderParameter.txt",
        dataset / "calibration" / "Vehicle2IMU.txt",
    ]
    if validate:
        required.extend(
            (
                dataset / "sensor_data" / "vrs_gps.csv",
                dataset / "calibration" / "Vehicle2VRS.txt",
            )
        )
    if mode in ("visual", "vio", "full", "all"):
        required.extend(
            (
                dataset / "calibration" / "left.yaml",
                dataset / "calibration" / "right.yaml",
                dataset / "calibration" / "Vehicle2Stereo.txt",
                dataset / "image" / "stereo_left",
                dataset / "image" / "stereo_right",
            )
        )
    if mode in ("lidar", "full", "all"):
        required.extend(
            (
                dataset / "calibration" / "Vehicle2LeftVLP.txt",
                dataset / "sensor_data" / "VLP_left",
            )
        )
    missing = [path for path in required if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing dataset files: "
            + ", ".join(str(path) for path in missing)
            + ". See data/README.md."
        )
    imu_transform = (dataset / "calibration" / "Vehicle2IMU.txt").read_text(
        encoding="utf-8"
    )
    if "R: 1 0 0 0 1 0 0 0 1" not in imu_transform:
        raise ValueError("The current planar model requires identity Vehicle2IMU R")
    if validate:
        vrs_transform = (dataset / "calibration" / "Vehicle2VRS.txt").read_text(
            encoding="utf-8"
        )
        translation = next(
            (line for line in vrs_transform.splitlines() if line.startswith("T:")),
            None,
        )
        if translation is None:
            raise ValueError("Vehicle2VRS.txt has no translation")
        x_m, y_m, _ = map(float, translation.removeprefix("T:").split())
        if abs(x_m) > 1e-6 or abs(y_m) > 1e-6:
            raise ValueError("Transform the horizontal VRS lever arm before validation")


def _load_wheels(config: RunConfig) -> list[WheelMeasurement]:
    calibration_path = config.general.dataset / "calibration" / "EncoderParameter.txt"
    return list(
        wheel_measurements(read_encoders(config.general.dataset), calibration_path)
    )


def _write_estimates(path: Path, estimates: list[Estimate]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("timestamp_ns", "x_m", "y_m", "yaw_rad", "speed_m_s"))
        for state in estimates:
            writer.writerow(
                (
                    state.timestamp_ns,
                    f"{state.x_m:.6f}",
                    f"{state.y_m:.6f}",
                    f"{state.yaw_rad:.8f}",
                    f"{state.speed_m_s:.6f}",
                )
            )


def _run_filter(
    config: RunConfig,
    name: str,
    imu: list[ImuSample],
    wheels: list[WheelMeasurement],
    relative_streams: list[list[RelativeMotion]] | None = None,
    relative_epoch_streams: list[list[RelativePoseEpoch]] | None = None,
    *,
    max_events: int | None,
) -> ExperimentResult:
    # Every INS configuration uses both complementary base sensors: wheel
    # kinematics for ground motion and IMU for high-rate propagation.
    use_kinematics = True
    estimator = VehicleEKF(config)
    states: list[Estimate] = []
    wheel_count = rejected = 0
    relative_count = rejected_relative = relative_available = 0
    last_lidar_motion_ns = -1
    event_count = 0
    diagnostics_path = config.general.output / f"diagnostics_{name}.csv"
    relative_path = config.general.output / f"diagnostics_relative_{name}.csv"
    with (
        diagnostics_path.open("w", newline="", encoding="utf-8") as stream,
        relative_path.open("w", newline="", encoding="utf-8") as relative_stream,
    ):
        writer = csv.writer(stream, lineterminator="\n")
        relative_writer = csv.writer(relative_stream, lineterminator="\n")
        writer.writerow(
            (
                "timestamp_ns",
                "wheel_speed_m_s",
                "wheel_yaw_rate_rad_s",
                "speed_nis",
                "yaw_rate_nis",
                "wheel_accepted",
                "yaw_rate_accepted",
            )
        )
        relative_writer.writerow(
            (
                "timestamp_ns",
                "source",
                "dx_m",
                "dy_m",
                "dyaw_rad",
                "quality",
                "speed_nis",
                "yaw_rate_nis",
                "speed_accepted",
                "yaw_rate_accepted",
                "pose_nis",
                "pose_accepted",
                "covariance_scale",
                "update_kind",
                "used_as_correction",
            )
        )
        streams = [imu, wheels]
        streams.extend(relative_streams or [])
        streams.extend(relative_epoch_streams or [])
        for event in ordered_sensor_events(*streams):
            if isinstance(event, ImuSample):
                estimate = estimator.predict(event)
                states.append(estimate)
            elif isinstance(event, WheelMeasurement):
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                estimate = estimator.update_wheel(
                    event,
                    use_yaw_rate=use_kinematics,
                )
                wheel_count += 1
                rejected += int(not estimator.last_wheel_accepted)
                writer.writerow(
                    (
                        event.timestamp_ns,
                        f"{event.speed_m_s:.6f}",
                        f"{event.yaw_rate_rad_s:.6f}",
                        ""
                        if estimator.last_wheel_nis is None
                        else f"{estimator.last_wheel_nis:.6f}",
                        ""
                        if estimator.last_wheel_yaw_nis is None
                        else f"{estimator.last_wheel_yaw_nis:.6f}",
                        estimator.last_wheel_accepted,
                        estimator.last_wheel_yaw_accepted,
                    )
                )
            elif isinstance(event, RelativePoseEpoch):
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                estimator.store_relative_pose_anchor(event.source, event.timestamp_ns)
            else:
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                relative_available += 1
                if event.source == "lidar":
                    last_lidar_motion_ns = event.timestamp_ns
                lidar_recent = (
                    last_lidar_motion_ns >= 0
                    and event.timestamp_ns - last_lidar_motion_ns
                    <= int(config.fusion.relative_fallback_window_s * 1e9)
                )
                # VO and LO share much of the same vehicle motion and are not
                # independent. In the combined mode, prefer the substantially
                # lower-noise LiDAR increment and use VO only across LO gaps.
                use_correction = not (
                    name == "full"
                    and event.source in ("visual", "vio")
                    and lidar_recent
                )
                if use_correction:
                    estimate = estimator.update_relative_pose(event)
                    accepted = bool(estimator.last_relative_pose_accepted)
                    pose_nis = f"{estimator.last_relative_pose_nis:.6f}"
                    pose_accepted = estimator.last_relative_pose_accepted
                    covariance_scale = (
                        f"{estimator.last_relative_pose_covariance_scale:.6f}"
                    )
                    update_kind = "pose"
                    relative_count += 1
                    rejected_relative += int(not accepted)
                else:
                    pose_nis = covariance_scale = update_kind = ""
                    pose_accepted = False
                speed_nis = yaw_nis = ""
                speed_accepted = yaw_accepted = False
                relative_writer.writerow(
                    (
                        event.timestamp_ns,
                        event.source,
                        f"{event.dx_m:.6f}",
                        f"{event.dy_m:.6f}",
                        f"{event.dyaw_rad:.8f}",
                        f"{event.quality:.6f}",
                        speed_nis,
                        yaw_nis,
                        speed_accepted,
                        yaw_accepted,
                        pose_nis,
                        pose_accepted,
                        covariance_scale,
                        update_kind,
                        use_correction,
                    )
                )
            event_count += 1
            if max_events is not None and event_count >= max_events:
                break
    if not states or wheel_count == 0:
        raise ValueError("Both IMU and wheel streams are required")
    _write_estimates(config.general.output / f"estimated_state_{name}.csv", states)
    return ExperimentResult(
        name,
        states,
        wheel_count,
        rejected,
        relative_count,
        rejected_relative,
        relative_available,
    )


def _evaluate(result: ExperimentResult, reference: list, config: RunConfig) -> None:
    result.evaluation = evaluate_position(
        result.states,
        reference,
        valid_fix_state=config.evaluation.reference_fix_state,
        tolerance_ns=config.evaluation.reference_tolerance_ns,
    )


def _format_summary(results: list[ExperimentResult]) -> str:
    lines = []
    for result in results:
        final = result.states[-1]
        line = (
            f"{result.name}: states={len(result.states)}, wheel={result.wheel_updates}, "
            f"rejected={result.rejected_wheel_updates}, "
            f"relative={result.relative_updates}/{result.available_relative_updates}, "
            f"relative_rejected={result.rejected_relative_updates}, "
            f"final=({final.x_m:.3f}, {final.y_m:.3f}) m"
        )
        if result.evaluation is not None:
            line += f", RMSE={result.evaluation.rmse_m:.3f} m"
        lines.append(line)
    return "\n".join(lines) + "\n"


def _evaluations(
    results: list[ExperimentResult],
) -> dict[str, PositionEvaluation]:
    evaluations = {}
    for result in results:
        if result.evaluation is not None:
            evaluations[result.name] = result.evaluation
    return evaluations


def _frontend_ready(accepted: int, attempted: int, minimum_coverage: float) -> bool:
    return attempted > 0 and accepted / attempted >= minimum_coverage


def wheel_turning_fraction(
    wheels: list[WheelMeasurement], *, yaw_rate_threshold_rad_s: float
) -> float:
    """Return the duration-weighted fraction with informative turning motion."""
    if len(wheels) < 2:
        return 0.0
    total_duration_s = 0.0
    turning_duration_s = 0.0
    for previous, current in zip(wheels, wheels[1:]):
        dt_s = (current.timestamp_ns - previous.timestamp_ns) * 1e-9
        if dt_s <= 0.0:
            raise ValueError("Wheel timestamps are not strictly increasing")
        total_duration_s += dt_s
        if abs(current.yaw_rate_rad_s) >= yaw_rate_threshold_rad_s:
            turning_duration_s += dt_s
    return turning_duration_s / total_duration_s if total_duration_s else 0.0


def run(
    config: RunConfig,
    mode: str,
    *,
    max_events: int | None = None,
    validate: bool = False,
    reuse_frontends: bool = False,
) -> None:
    _require_inputs(config.general.dataset, mode=mode, validate=validate)
    config.general.output.mkdir(parents=True, exist_ok=True)
    imu = list(read_imu(config.general.dataset))
    wheels = _load_wheels(config)
    visual: VisualOdometryResult | None = None
    visual_ready = False
    if mode in ("visual", "vio", "full", "all"):
        visual = (
            read_visual_odometry(config)
            if reuse_frontends
            else compute_visual_odometry(config, wheels)
        )
        visual_ready = _frontend_ready(
            len(visual.motions),
            visual.attempted_pairs,
            config.visual_odometry.min_fusion_coverage,
        )
        print(
            f"visual frontend: accepted={len(visual.motions)}/"
            f"{visual.attempted_pairs}, rejected={visual.rejected_pairs}, "
            f"fusion={'enabled' if visual_ready else 'disabled (low coverage)'}"
        )
    vio: VioOdometryResult | None = None
    vio_ready = False
    if mode in ("vio", "full", "all"):
        assert visual is not None
        vio = (
            read_vio_odometry(config, visual)
            if reuse_frontends
            and (config.general.output / "vio_odometry.csv").exists()
            else compute_vio_odometry(config, imu, visual)
        )
        vio_ready = _frontend_ready(
            len(vio.motions),
            vio.attempted_pairs,
            config.vio.min_fusion_coverage,
        )
        print(
            f"vio frontend: accepted={len(vio.motions)}/"
            f"{vio.attempted_pairs}, rejected={vio.rejected_pairs}, "
            f"fusion={'enabled' if vio_ready else 'disabled (low coverage)'}"
        )
    lidar: LidarOdometryResult | None = None
    lidar_ready = False
    if mode in ("lidar", "full", "all"):
        turning_fraction = wheel_turning_fraction(
            wheels,
            yaw_rate_threshold_rad_s=(
                config.lidar_odometry.turning_yaw_rate_rad_s
            ),
        )
        motion_ready = (
            turning_fraction >= config.lidar_odometry.min_turning_fraction
        )
        if motion_ready:
            lidar = (
                read_lidar_odometry(config)
                if reuse_frontends
                else compute_lidar_odometry(config, wheels)
            )
            lidar_ready = _frontend_ready(
                len(lidar.motions),
                lidar.attempted_pairs,
                config.lidar_odometry.min_fusion_coverage,
            )
            print(
                f"lidar frontend: accepted={len(lidar.motions)}/"
                f"{lidar.attempted_pairs}, rejected={lidar.rejected_pairs}, "
                f"turning={turning_fraction:.1%}, "
                f"fusion={'enabled' if lidar_ready else 'disabled (low coverage)'}"
            )
        else:
            print(
                f"lidar frontend: skipped, turning={turning_fraction:.1%} "
                f"< {config.lidar_odometry.min_turning_fraction:.1%}"
            )
    modes = (
        ["base", "visual", "vio", "lidar", "full"]
        if mode == "all"
        else [mode]
    )
    results = []
    for name in modes:
        relative_streams = []
        relative_epoch_streams = []
        if name in ("visual", "full") and visual and visual_ready:
            if name == "visual" or not vio_ready:
                relative_streams.append(visual.motions)
                relative_epoch_streams.append(visual.epochs)
        if name in ("vio", "full") and vio and vio_ready:
            relative_streams.append(vio.motions)
            relative_epoch_streams.append(vio.epochs)
        if name in ("lidar", "full") and lidar and lidar_ready:
            relative_streams.append(lidar.motions)
            relative_epoch_streams.append(lidar.epochs)
        results.append(
            _run_filter(
                config,
                name,
                imu,
                wheels,
                relative_streams,
                relative_epoch_streams,
                max_events=max_events,
            )
        )

    if validate:
        reference = list(read_vrs_reference(config.general.dataset))
        for result in results:
            _evaluate(result, reference, config)
    print(_format_summary(results), end="")
    if validate:
        export_validation_artifacts(config, _evaluations(results))

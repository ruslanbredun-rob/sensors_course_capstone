"""Experiment orchestration, result export and independent validation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

from .config import RunConfig
from .dataset import read_encoders, read_imu, read_vrs_reference
from .ekf import VehicleEKF
from .evaluation import PositionEvaluation, evaluate_position
from .models import Estimate, ImuSample, RelativeMotion, WheelMeasurement
from .slip_detection import WheelSlipDetector
from .synchronization import ordered_sensor_events
from .visual_odometry import VisualOdometryResult, compute_visual_odometry
from .wheel_odometry import (
    inject_right_wheel_scale_fault,
    read_encoder_calibration,
    wheel_measurements,
    wheel_only_baseline,
)


@dataclass
class ExperimentResult:
    name: str
    states: list[Estimate]
    wheel_updates: int
    rejected_wheel_updates: int
    slip_samples: int
    injected_samples: int
    detected_injected_samples: int
    false_positive_samples: int
    relative_updates: int = 0
    rejected_relative_updates: int = 0
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
    if mode in ("visual", "full", "all"):
        required.extend(
            (
                dataset / "calibration" / "left.yaml",
                dataset / "calibration" / "right.yaml",
                dataset / "calibration" / "Vehicle2Stereo.txt",
                dataset / "image" / "stereo_left",
                dataset / "image" / "stereo_right",
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


def _load_wheels(config: RunConfig, *, inject_slip: bool) -> list[WheelMeasurement]:
    calibration_path = config.dataset / "calibration" / "EncoderParameter.txt"
    calibration = read_encoder_calibration(calibration_path)
    wheels = list(wheel_measurements(read_encoders(config.dataset), calibration_path))
    if inject_slip and wheels:
        start_ns = wheels[0].timestamp_ns + int(config.slip_fault_start_s * 1e9)
        end_ns = start_ns + int(config.slip_fault_duration_s * 1e9)
        wheels = list(
            inject_right_wheel_scale_fault(
                wheels,
                wheel_base_m=calibration.wheel_base_m,
                start_ns=start_ns,
                end_ns=end_ns,
                scale=config.slip_fault_right_scale,
            )
        )
    return wheels


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
    wheels: list[WheelMeasurement],
    relative_streams: list[list[RelativeMotion]] | None = None,
    *,
    max_events: int | None,
) -> ExperimentResult:
    use_kinematics = name != "base"
    detector = (
        WheelSlipDetector(
            yaw_threshold_rad_s=config.slip_yaw_threshold_rad_s,
            accel_threshold_m_s2=config.slip_accel_threshold_m_s2,
            enter_count=config.slip_enter_count,
            exit_count=config.slip_exit_count,
        )
        if use_kinematics
        else None
    )
    estimator = VehicleEKF(config)
    states: list[Estimate] = []
    wheel_count = rejected = slip_samples = 0
    injected = detected_injected = false_positive = 0
    relative_count = rejected_relative = 0
    event_count = 0
    diagnostics_path = config.output / f"diagnostics_{name}.csv"
    relative_path = config.output / f"diagnostics_relative_{name}.csv"
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
                "slip_active",
                "fault_injected",
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
            )
        )
        streams = [read_imu(config.dataset), wheels]
        streams.extend(relative_streams or [])
        for event in ordered_sensor_events(*streams):
            if isinstance(event, ImuSample):
                estimate = estimator.predict(event)
                states.append(estimate)
            elif isinstance(event, WheelMeasurement):
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                slip_active = False
                if detector is not None:
                    decision = detector.update(
                        event,
                        estimator.current_imu,
                        float(estimator.x[4]),
                    )
                    slip_active = decision.active
                estimator.update_wheel(
                    event,
                    use_yaw_rate=use_kinematics,
                    reject=slip_active,
                )
                wheel_count += 1
                rejected += int(not estimator.last_wheel_accepted)
                slip_samples += int(slip_active)
                injected += int(event.fault_injected)
                detected_injected += int(event.fault_injected and slip_active)
                false_positive += int(not event.fault_injected and slip_active)
                writer.writerow(
                    (
                        event.timestamp_ns,
                        f"{event.speed_m_s:.6f}",
                        f"{event.yaw_rate_rad_s:.6f}",
                        "" if estimator.last_wheel_nis is None else f"{estimator.last_wheel_nis:.6f}",
                        "" if estimator.last_wheel_yaw_nis is None else f"{estimator.last_wheel_yaw_nis:.6f}",
                        estimator.last_wheel_accepted,
                        estimator.last_wheel_yaw_accepted,
                        slip_active,
                        event.fault_injected,
                    )
                )
            else:
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                estimator.update_relative_motion(event)
                relative_count += 1
                accepted = bool(
                    estimator.last_relative_speed_accepted
                    and estimator.last_relative_yaw_accepted
                )
                rejected_relative += int(not accepted)
                relative_writer.writerow(
                    (
                        event.timestamp_ns,
                        event.source,
                        f"{event.dx_m:.6f}",
                        f"{event.dy_m:.6f}",
                        f"{event.dyaw_rad:.8f}",
                        f"{event.quality:.6f}",
                        f"{estimator.last_relative_speed_nis:.6f}",
                        f"{estimator.last_relative_yaw_nis:.6f}",
                        estimator.last_relative_speed_accepted,
                        estimator.last_relative_yaw_accepted,
                    )
                )
            event_count += 1
            if max_events is not None and event_count >= max_events:
                break
    if not states or wheel_count == 0:
        raise ValueError("Both IMU and wheel streams are required")
    _write_estimates(config.output / f"estimated_state_{name}.csv", states)
    return ExperimentResult(
        name,
        states,
        wheel_count,
        rejected,
        slip_samples,
        injected,
        detected_injected,
        false_positive,
        relative_count,
        rejected_relative,
    )


def _evaluate(result: ExperimentResult, reference: list, config: RunConfig) -> None:
    result.evaluation = evaluate_position(
        result.states,
        reference,
        valid_fix_state=config.reference_fix_state,
        tolerance_ns=config.reference_tolerance_ns,
    )


def _write_summary(config: RunConfig, results: list[ExperimentResult]) -> str:
    lines = []
    for result in results:
        final = result.states[-1]
        line = (
            f"{result.name}: states={len(result.states)}, wheel={result.wheel_updates}, "
            f"rejected={result.rejected_wheel_updates}, slip={result.slip_samples}, "
            f"relative={result.relative_updates}, relative_rejected={result.rejected_relative_updates}, "
            f"final=({final.x_m:.3f}, {final.y_m:.3f}) m"
        )
        if result.evaluation is not None:
            line += f", RMSE={result.evaluation.rmse_m:.3f} m"
        lines.append(line)
        if result.injected_samples:
            detection_rate = result.detected_injected_samples / result.injected_samples
            non_fault = result.wheel_updates - result.injected_samples
            false_rate = result.false_positive_samples / max(non_fault, 1)
            lines.append(
                f"  injected slip: detection={detection_rate:.1%}, "
                f"false-positive samples={false_rate:.2%}"
            )
    summary = "\n".join(lines) + "\n"
    (config.output / "run_summary.txt").write_text(summary, encoding="utf-8")
    return summary


def run(
    config: RunConfig,
    mode: str,
    *,
    max_events: int | None = None,
    validate: bool = False,
    inject_slip: bool = False,
) -> None:
    _require_inputs(config.dataset, mode=mode, validate=validate)
    config.output.mkdir(parents=True, exist_ok=True)
    wheels = _load_wheels(config, inject_slip=inject_slip)
    visual: VisualOdometryResult | None = None
    if mode in ("visual", "full", "all"):
        visual = compute_visual_odometry(config)
        print(
            f"visual frontend: accepted={len(visual.motions)}/"
            f"{visual.attempted_pairs}, rejected={visual.rejected_pairs}"
        )
    modes = ["base", "slip", "visual"] if mode == "all" else [mode]
    results = []
    for name in modes:
        relative_streams = [visual.motions] if name in ("visual", "full") and visual else []
        results.append(
            _run_filter(
                config,
                name,
                wheels,
                relative_streams,
                max_events=max_events,
            )
        )

    baseline = list(wheel_only_baseline(wheels))
    _write_estimates(config.output / "estimated_state_wheel_only.csv", baseline)
    if validate:
        reference = list(read_vrs_reference(config.dataset))
        baseline_result = ExperimentResult(
            "wheel_only", baseline, len(wheels), 0, 0, 0, 0, 0
        )
        _evaluate(baseline_result, reference, config)
        for result in results:
            _evaluate(result, reference, config)
        results.insert(0, baseline_result)
    print(_write_summary(config, results), end="")

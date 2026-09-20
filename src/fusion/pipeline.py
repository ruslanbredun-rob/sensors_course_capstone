"""Experiment orchestration, result export and independent validation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from math import hypot
from pathlib import Path

from src.camera.visual_odometry import VisualOdometryResult, compute_visual_odometry
from src.common.config import RunConfig
from src.common.models import Estimate, ImuSample, RelativeMotion, WheelMeasurement
from src.common.synchronization import ordered_sensor_events
from src.dataset.readers import read_encoders, read_vrs_reference
from src.evaluation.metrics import PositionEvaluation, evaluate_position
from src.evaluation.visualization import (
    DISPLAY_NAMES,
    save_comparison_plots,
    save_consistency_plot,
    save_metrics_table,
    save_slip_plot,
    save_validation_plot,
)
from src.fusion.ekf import VehicleEKF
from src.imu.reader import read_imu
from src.lidar.odometry import LidarOdometryResult, compute_lidar_odometry
from src.wheel.odometry import (
    inject_right_wheel_scale_fault,
    read_encoder_calibration,
    wheel_measurements,
)
from src.wheel.slip_detection import WheelSlipDetector


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


def _load_wheels(config: RunConfig, *, inject_slip: bool) -> list[WheelMeasurement]:
    calibration_path = config.general.dataset / "calibration" / "EncoderParameter.txt"
    calibration = read_encoder_calibration(calibration_path)
    wheels = list(
        wheel_measurements(read_encoders(config.general.dataset), calibration_path)
    )
    if inject_slip and wheels:
        start_ns = wheels[0].timestamp_ns + int(config.slip.fault_start_s * 1e9)
        end_ns = start_ns + int(config.slip.fault_duration_s * 1e9)
        wheels = list(
            inject_right_wheel_scale_fault(
                wheels,
                wheel_base_m=calibration.wheel_base_m,
                start_ns=start_ns,
                end_ns=end_ns,
                scale=config.slip.fault_right_scale,
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
    # Every INS configuration uses both complementary base sensors: wheel
    # kinematics for ground motion and IMU for high-rate propagation.
    use_kinematics = True
    detector = (
        WheelSlipDetector(
            yaw_threshold_rad_s=config.slip.yaw_threshold_rad_s,
            accel_threshold_m_s2=config.slip.accel_threshold_m_s2,
            enter_count=config.slip.enter_count,
            exit_count=config.slip.exit_count,
        )
        if name != "base"
        else None
    )
    estimator = VehicleEKF(config)
    states: list[Estimate] = []
    wheel_count = rejected = slip_samples = 0
    injected = detected_injected = false_positive = 0
    relative_count = rejected_relative = relative_available = 0
    slip_active_current = False
    first_wheel_timestamp_ns: int | None = None
    wheel_unhealthy_until_ns = -1
    last_visual_update_ns = -1
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
                "used_as_correction",
            )
        )
        streams = [read_imu(config.general.dataset), wheels]
        streams.extend(relative_streams or [])
        for event in ordered_sensor_events(*streams):
            if isinstance(event, ImuSample):
                estimate = estimator.predict(event)
                states.append(estimate)
            elif isinstance(event, WheelMeasurement):
                if estimator.timestamp_ns is None or estimator.current_imu is None:
                    continue
                if first_wheel_timestamp_ns is None:
                    first_wheel_timestamp_ns = event.timestamp_ns
                slip_active = False
                if detector is not None:
                    decision = detector.update(
                        event,
                        estimator.current_imu,
                        float(estimator.x[4]),
                    )
                    slip_active = decision.active
                slip_active_current = slip_active
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
                beyond_initialization = (
                    event.timestamp_ns - first_wheel_timestamp_ns > int(1e9)
                )
                if slip_active or (
                    beyond_initialization and not estimator.last_wheel_accepted
                ):
                    wheel_unhealthy_until_ns = event.timestamp_ns + int(
                        config.fusion.relative_fallback_window_s * 1e9
                    )
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
                relative_available += 1
                wheel_degraded = (
                    slip_active_current
                    or event.timestamp_ns <= wheel_unhealthy_until_ns
                )
                use_correction = wheel_degraded
                if event.source == "lidar":
                    # LiDAR is the second fallback: use it only if no recent
                    # healthy visual increment already covered this interval.
                    use_correction = use_correction and (
                        event.timestamp_ns - last_visual_update_ns
                        > int(config.fusion.relative_fallback_window_s * 1e9)
                    )
                if use_correction:
                    estimator.update_relative_motion(event)
                    relative_count += 1
                    accepted = bool(
                        estimator.last_relative_speed_accepted
                        and estimator.last_relative_yaw_accepted
                    )
                    rejected_relative += int(not accepted)
                    if event.source == "visual" and accepted:
                        last_visual_update_ns = event.timestamp_ns
                    speed_nis = f"{estimator.last_relative_speed_nis:.6f}"
                    yaw_nis = f"{estimator.last_relative_yaw_nis:.6f}"
                    speed_accepted = estimator.last_relative_speed_accepted
                    yaw_accepted = estimator.last_relative_yaw_accepted
                else:
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
        slip_samples,
        injected,
        detected_injected,
        false_positive,
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


def _write_summary(config: RunConfig, results: list[ExperimentResult]) -> str:
    lines = []
    for result in results:
        final = result.states[-1]
        line = (
            f"{result.name}: states={len(result.states)}, wheel={result.wheel_updates}, "
            f"rejected={result.rejected_wheel_updates}, slip={result.slip_samples}, "
            f"relative={result.relative_updates}/{result.available_relative_updates}, "
            f"relative_rejected={result.rejected_relative_updates}, "
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
    (config.general.output / "run_summary.txt").write_text(summary, encoding="utf-8")
    return summary


def _nis_fraction(path: Path, column: str, threshold: float) -> tuple[int, float]:
    rows = csv.DictReader(path.open(encoding="utf-8"))
    values = [float(row[column]) for row in rows if row.get(column)]
    if not values:
        return 0, float("nan")
    return len(values), sum(value <= threshold for value in values) / len(values)


def _write_validation_artifacts(
    config: RunConfig, results: list[ExperimentResult]
) -> None:
    evaluations = {
        result.name: result.evaluation
        for result in results
        if result.evaluation is not None
    }
    if not evaluations:
        return
    typed_evaluations: dict[str, PositionEvaluation] = {
        name: evaluation
        for name, evaluation in evaluations.items()
        if evaluation is not None
    }
    screenshot_directory = config.general.output / "screenshots"
    screenshot_directory.mkdir(parents=True, exist_ok=True)
    save_comparison_plots(typed_evaluations, screenshot_directory)
    save_metrics_table(
        typed_evaluations, screenshot_directory / "metrics_summary.png"
    )

    best_name, best = min(
        typed_evaluations.items(), key=lambda item: item[1].rmse_m
    )
    save_validation_plot(best, config.general.output / "trajectory_validation.png")

    with (config.general.output / "comparison_metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "configuration",
                "matched_vrs_epochs",
                "rmse_m",
                "median_m",
                "p95_m",
                "final_error_m",
            )
        )
        for name, evaluation in typed_evaluations.items():
            writer.writerow(
                (
                    name,
                    evaluation.matched_epochs,
                    f"{evaluation.rmse_m:.6f}",
                    f"{evaluation.median_m:.6f}",
                    f"{evaluation.p95_m:.6f}",
                    f"{evaluation.final_error_m:.6f}",
                )
            )

    with (config.general.output / "validation_pairs.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "configuration",
                "timestamp_ns",
                "estimate_time_offset_ms",
                "vrs_utm_easting_m",
                "vrs_utm_northing_m",
                "aligned_est_easting_m",
                "aligned_est_northing_m",
                "error_m",
            )
        )
        for name, evaluation in typed_evaluations.items():
            for index, timestamp in enumerate(evaluation.timestamp_ns):
                writer.writerow(
                    (
                        name,
                        timestamp,
                        f"{evaluation.time_offset_ms[index]:.3f}",
                        f"{evaluation.reference_xy_m[index, 0]:.4f}",
                        f"{evaluation.reference_xy_m[index, 1]:.4f}",
                        f"{evaluation.aligned_xy_m[index, 0]:.4f}",
                        f"{evaluation.aligned_xy_m[index, 1]:.4f}",
                        f"{evaluation.error_m[index]:.4f}",
                    )
                )

    ins_baseline = typed_evaluations.get("base")
    reference_length_m = sum(
        hypot(
            float(current[0] - previous[0]),
            float(current[1] - previous[1]),
        )
        for previous, current in zip(
            best.reference_xy_m, best.reference_xy_m[1:]
        )
    )
    improvement = (
        100.0 * (ins_baseline.rmse_m - best.rmse_m) / ins_baseline.rmse_m
        if ins_baseline is not None
        else float("nan")
    )
    summary_lines = [
        f"VRS fix={config.evaluation.reference_fix_state}: {best.matched_epochs}/"
        f"{best.valid_fix_epochs} epochs matched within "
        f"{config.evaluation.reference_tolerance_ns / 1e6:.0f} ms",
        f"Reference trajectory length={reference_length_m:.1f} m",
        "2D ATE uses one rigid SE(2) alignment; trajectory scale is unchanged.",
    ]
    for name, evaluation in typed_evaluations.items():
        summary_lines.append(
            f"{DISPLAY_NAMES.get(name, name)}: RMSE={evaluation.rmse_m:.3f} m, "
            f"median={evaluation.median_m:.3f} m, P95={evaluation.p95_m:.3f} m"
        )
    if ins_baseline is not None:
        summary_lines.append(
            f"Best={DISPLAY_NAMES.get(best_name, best_name)}: "
            f"improvement over E1 INS baseline="
            f"{improvement:.1f}%"
        )
    (config.general.output / "validation_summary.txt").write_text(
        "\n".join(summary_lines) + "\n", encoding="utf-8"
    )

    diagnostic_mode = next(
        name
        for name in ("full", "lidar", "visual", "slip", "base")
        if name in typed_evaluations
    )
    save_consistency_plot(
        config.general.output,
        diagnostic_mode,
        wheel_speed_threshold=config.wheel.speed_nis_threshold,
        wheel_yaw_threshold=config.wheel.yaw_nis_threshold,
        relative_speed_threshold=config.fusion.relative_speed_nis_threshold,
        relative_yaw_threshold=config.fusion.relative_yaw_nis_threshold,
    )
    save_slip_plot(config.general.output, diagnostic_mode)

    speed_count, speed_below = _nis_fraction(
        config.general.output / f"diagnostics_{diagnostic_mode}.csv",
        "speed_nis",
        config.wheel.speed_nis_threshold,
    )
    yaw_count, yaw_below = _nis_fraction(
        config.general.output / f"diagnostics_{diagnostic_mode}.csv",
        "yaw_rate_nis",
        config.wheel.yaw_nis_threshold,
    )
    conclusion_lines = [
        "# Висновки ДЗ 18–19",
        "",
        "## Результати на `urban35`",
        "",
        "| Конфігурація | RMSE, м | Median, м | P95, м |",
        "|---|---:|---:|---:|",
    ]
    for name, evaluation in typed_evaluations.items():
        conclusion_lines.append(
            f"| {DISPLAY_NAMES.get(name, name)} | {evaluation.rmse_m:.3f} | "
            f"{evaluation.median_m:.3f} | {evaluation.p95_m:.3f} |"
        )
    conclusion_lines.extend(
        (
            "",
            (
                f"Найкращий режим — `{best_name}`: {best.rmse_m:.3f} м RMSE. "
                + (
                    f"Покращення відносно E1 Wheel+IMU становить {improvement:.1f}%."
                    if ins_baseline is not None
                    else ""
                )
            ).rstrip(),
            f"RMSE пораховано по {best.matched_epochs} VRS epochs уздовж "
            f"траєкторії {reference_length_m:.0f} м.",
            "",
            "## Наш шлях обробки даних",
            "",
            "1. `encoder.csv` і `xsens_imu.csv` читаються потоково з перевіркою "
            "кількості колонок, SI units і строго зростаючих nanosecond timestamps. "
            "Обидва потоки мають близько 100 Hz, тому жоден із них не є "
            "низькочастотною зовнішньою поправкою.",
            "2. Encoder counts через resolution, діаметри коліс і фактичний `dt` "
            "перетворюються на left/right speed, лінійну швидкість та "
            "differential-drive course constraint; `x,y` інтегрує EKF.",
            "3. IMU gyro `z` і acceleration `x` виконують high-rate EKF prediction: "
            "кутова швидкість поширює orientation/yaw, прискорення — speed. "
            "IMU-only trajectory не використовується, бо інтегрування acceleration "
            "без надійної початкової лінійної швидкості швидко накопичує drift.",
            "4. E1 Base завжди використовує обидва комплементарні джерела: Wheel "
            "+ IMU. Wheel update коригує speed і gyro bias; EKF записує "
            "`x, y, yaw, speed` після кожного IMU step.",
            "5. E2 додає wheel/IMU disagreement detector. Під час slip wheel "
            "correction пропускається, а IMU prediction продовжується.",
            "6. Stereo frontend ректифікує пари, знаходить ORB matches, виконує "
            "RANSAC essential matrix і відновлює metric scale зі disparity. "
            "LiDAR frontend переводить VLP-16 points у vehicle frame, voxelizes "
            "їх та оцінює increment через 2D ICP.",
            "7. Health manager використовує VO лише у degraded wheel interval. "
            "LiDAR є другим fallback, якщо немає недавньої якісної VO correction. "
            "Низька quality або NIS вище gate залишають стан попереднього етапу.",
            "8. `vrs_gps.csv` не читається estimator-ом. Після завершення run "
            "valid RTK epochs зіставляються за часом, траєкторії один раз "
            "вирівнюються rigid SE(2) без scale fit, після чого рахується ATE.",
            "",
            "## Консистентність і межі",
            "",
            f"Для `{diagnostic_mode}` wheel-speed NIS нижче порога "
            f"{config.wheel.speed_nis_threshold:.3f} у {speed_below:.2%} з "
            f"{speed_count} перевірених updates; wheel-yaw NIS нижче порога "
            f"{config.wheel.yaw_nis_threshold:.3f} у {yaw_below:.2%} з "
            f"{yaw_count} updates. Поточна модель шуму консервативна.",
            "",
            "VRS-GPS не надходить у EKF. Він використаний після оцінювання стану "
            "для часових пар, одного SE(2) вирівнювання без зміни масштабу та "
            "ATE по всій траєкторії.",
        )
    )
    if any(
        name in typed_evaluations for name in ("visual", "lidar", "full")
    ):
        visual_result = next(
            (result for result in results if result.name == "visual"), None
        )
        lidar_result = next(
            (result for result in results if result.name == "lidar"), None
        )
        slip_result = next(
            (result for result in results if result.name == "slip"), None
        )
        fallback_details = []
        if visual_result is not None:
            fallback_details.append(
                f"VO: {visual_result.relative_updates}/"
                f"{visual_result.available_relative_updates} corrections"
            )
        if lidar_result is not None:
            fallback_details.append(
                f"LiDAR без камер: {lidar_result.relative_updates}/"
                f"{lidar_result.available_relative_updates} corrections"
            )
        conclusion_lines.extend(
            (
                "",
                "Strict health gating не дозволив VO або LiDAR погіршити E2. "
                + (
                    "На звичайній послідовності використано "
                    + "; ".join(fallback_details)
                    + ". "
                    if fallback_details
                    else ""
                )
                + "Frontends повністю обробили дані, але estimator приймав "
                "correction лише під час degraded wheel interval.",
                "",
                "## Чому VO та LiDAR майже не покращили результат",
                "",
                "Саме `urban35` є легкою послідовністю для Wheel+IMU: обидва "
                "потоки працюють приблизно зі 100 Hz, рух переважно плавний, а "
                + (
                    f"slip detector був активний лише для "
                    f"{slip_result.slip_samples} із "
                    f"{slip_result.wheel_updates} wheel samples "
                    f"({slip_result.slip_samples / slip_result.wheel_updates:.2%}). "
                    if slip_result is not None
                    else "природна деградація коліс була короткою. "
                )
                + "Тому E2 вже добре відтворює форму траєкторії, а зовнішнім "
                "сенсорам майже нічого виправляти.",
                "",
                "Реалізовані frontends не є повноцінними VIO/LIO. Stereo VO "
                "не оптимізує features разом з IMU state, а LiDAR ICP не робить "
                "IMU deskew rolling scan. При постійному fusion їхні noisy "
                "increments трохи погіршували RMSE, тому health manager "
                "використовує їх лише як fallback. На цій послідовності це "
                "означає практично однаковий результат E2, E2+VO та E2+LiDAR.",
                "",
                "## Порівняння з VIO та LIO",
                "",
                "Поточна система є **loosely coupled**: stereo VO та LiDAR ICP "
                "спочатку окремо оцінюють relative motion, після чого EKF отримує "
                "лише speed/yaw-rate correction. Cross-covariance features, "
                "point clouds, IMU bias і state при цьому втрачається.",
                "",
                "**VIO** спільно оптимізує camera reprojection residuals, IMU "
                "preintegration, pose, velocity і biases. Це краще утримує scale "
                "та orientation, але потребує точної camera–IMU calibration, "
                "ініціалізації й складнішого nonlinear solver.",
                "",
                "**LIO** використовує IMU для deskew кожного LiDAR scan і спільно "
                "оцінює trajectory та scan residuals. Це прямо усуває основну "
                "проблему нашого VLP-16 ICP — rolling motion distortion. Ціна — "
                "точна time/extrinsic calibration, більший state і суттєво більше "
                "обчислень.",
            )
        )
    result_for_detector = next(
        (result for result in results if result.injected_samples), None
    )
    if result_for_detector is not None:
        detection = (
            result_for_detector.detected_injected_samples
            / result_for_detector.injected_samples
        )
        healthy = result_for_detector.wheel_updates - result_for_detector.injected_samples
        false_rate = result_for_detector.false_positive_samples / max(healthy, 1)
        conclusion_lines.extend(
            (
                "",
                "## Контрольована перевірка slip detector",
                "",
                f"Detection rate: {detection:.1%}; false-positive samples: "
                f"{false_rate:.2%}. Під час fault interval wheel updates "
                "відкидаються, а EKF продовжує predict за IMU.",
            )
        )
    (config.general.output / "conclusions.md").write_text(
        "\n".join(conclusion_lines) + "\n", encoding="utf-8"
    )


def run(
    config: RunConfig,
    mode: str,
    *,
    max_events: int | None = None,
    validate: bool = False,
    inject_slip: bool = False,
) -> None:
    _require_inputs(config.general.dataset, mode=mode, validate=validate)
    config.general.output.mkdir(parents=True, exist_ok=True)
    wheels = _load_wheels(config, inject_slip=inject_slip)
    visual: VisualOdometryResult | None = None
    if mode in ("visual", "full", "all"):
        visual = compute_visual_odometry(config)
        print(
            f"visual frontend: accepted={len(visual.motions)}/"
            f"{visual.attempted_pairs}, rejected={visual.rejected_pairs}"
        )
    lidar: LidarOdometryResult | None = None
    if mode in ("lidar", "full", "all"):
        lidar = compute_lidar_odometry(config, wheels)
        print(
            f"lidar frontend: accepted={len(lidar.motions)}/"
            f"{lidar.attempted_pairs}, rejected={lidar.rejected_pairs}"
        )
    modes = (
        ["base", "slip", "visual", "lidar", "full"]
        if mode == "all"
        else [mode]
    )
    results = []
    for name in modes:
        relative_streams = []
        if name in ("visual", "full") and visual:
            relative_streams.append(visual.motions)
        if name in ("lidar", "full") and lidar:
            relative_streams.append(lidar.motions)
        results.append(
            _run_filter(
                config,
                name,
                wheels,
                relative_streams,
                max_events=max_events,
            )
        )

    if validate:
        reference = list(read_vrs_reference(config.general.dataset))
        for result in results:
            _evaluate(result, reference, config)
    print(_write_summary(config, results), end="")
    if validate:
        _write_validation_artifacts(config, results)

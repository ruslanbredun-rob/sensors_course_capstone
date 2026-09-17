"""Timestamp-driven wheel + IMU prototype and state export."""

from __future__ import annotations

import csv
from pathlib import Path

from .config import RunConfig
from .dataset import read_encoders, read_imu
from .ekf import VehicleEKF
from .models import ImuSample
from .synchronization import ordered_events
from .wheel_odometry import wheel_measurements


def _require_inputs(dataset: Path) -> None:
    required = (
        dataset / "sensor_data" / "encoder.csv",
        dataset / "sensor_data" / "xsens_imu.csv",
        dataset / "calibration" / "EncoderParameter.txt",
        dataset / "calibration" / "Vehicle2IMU.txt",
    )
    missing = [path for path in required if not path.is_file()]
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
        raise ValueError(
            "This planar prototype expects the urban35 identity vehicle-to-IMU "
            "rotation; update the frame transform before using another sequence."
        )

def run(
    config: RunConfig,
    mode: str,
    *,
    max_events: int | None = None,
) -> None:
    if mode != "base":
        raise NotImplementedError(
            f"Mode {mode!r} belongs to a later phase; homework 18 implements 'base'."
        )
    _require_inputs(config.dataset)
    config.output.mkdir(parents=True, exist_ok=True)
    output_path = config.output / "estimated_state.csv"

    imu = read_imu(config.dataset)
    encoder = read_encoders(config.dataset)
    wheels = wheel_measurements(
        encoder, config.dataset / "calibration" / "EncoderParameter.txt"
    )
    estimator = VehicleEKF(config)
    imu_count = wheel_count = rejected_count = event_count = 0

    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                "timestamp_ns",
                "source",
                "x_m",
                "y_m",
                "yaw_rad",
                "speed_m_s",
                "wheel_nis",
                "wheel_accepted",
            )
        )
        for event in ordered_events(imu, wheels):
            if isinstance(event, ImuSample):
                estimate = estimator.predict(event)
                source = "imu"
                nis = accepted = ""
                imu_count += 1
            else:
                if estimator.timestamp_ns is None:
                    # An encoder event before the first IMU cannot be propagated.
                    continue
                estimate = estimator.update_wheel(event)
                source = "wheel"
                nis = estimator.last_wheel_nis
                accepted = estimator.last_wheel_accepted
                wheel_count += 1
                rejected_count += int(not accepted)
            writer.writerow(
                (
                    estimate.timestamp_ns,
                    source,
                    f"{estimate.x_m:.6f}",
                    f"{estimate.y_m:.6f}",
                    f"{estimate.yaw_rad:.8f}",
                    f"{estimate.speed_m_s:.6f}",
                    "" if nis == "" else f"{nis:.6f}",
                    accepted,
                )
            )
            event_count += 1
            if max_events is not None and event_count >= max_events:
                break

    if imu_count == 0 or wheel_count == 0:
        raise ValueError("Both IMU and wheel measurements are required")
    summary = (
        f"Base: IMU={imu_count}, wheel={wheel_count}, "
        f"wheel_rejected={rejected_count}\n"
        f"Final estimated state: t={estimate.timestamp_ns}, "
        f"x={estimate.x_m:.3f} m, y={estimate.y_m:.3f} m, "
        f"yaw={estimate.yaw_rad:.5f} rad, v={estimate.speed_m_s:.3f} m/s\n"
        f"State at every processed event: {output_path.name}\n"
        "Frame: local origin (0, 0), initial yaw=0.\n"
    )
    summary += "VRS validation: not implemented.\n"
    (config.output / "run_summary.txt").write_text(summary, encoding="utf-8")
    print(summary, end="")

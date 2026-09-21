"""Focused checks for the homework 18 timing and fusion contracts."""

from __future__ import annotations

import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

from src.camera.vio import preintegrate_imu, preintegrate_wheels
from src.common.config import load_config
from src.common.models import (
    EncoderSample,
    Estimate,
    ImuSample,
    PositionReference,
    RelativeMotion,
    WheelMeasurement,
)
from src.common.synchronization import ordered_events
from src.dataset.readers import read_vrs_reference
from src.evaluation.metrics import evaluate_position, evaluate_rtk_segments
from src.fusion.ekf import VehicleEKF
from src.imu.reader import read_imu
from src.wheel.odometry import wheel_measurements


class PrototypeTests(unittest.TestCase):
    def test_config_is_grouped_by_subsystem_and_accepts_path_overrides(self) -> None:
        config = load_config(
            Path("config/default.json"),
            dataset=Path("/tmp/test-dataset"),
            output=Path("/tmp/test-output"),
        )
        self.assertEqual(config.general.dataset, Path("/tmp/test-dataset"))
        self.assertEqual(config.general.output, Path("/tmp/test-output"))
        self.assertEqual(config.evaluation.reference_fix_state, 4)
        self.assertGreater(config.imu.gyro_std_rad_s, 0.0)
        self.assertGreater(config.wheel.speed_nis_threshold, 0.0)
        self.assertGreater(config.visual_odometry.min_matches, 0)
        self.assertLessEqual(config.visual_odometry.min_fusion_coverage, 1.0)
        self.assertGreater(config.vio.window_size, 1)
        self.assertGreaterEqual(config.vio.max_tracks, config.vio.min_tracks)

    def test_vrs_reader_uses_utm_and_fix_state(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            sensor_dir = Path(directory) / "sensor_data"
            sensor_dir.mkdir()
            row = ["0"] * 18
            row[0], row[3], row[4], row[6] = "1000000000", "325285.5", "4150987.25", "4"
            (sensor_dir / "vrs_gps.csv").write_text(
                ",".join(row) + "\n", encoding="utf-8"
            )
            sample = next(read_vrs_reference(Path(directory)))
            self.assertEqual((sample.x_m, sample.y_m, sample.fix_state), (325285.5, 4150987.25, 4))

    def test_se2_validation_excludes_float_fixes_and_never_fits_scale(self) -> None:
        xy = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2)]
        states = [
            Estimate(index * 1_000_000_000, x, y, 0.0, 1.0)
            for index, (x, y) in enumerate(xy)
        ]
        reference = [
            PositionReference(
                index * 1_000_000_000 + 2_000_000,
                10.0 - y,
                20.0 + x,
                5 if index == 2 else 4,
            )
            for index, (x, y) in enumerate(xy)
        ]
        result = evaluate_position(
            states, reference, valid_fix_state=4, tolerance_ns=5_000_000
        )
        self.assertEqual((result.valid_fix_epochs, result.matched_epochs), (4, 4))
        self.assertLess(result.rmse_m, 1e-10)
        self.assertLess(result.start_error_m[0], 1e-10)
        self.assertLess(max(result.start_error_m), 1e-10)
        self.assertGreater(result.start_alignment_baseline_m, 0.0)
        scaled = [
            PositionReference(
                sample.timestamp_ns,
                10 + 2 * (sample.x_m - 10),
                20 + 2 * (sample.y_m - 20),
                sample.fix_state,
            )
            for sample in reference
        ]
        scaled_result = evaluate_position(
            states, scaled, valid_fix_state=4, tolerance_ns=5_000_000
        )
        self.assertGreater(scaled_result.rmse_m, 0.5)
        segmented = replace(
            result,
            timestamp_ns=np.array((0, 1_000_000_000, 10_000_000_000, 11_000_000_000)),
        )
        segments = evaluate_rtk_segments(
            segmented, max_gap_ns=1_500_000_000, min_epochs=2
        )
        self.assertEqual([segment.epochs for segment in segments], [2, 2])

    def test_validation_interpolates_low_rate_estimates(self) -> None:
        states = [
            Estimate(timestamp_ms * 1_000_000, timestamp_ms / 100.0, 0.0, 0.0, 10.0)
            for timestamp_ms in (0, 200, 400, 600)
        ]
        reference = [
            PositionReference(
                timestamp_ms * 1_000_000,
                timestamp_ms / 100.0,
                0.0,
                4,
            )
            for timestamp_ms in (100, 300, 500)
        ]
        result = evaluate_position(
            states,
            reference,
            valid_fix_state=4,
            tolerance_ns=50_000_000,
        )
        self.assertEqual(result.matched_epochs, 3)
        self.assertLess(result.rmse_m, 1e-10)

    def test_imu_columns_are_gyro_z_and_accel_x(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            sensor_dir = Path(directory) / "sensor_data"
            sensor_dir.mkdir()
            values = [0.0] * 17
            values[0] = 1000000000
            values[7] = 99.0  # Euler yaw must not drive the filter.
            values[10] = 0.2
            values[11] = 1.5
            (sensor_dir / "xsens_imu.csv").write_text(
                ",".join(map(str, values)) + "\n", encoding="utf-8"
            )
            sample = next(read_imu(Path(directory)))
            self.assertEqual(sample.yaw_rate_rad_s, 0.2)
            self.assertEqual(sample.forward_accel_m_s2, 1.5)

    def test_imu_preintegration_uses_camera_interval_boundaries(self) -> None:
        samples = [
            ImuSample(0, 0.1, 1.0),
            ImuSample(1_000_000_000, 0.2, 3.0),
            ImuSample(2_000_000_000, 0.3, 5.0),
        ]
        result = preintegrate_imu(
            samples,
            [sample.timestamp_ns for sample in samples],
            500_000_000,
            1_500_000_000,
        )
        self.assertAlmostEqual(result.dt_s, 1.0)
        self.assertAlmostEqual(result.mean_gyro_rad_s, 0.15)
        self.assertAlmostEqual(result.mean_accel_m_s2, 2.0)

    def test_wheel_preintegration_uses_camera_interval_boundaries(self) -> None:
        samples = [
            WheelMeasurement(0, 2.0, 0.1),
            WheelMeasurement(1_000_000_000, 4.0, 0.2),
            WheelMeasurement(2_000_000_000, 6.0, 0.3),
        ]
        result = preintegrate_wheels(
            samples,
            [sample.timestamp_ns for sample in samples],
            500_000_000,
            1_500_000_000,
        )
        self.assertAlmostEqual(result.dt_s, 1.0)
        self.assertAlmostEqual(result.mean_speed_m_s, 4.0)
        self.assertAlmostEqual(result.mean_yaw_rate_rad_s, 0.2)

    def test_wheel_speed_uses_calibration_and_actual_dt(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "EncoderParameter.txt"
            path.write_text(
                "Encoder resolution: 100\n"
                "Encoder left wheel diameter: 1\n"
                "Encoder right wheel diameter: 1\n"
                "Encoder wheel base: 2\n",
                encoding="utf-8",
            )
            samples = [
                EncoderSample(1000000000, 0, 0),
                EncoderSample(2000000000, 100, 100),
            ]
            measurement = next(wheel_measurements(samples, path))
            self.assertAlmostEqual(measurement.speed_m_s, 3.141592653589793)
            self.assertEqual(measurement.yaw_rate_rad_s, 0.0)

    def test_wheel_speed_preserves_reverse_motion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "EncoderParameter.txt"
            path.write_text(
                "Encoder resolution: 100\n"
                "Encoder left wheel diameter: 1\n"
                "Encoder right wheel diameter: 1\n"
                "Encoder wheel base: 2\n",
                encoding="utf-8",
            )
            samples = [
                EncoderSample(1_000_000_000, 100, 100),
                EncoderSample(2_000_000_000, 50, 50),
            ]
            measurement = next(wheel_measurements(samples, path))
            self.assertAlmostEqual(measurement.speed_m_s, -math.pi / 2)
            self.assertEqual(measurement.yaw_rate_rad_s, 0.0)

    def test_events_are_sorted_and_wheel_update_changes_estimate(self) -> None:
        events = list(
            ordered_events(
                [ImuSample(0, 0.0, 0.0), ImuSample(20000000, 0.0, 0.0)],
                [WheelMeasurement(10000000, 10.0, 0.0)],
            )
        )
        self.assertEqual([event.timestamp_ns for event in events], [0, 10000000, 20000000])
        config = load_config(Path("config/default.json"))
        filter_ = VehicleEKF(config)
        filter_.predict(events[0])
        estimate = filter_.update_wheel(events[1])
        self.assertGreater(estimate.speed_m_s, 9.0)
        self.assertTrue(filter_.last_wheel_accepted)
        self.assertEqual(filter_.predict(events[2]).timestamp_ns, 20000000)

    def test_relative_motion_updates_speed_and_gyro_bias(self) -> None:
        config = load_config(Path("config/default.json"))
        filter_ = VehicleEKF(config)
        filter_.predict(ImuSample(0, 0.2, 0.0))
        result = filter_.update_relative_motion(
            RelativeMotion(
                timestamp_ns=100_000_000,
                dt_s=0.1,
                dx_m=1.0,
                dy_m=0.0,
                dyaw_rad=0.01,
                source="test",
                translation_std_m=0.05,
                yaw_std_rad=0.005,
            )
        )
        self.assertGreater(result.speed_m_s, 5.0)
        self.assertTrue(filter_.last_relative_speed_accepted)
        self.assertTrue(filter_.last_relative_yaw_accepted)
        self.assertGreater(filter_.x[4], 0.0)

    def test_relative_pose_updates_position_and_yaw(self) -> None:
        config = load_config(Path("config/default.json"))
        filter_ = VehicleEKF(config)
        filter_.predict(ImuSample(0, 0.0, 0.0))
        filter_.store_relative_pose_anchor("test", 0)
        result = filter_.update_relative_pose(
            RelativeMotion(
                timestamp_ns=100_000_000,
                dt_s=0.1,
                dx_m=1.0,
                dy_m=0.2,
                dyaw_rad=0.05,
                source="test",
                translation_std_m=0.05,
                yaw_std_rad=0.01,
            )
        )
        self.assertTrue(filter_.last_relative_pose_accepted)
        self.assertGreater(result.x_m, 0.9)
        self.assertGreater(result.yaw_rad, 0.01)

if __name__ == "__main__":
    unittest.main()

"""Focused checks for the homework 18 timing and fusion contracts."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from src.config import load_config
from src.dataset import read_imu, read_vrs_reference
from src.ekf import VehicleEKF
from src.evaluation import evaluate_position
from src.lidar_odometry import rigid_fit_2d
from src.models import (
    EncoderSample,
    Estimate,
    ImuSample,
    PositionReference,
    RelativeMotion,
    WheelMeasurement,
)
from src.slip_detection import WheelSlipDetector
from src.synchronization import ordered_events
from src.wheel_odometry import wheel_measurements


class PrototypeTests(unittest.TestCase):
    def test_slip_detector_debounces_disagreement(self) -> None:
        detector = WheelSlipDetector(
            yaw_threshold_rad_s=0.3,
            accel_threshold_m_s2=2.0,
            enter_count=2,
            exit_count=2,
        )
        imu = ImuSample(0, 0.0, 0.0)
        bad = WheelMeasurement(0, 10.0, 1.0)
        good = WheelMeasurement(0, 10.0, 0.0)
        self.assertFalse(detector.update(bad, imu, 0.0).active)
        self.assertTrue(detector.update(bad, imu, 0.0).active)
        self.assertTrue(detector.update(good, imu, 0.0).active)
        self.assertFalse(detector.update(good, imu, 0.0).active)

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

    def test_lidar_rigid_fit_recovers_planar_transform(self) -> None:
        import numpy as np

        source = np.array(((0.0, 0.0), (2.0, 0.0), (0.0, 1.0), (2.0, 2.0)))
        yaw = 0.2
        rotation_expected = np.array(
            ((np.cos(yaw), -np.sin(yaw)), (np.sin(yaw), np.cos(yaw)))
        )
        translation_expected = np.array((1.5, -0.4))
        target = source @ rotation_expected.T + translation_expected
        rotation, translation = rigid_fit_2d(source, target)
        np.testing.assert_allclose(rotation, rotation_expected, atol=1e-12)
        np.testing.assert_allclose(translation, translation_expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

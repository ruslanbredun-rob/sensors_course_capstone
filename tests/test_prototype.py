"""Focused checks for the homework 18 timing and fusion contracts."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from src.config import load_config
from src.dataset import read_imu
from src.ekf import VehicleEKF
from src.models import EncoderSample, ImuSample, WheelMeasurement
from src.synchronization import ordered_events
from src.wheel_odometry import wheel_measurements


class PrototypeTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

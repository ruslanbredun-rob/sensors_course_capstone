"""Planar wheel + IMU EKF for homework 18.

State is [x, y, yaw, speed, gyro_z_bias, accel_x_bias]. The world origin and
initial yaw are arbitrary for this GPS-denied prototype. Wheel speed is the
only measurement update; VRS and IMU Euler attitude are never fused.
"""

from __future__ import annotations

import math

import numpy as np

from .config import RunConfig
from .models import Estimate, ImuSample, RelativeMotion, WheelMeasurement


class VehicleEKF:
    def __init__(self, config: RunConfig) -> None:
        self.config = config
        self.x = np.zeros(6, dtype=float)
        self.P = np.diag([1.0, 1.0, 0.1, 100.0, 0.01, 1.0])
        self.timestamp_ns: int | None = None
        self._imu: ImuSample | None = None
        self.last_wheel_nis: float | None = None
        self.last_wheel_accepted: bool | None = None

    def _estimate(self) -> Estimate:
        assert self.timestamp_ns is not None
        return Estimate(
            self.timestamp_ns,
            float(self.x[0]),
            float(self.x[1]),
            float(self.x[2]),
            float(self.x[3]),
        )

    def _advance(self, timestamp_ns: int) -> None:
        if self.timestamp_ns is None or self._imu is None:
            raise ValueError("An IMU sample must initialize the EKF")
        dt = (timestamp_ns - self.timestamp_ns) * 1e-9
        if dt < 0:
            raise ValueError("EKF received an event older than its current state")
        if dt == 0:
            return
        if dt > self.config.max_dt_s:
            raise ValueError(f"IMU gap {dt:.3f} s exceeds max_dt_s")

        px, py, yaw, speed, gyro_bias, accel_bias = self.x
        yaw_rate = self._imu.yaw_rate_rad_s - gyro_bias
        accel = self._imu.forward_accel_m_s2 - accel_bias
        yaw_mid = yaw + 0.5 * yaw_rate * dt
        speed_mid = speed + 0.5 * accel * dt
        cosine, sine = math.cos(yaw_mid), math.sin(yaw_mid)

        self.x[0] = px + speed_mid * cosine * dt
        self.x[1] = py + speed_mid * sine * dt
        self.x[2] = math.atan2(math.sin(yaw + yaw_rate * dt), math.cos(yaw + yaw_rate * dt))
        self.x[3] = speed + accel * dt

        F = np.eye(6)
        F[0, 2] = -speed_mid * sine * dt
        F[0, 3] = cosine * dt
        F[0, 4] = 0.5 * speed_mid * sine * dt * dt
        F[0, 5] = -0.5 * cosine * dt * dt
        F[1, 2] = speed_mid * cosine * dt
        F[1, 3] = sine * dt
        F[1, 4] = -0.5 * speed_mid * cosine * dt * dt
        F[1, 5] = -0.5 * sine * dt * dt
        F[2, 4] = -dt
        F[3, 5] = -dt

        G = np.zeros((6, 4))
        G[0, 0] = -0.5 * speed_mid * sine * dt * dt
        G[1, 0] = 0.5 * speed_mid * cosine * dt * dt
        G[2, 0] = dt
        G[0, 1] = 0.5 * cosine * dt * dt
        G[1, 1] = 0.5 * sine * dt * dt
        G[3, 1] = dt
        G[4, 2] = math.sqrt(dt)
        G[5, 3] = math.sqrt(dt)
        noise = np.array(
            [
                self.config.gyro_std_rad_s**2,
                self.config.accel_std_m_s2**2,
                self.config.gyro_bias_walk_rad_s_sqrt_s**2,
                self.config.accel_bias_walk_m_s2_sqrt_s**2,
            ]
        )
        self.P = F @ self.P @ F.T + (G * noise) @ G.T
        self.P = 0.5 * (self.P + self.P.T)
        self.timestamp_ns = timestamp_ns

    def predict(self, sample: ImuSample) -> Estimate:
        """Propagate with the previous IMU reading, then hold this reading."""
        if self.timestamp_ns is None:
            self.timestamp_ns = sample.timestamp_ns
        else:
            self._advance(sample.timestamp_ns)
        self._imu = sample
        return self._estimate()

    def update_wheel(self, measurement: WheelMeasurement) -> Estimate:
        """Scalar wheel-speed correction with 95% chi-square NIS gate."""
        self._advance(measurement.timestamp_ns)
        residual = measurement.speed_m_s - self.x[3]
        variance = self.P[3, 3] + self.config.wheel_speed_std_m_s**2
        nis = residual * residual / variance
        self.last_wheel_nis = float(nis)
        self.last_wheel_accepted = nis <= self.config.wheel_nis_threshold
        if self.last_wheel_accepted:
            gain = self.P[:, 3] / variance
            self.x += gain * residual
            h = np.zeros(6)
            h[3] = 1.0
            identity_minus_kh = np.eye(6) - np.outer(gain, h)
            # Joseph form keeps P symmetric and positive under repeated updates.
            self.P = (
                identity_minus_kh @ self.P @ identity_minus_kh.T
                + np.outer(gain, gain) * self.config.wheel_speed_std_m_s**2
            )
            self.P = 0.5 * (self.P + self.P.T)
        return self._estimate()

    def update_relative_motion(self, measurement: RelativeMotion) -> Estimate:
        """Reserved for VO/LiDAR in later phases."""
        raise NotImplementedError("VO/LiDAR updates are outside homework 18")

"""Planar multi-sensor EKF for homework 18 and 19.

State is [x, y, yaw, speed, gyro_z_bias, accel_x_bias]. The world origin and
initial yaw are arbitrary. Wheel and stereo VO provide local
updates; VRS and IMU Euler attitude are never fused.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.common.config import RunConfig
from src.common.models import (
    Estimate,
    GpsMeasurement,
    ImuSample,
    RelativeMotion,
    WheelMeasurement,
)


@dataclass
class _RelativePoseAnchor:
    timestamp_ns: int
    pose: np.ndarray
    covariance: np.ndarray
    cross_covariance: np.ndarray


class VehicleEKF:
    def __init__(self, config: RunConfig) -> None:
        self.config = config
        self.x = np.zeros(6, dtype=float)
        self.P = np.diag([1.0, 1.0, 0.1, 100.0, 0.01, 1.0])
        self.timestamp_ns: int | None = None
        self._imu: ImuSample | None = None
        self.last_wheel_nis: float | None = None
        self.last_wheel_accepted: bool | None = None
        self.last_wheel_yaw_nis: float | None = None
        self.last_wheel_yaw_accepted: bool | None = None
        self.last_relative_source: str | None = None
        self.last_relative_speed_nis: float | None = None
        self.last_relative_speed_accepted: bool | None = None
        self.last_relative_yaw_nis: float | None = None
        self.last_relative_yaw_accepted: bool | None = None
        self.last_relative_pose_nis: float | None = None
        self.last_relative_pose_accepted: bool | None = None
        self.last_relative_pose_covariance_scale: float | None = None
        self.last_gps_nis: float | None = None
        self.last_gps_accepted: bool | None = None
        self.last_gps_covariance_scale: float | None = None
        self.last_gps_update_kind: str | None = None
        self._gps_anchor_global: np.ndarray | None = None
        self._gps_anchor_local: np.ndarray | None = None
        self._gps_global_to_local_yaw: float | None = None
        self._relative_pose_anchors: dict[str, _RelativePoseAnchor] = {}

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
        if dt > self.config.general.max_dt_s:
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
                self.config.imu.gyro_std_rad_s**2,
                self.config.imu.accel_std_m_s2**2,
                self.config.imu.gyro_bias_walk_rad_s_sqrt_s**2,
                self.config.imu.accel_bias_walk_m_s2_sqrt_s**2,
            ]
        )
        self.P = F @ self.P @ F.T + (G * noise) @ G.T
        for anchor in self._relative_pose_anchors.values():
            anchor.cross_covariance = F @ anchor.cross_covariance
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

    @property
    def current_imu(self) -> ImuSample | None:
        return self._imu

    @property
    def imu_biases(self) -> tuple[float, float]:
        """Return the current gyro-z and forward-acceleration bias estimates."""
        return float(self.x[4]), float(self.x[5])

    def _scalar_update(
        self,
        residual: float,
        h: np.ndarray,
        measurement_variance: float,
        nis_threshold: float,
    ) -> tuple[float, bool]:
        variance = float(h @ self.P @ h + measurement_variance)
        nis = residual * residual / variance
        accepted = nis <= nis_threshold
        if accepted:
            gain = self.P @ h / variance
            self.x += gain * residual
            identity_minus_kh = np.eye(6) - np.outer(gain, h)
            self.P = (
                identity_minus_kh @ self.P @ identity_minus_kh.T
                + np.outer(gain, gain) * measurement_variance
            )
            self.P = 0.5 * (self.P + self.P.T)
            for anchor in self._relative_pose_anchors.values():
                anchor.cross_covariance = (
                    identity_minus_kh @ anchor.cross_covariance
                )
        return float(nis), accepted

    def _vector_update(
        self,
        residual: np.ndarray,
        h: np.ndarray,
        measurement_covariance: np.ndarray,
        nis_threshold: float,
    ) -> tuple[float, bool]:
        innovation_covariance = h @ self.P @ h.T + measurement_covariance
        nis = float(
            residual.T @ np.linalg.solve(innovation_covariance, residual)
        )
        accepted = nis <= nis_threshold
        if accepted:
            cross_covariance = self.P @ h.T
            gain = np.linalg.solve(
                innovation_covariance.T, cross_covariance.T
            ).T
            self.x += gain @ residual
            self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
            identity_minus_kh = np.eye(6) - gain @ h
            self.P = (
                identity_minus_kh @ self.P @ identity_minus_kh.T
                + gain @ measurement_covariance @ gain.T
            )
            self.P = 0.5 * (self.P + self.P.T)
            for anchor in self._relative_pose_anchors.values():
                anchor.cross_covariance = (
                    identity_minus_kh @ anchor.cross_covariance
                )
        return nis, accepted

    def update_wheel(
        self,
        measurement: WheelMeasurement,
        *,
        use_yaw_rate: bool = False,
    ) -> Estimate:
        """Correct speed and gyro bias with fixed calibrated noise."""
        self._advance(measurement.timestamp_ns)
        self.last_wheel_yaw_nis = None
        self.last_wheel_yaw_accepted = None
        speed_h = np.zeros(6)
        speed_h[3] = 1.0
        self.last_wheel_nis, self.last_wheel_accepted = self._scalar_update(
            measurement.speed_m_s - self.x[3],
            speed_h,
            self.config.wheel.speed_std_m_s**2,
            self.config.wheel.speed_nis_threshold,
        )
        if use_yaw_rate:
            assert self._imu is not None
            yaw_h = np.zeros(6)
            yaw_h[4] = -1.0
            predicted_yaw_rate = self._imu.yaw_rate_rad_s - self.x[4]
            (
                self.last_wheel_yaw_nis,
                self.last_wheel_yaw_accepted,
            ) = self._scalar_update(
                measurement.yaw_rate_rad_s - predicted_yaw_rate,
                yaw_h,
                self.config.wheel.yaw_rate_std_rad_s**2,
                self.config.wheel.yaw_nis_threshold,
            )
        return self._estimate()

    def update_gps(
        self,
        measurement: GpsMeasurement,
        antenna_offset_xy_m: np.ndarray,
    ) -> Estimate:
        """Fuse a commercial GPS antenna position with lever-arm compensation."""
        self._advance(measurement.timestamp_ns)
        if antenna_offset_xy_m.shape != (2,):
            raise ValueError("GPS antenna offset must contain x and y")
        yaw = float(self.x[2])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        lever_x, lever_y = map(float, antenna_offset_xy_m)
        antenna = np.array(
            (
                self.x[0] + cosine * lever_x - sine * lever_y,
                self.x[1] + sine * lever_x + cosine * lever_y,
            )
        )
        global_position = np.array(
            (measurement.easting_m, measurement.northing_m)
        )
        if self._gps_anchor_global is None:
            self._gps_anchor_global = global_position
            self._gps_anchor_local = antenna
            self.last_gps_nis = None
            self.last_gps_accepted = None
            self.last_gps_covariance_scale = None
            self.last_gps_update_kind = "initializing"
            return self._estimate()
        assert self._gps_anchor_local is not None
        global_delta = global_position - self._gps_anchor_global
        local_delta = antenna - self._gps_anchor_local
        if self._gps_global_to_local_yaw is None:
            if (
                np.linalg.norm(global_delta)
                < self.config.gps.initial_alignment_distance_m
                or np.linalg.norm(local_delta)
                < 0.5 * self.config.gps.initial_alignment_distance_m
            ):
                self.last_gps_nis = None
                self.last_gps_accepted = None
                self.last_gps_covariance_scale = None
                self.last_gps_update_kind = "initializing"
                return self._estimate()
            self._gps_global_to_local_yaw = math.atan2(
                local_delta[1], local_delta[0]
            ) - math.atan2(global_delta[1], global_delta[0])
        alignment_cosine = math.cos(self._gps_global_to_local_yaw)
        alignment_sine = math.sin(self._gps_global_to_local_yaw)
        global_to_local = np.array(
            (
                (alignment_cosine, -alignment_sine),
                (alignment_sine, alignment_cosine),
            )
        )
        measured_local = self._gps_anchor_local + global_to_local @ global_delta
        residual = measured_local - antenna
        h = np.zeros((2, 6))
        h[0, 0] = h[1, 1] = 1.0
        h[0, 2] = -sine * lever_x - cosine * lever_y
        h[1, 2] = cosine * lever_x - sine * lever_y
        minimum_variance = self.config.gps.min_position_std_m**2
        covariance_xx = max(measurement.covariance_xx_m2, minimum_variance)
        covariance_yy = max(measurement.covariance_yy_m2, minimum_variance)
        covariance_xy_limit = 0.99 * math.sqrt(covariance_xx * covariance_yy)
        covariance_xy = float(
            np.clip(
                measurement.covariance_xy_m2,
                -covariance_xy_limit,
                covariance_xy_limit,
            )
        )
        covariance = np.array(
            ((covariance_xx, covariance_xy), (covariance_xy, covariance_yy))
        )
        covariance = global_to_local @ covariance @ global_to_local.T
        innovation = h @ self.P @ h.T + covariance
        raw_nis = float(residual.T @ np.linalg.solve(innovation, residual))
        required_scale = max(
            1.0, raw_nis / self.config.gps.position_nis_threshold
        )
        self.last_gps_covariance_scale = required_scale
        if required_scale > self.config.gps.max_covariance_scale:
            self.last_gps_nis = raw_nis
            self.last_gps_accepted = False
            self.last_gps_update_kind = "rejected"
            return self._estimate()
        self.last_gps_nis, self.last_gps_accepted = self._vector_update(
            residual,
            h,
            covariance * required_scale,
            self.config.gps.position_nis_threshold * (1.0 + 1e-9),
        )
        self.last_gps_update_kind = (
            "position" if self.last_gps_accepted else "rejected"
        )
        return self._estimate()

    def update_relative_motion(self, measurement: RelativeMotion) -> Estimate:
        """Fuse body-forward speed and yaw rate from relative odometry."""
        if measurement.dt_s <= 0.0:
            raise ValueError("Relative-motion dt_s must be positive")
        self._advance(measurement.timestamp_ns)
        self.last_relative_source = measurement.source
        self.last_relative_pose_nis = None
        self.last_relative_pose_accepted = None
        self.last_relative_pose_covariance_scale = None
        quality = max(0.05, min(1.0, measurement.quality))

        speed_h = np.zeros(6)
        speed_h[3] = 1.0
        speed_measurement = measurement.dx_m / measurement.dt_s
        speed_std = measurement.translation_std_m / (measurement.dt_s * quality)
        (
            self.last_relative_speed_nis,
            self.last_relative_speed_accepted,
        ) = self._scalar_update(
            speed_measurement - self.x[3],
            speed_h,
            speed_std**2,
            self.config.fusion.relative_speed_nis_threshold,
        )

        assert self._imu is not None
        yaw_h = np.zeros(6)
        yaw_h[4] = -1.0
        predicted_yaw_rate = self._imu.yaw_rate_rad_s - self.x[4]
        yaw_rate_measurement = measurement.dyaw_rad / measurement.dt_s
        yaw_std = measurement.yaw_std_rad / (measurement.dt_s * quality)
        (
            self.last_relative_yaw_nis,
            self.last_relative_yaw_accepted,
        ) = self._scalar_update(
            yaw_rate_measurement - predicted_yaw_rate,
            yaw_h,
            yaw_std**2,
            self.config.fusion.relative_yaw_nis_threshold,
        )
        return self._estimate()

    def store_relative_pose_anchor(self, source: str, timestamp_ns: int) -> Estimate:
        """Clone the current pose and its correlation for the next VO edge."""
        if not source:
            raise ValueError("Relative-pose source must be non-empty")
        self._advance(timestamp_ns)
        self._relative_pose_anchors[source] = _RelativePoseAnchor(
            timestamp_ns=timestamp_ns,
            pose=self.x[:3].copy(),
            covariance=self.P[:3, :3].copy(),
            cross_covariance=self.P[:, :3].copy(),
        )
        return self._estimate()

    def update_relative_pose(self, measurement: RelativeMotion) -> Estimate:
        """Fuse body-frame ``dx, dy, dyaw`` against a correlated pose clone."""
        if measurement.dt_s <= 0.0:
            raise ValueError("Relative-motion dt_s must be positive")
        anchor = self._relative_pose_anchors.get(measurement.source)
        if anchor is None:
            raise ValueError(f"No relative-pose anchor for {measurement.source}")
        expected_anchor_ns = measurement.timestamp_ns - round(measurement.dt_s * 1e9)
        tolerance_ns = int(self.config.general.max_dt_s * 0.5e9)
        if abs(anchor.timestamp_ns - expected_anchor_ns) > tolerance_ns:
            raise ValueError(
                f"{measurement.source} anchor differs from frontend interval by "
                f"{abs(anchor.timestamp_ns - expected_anchor_ns) / 1e6:.1f} ms"
            )
        self._advance(measurement.timestamp_ns)
        self.last_relative_source = measurement.source
        self.last_relative_speed_nis = None
        self.last_relative_speed_accepted = None
        self.last_relative_yaw_nis = None
        self.last_relative_yaw_accepted = None
        self.last_relative_pose_covariance_scale = None
        quality = max(0.05, min(1.0, measurement.quality))

        anchor_x, anchor_y, anchor_yaw = anchor.pose
        cosine = math.cos(anchor_yaw)
        sine = math.sin(anchor_yaw)
        world_dx = self.x[0] - anchor_x
        world_dy = self.x[1] - anchor_y
        predicted = np.array(
            (
                cosine * world_dx + sine * world_dy,
                -sine * world_dx + cosine * world_dy,
                math.atan2(
                    math.sin(self.x[2] - anchor_yaw),
                    math.cos(self.x[2] - anchor_yaw),
                ),
            )
        )
        residual = np.array(
            (measurement.dx_m, measurement.dy_m, measurement.dyaw_rad)
        ) - predicted
        residual[2] = math.atan2(math.sin(residual[2]), math.cos(residual[2]))
        current_h = np.zeros((3, 6))
        current_h[0, 0], current_h[0, 1] = cosine, sine
        current_h[1, 0], current_h[1, 1] = -sine, cosine
        current_h[2, 2] = 1.0
        anchor_h = np.array(
            (
                (-cosine, -sine, predicted[1]),
                (sine, -cosine, -predicted[0]),
                (0.0, 0.0, -1.0),
            )
        )
        translation_std = measurement.translation_std_m / quality
        yaw_std = measurement.yaw_std_rad / quality
        covariance = np.diag(
            (translation_std**2, translation_std**2, yaw_std**2)
        )

        def innovation_covariance(measurement_covariance: np.ndarray) -> np.ndarray:
            cross = anchor.cross_covariance
            return (
                current_h @ self.P @ current_h.T
                + anchor_h @ anchor.covariance @ anchor_h.T
                + current_h @ cross @ anchor_h.T
                + anchor_h @ cross.T @ current_h.T
                + measurement_covariance
            )

        innovation = innovation_covariance(covariance)
        raw_nis = float(residual.T @ np.linalg.solve(innovation, residual))
        threshold = self.config.fusion.relative_pose_nis_threshold
        covariance_scale = max(1.0, raw_nis / threshold)
        max_scale = self.config.fusion.relative_pose_max_covariance_scale
        accepted = covariance_scale <= max_scale
        covariance_scale = min(covariance_scale, max_scale)
        adaptive_covariance = covariance * covariance_scale
        innovation = innovation_covariance(adaptive_covariance)
        nis = float(residual.T @ np.linalg.solve(innovation, residual))
        if accepted:
            state_innovation_cross = (
                self.P @ current_h.T
                + anchor.cross_covariance @ anchor_h.T
            )
            gain = np.linalg.solve(
                innovation.T, state_innovation_cross.T
            ).T
            self.x += gain @ residual
            self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))
            self.P -= gain @ innovation @ gain.T
            self.P = 0.5 * (self.P + self.P.T)
        self.last_relative_pose_nis = nis
        self.last_relative_pose_accepted = accepted
        self.last_relative_pose_covariance_scale = covariance_scale
        return self._estimate()

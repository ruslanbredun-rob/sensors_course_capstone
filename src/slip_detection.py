"""Debounced wheel-slip detection from wheel/IMU disagreement."""

from __future__ import annotations

from dataclasses import dataclass

from .models import ImuSample, WheelMeasurement


@dataclass(frozen=True)
class SlipDecision:
    active: bool
    yaw_residual_rad_s: float
    accel_residual_m_s2: float
    evidence: bool


class WheelSlipDetector:
    def __init__(
        self,
        *,
        yaw_threshold_rad_s: float,
        accel_threshold_m_s2: float,
        enter_count: int,
        exit_count: int,
    ) -> None:
        if min(yaw_threshold_rad_s, accel_threshold_m_s2, enter_count, exit_count) <= 0:
            raise ValueError("Slip detector parameters must be positive")
        self.yaw_threshold = yaw_threshold_rad_s
        self.accel_threshold = accel_threshold_m_s2
        self.enter_count = enter_count
        self.exit_count = exit_count
        self.active = False
        self._bad_count = 0
        self._good_count = 0
        self._previous_wheel: WheelMeasurement | None = None

    def update(
        self,
        wheel: WheelMeasurement,
        imu: ImuSample,
        gyro_bias_rad_s: float,
    ) -> SlipDecision:
        yaw_residual = wheel.yaw_rate_rad_s - (
            imu.yaw_rate_rad_s - gyro_bias_rad_s
        )
        accel_residual = 0.0
        if self._previous_wheel is not None:
            dt = (wheel.timestamp_ns - self._previous_wheel.timestamp_ns) * 1e-9
            if dt > 0:
                wheel_accel = (wheel.speed_m_s - self._previous_wheel.speed_m_s) / dt
                accel_residual = wheel_accel - imu.forward_accel_m_s2
        self._previous_wheel = wheel
        yaw_evidence = abs(yaw_residual) > self.yaw_threshold
        accel_evidence = abs(accel_residual) > self.accel_threshold
        evidence = yaw_evidence or accel_evidence
        if evidence:
            self._bad_count += 1
            self._good_count = 0
        else:
            self._good_count += 1
            self._bad_count = 0
        if not self.active and self._bad_count >= self.enter_count:
            self.active = True
        elif self.active and self._good_count >= self.exit_count:
            self.active = False
        return SlipDecision(self.active, yaw_residual, accel_residual, evidence)

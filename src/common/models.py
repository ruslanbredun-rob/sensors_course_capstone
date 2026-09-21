"""Internal contracts. All timestamps are nanoseconds; angles are radians."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EncoderSample:
    timestamp_ns: int
    left_count: int
    right_count: int


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    yaw_rate_rad_s: float
    forward_accel_m_s2: float


@dataclass(frozen=True)
class WheelMeasurement:
    timestamp_ns: int
    speed_m_s: float
    yaw_rate_rad_s: float
    left_speed_m_s: float = 0.0
    right_speed_m_s: float = 0.0


@dataclass(frozen=True)
class RelativeMotion:
    timestamp_ns: int
    dt_s: float
    dx_m: float
    dy_m: float
    dyaw_rad: float
    source: str
    translation_std_m: float = 1.0
    yaw_std_rad: float = 0.1
    quality: float = 1.0


@dataclass(frozen=True)
class RelativePoseEpoch:
    """Timestamp at which a VO/LO frontend starts its next pose increment."""

    timestamp_ns: int
    source: str


@dataclass(frozen=True)
class PositionReference:
    timestamp_ns: int
    x_m: float
    y_m: float
    fix_state: int


@dataclass(frozen=True)
class Estimate:
    timestamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    speed_m_s: float

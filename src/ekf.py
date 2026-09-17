"""2D EKF owns state, covariance and NIS measurement checks."""

from __future__ import annotations

from .models import Estimate, ImuSample, RelativeMotion, WheelMeasurement


class VehicleEKF:
    """Planned state: x, y, yaw, v, gyro bias and forward acceleration bias."""

    def predict(self, sample: ImuSample) -> Estimate:
        """Advance to the IMU timestamp and expose the current estimated state."""
        raise NotImplementedError("Implement process model, Jacobian, Q and dt checks")

    def update_wheel(self, measurement: WheelMeasurement) -> Estimate:
        """Apply wheel update and return the state to record at this timestamp."""
        raise NotImplementedError("Implement wheel measurement model and gating")

    def update_relative_motion(self, measurement: RelativeMotion) -> Estimate:
        """Future VO/LiDAR update; return the state at this timestamp."""
        raise NotImplementedError("Implement only after the base EKF is validated")

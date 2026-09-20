"""Small readers for the calibration formats shipped with Complex Urban."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def read_rigid_transform(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return the sensor-to-vehicle rotation and translation from a text file."""
    rotation: np.ndarray | None = None
    translation: np.ndarray | None = None
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("R:"):
            values = np.fromstring(line.removeprefix("R:"), sep=" ")
            if values.size == 9:
                rotation = values.reshape(3, 3)
        elif line.startswith("T:"):
            values = np.fromstring(line.removeprefix("T:"), sep=" ")
            if values.size == 3:
                translation = values
    if rotation is None or translation is None:
        raise ValueError(f"{path}: expected one 3x3 R and one 3-vector T")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=2e-3):
        raise ValueError(f"{path}: R is not orthonormal")
    return rotation, translation

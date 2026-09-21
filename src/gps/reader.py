"""Reader for the commercial GPS stream in Complex Urban Dataset."""

from __future__ import annotations

import math
from collections.abc import Iterator
from pathlib import Path

from src.common.models import GpsMeasurement
from src.dataset.readers import checked_rows


def wgs84_to_utm(latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
    """Convert WGS84 latitude/longitude to the point's UTM zone."""
    if not -80.0 <= latitude_deg <= 84.0:
        raise ValueError("UTM latitude is outside the supported range")
    zone = int((longitude_deg + 180.0) // 6.0) + 1
    longitude_origin_deg = (zone - 1) * 6 - 180 + 3
    latitude = math.radians(latitude_deg)
    longitude_delta = math.radians(longitude_deg - longitude_origin_deg)

    semi_major_m = 6_378_137.0
    eccentricity_sq = 0.00669437999014
    second_eccentricity_sq = eccentricity_sq / (1.0 - eccentricity_sq)
    scale = 0.9996
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    tangent_sq = math.tan(latitude) ** 2
    curvature = semi_major_m / math.sqrt(
        1.0 - eccentricity_sq * sin_latitude**2
    )
    c = second_eccentricity_sq * cos_latitude**2
    a = cos_latitude * longitude_delta
    e2, e4, e6 = eccentricity_sq, eccentricity_sq**2, eccentricity_sq**3
    meridian = semi_major_m * (
        (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0)
        * latitude
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0)
        * math.sin(2.0 * latitude)
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0)
        * math.sin(4.0 * latitude)
        - 35.0 * e6 / 3072.0 * math.sin(6.0 * latitude)
    )
    easting_m = 500_000.0 + scale * curvature * (
        a
        + (1.0 - tangent_sq + c) * a**3 / 6.0
        + (
            5.0
            - 18.0 * tangent_sq
            + tangent_sq**2
            + 72.0 * c
            - 58.0 * second_eccentricity_sq
        )
        * a**5
        / 120.0
    )
    northing_m = scale * (
        meridian
        + curvature
        * math.tan(latitude)
        * (
            a**2 / 2.0
            + (5.0 - tangent_sq + 9.0 * c + 4.0 * c**2) * a**4 / 24.0
            + (
                61.0
                - 58.0 * tangent_sq
                + tangent_sq**2
                + 600.0 * c
                - 330.0 * second_eccentricity_sq
            )
            * a**6
            / 720.0
        )
    )
    if latitude_deg < 0.0:
        northing_m += 10_000_000.0
    return easting_m, northing_m


def read_gps(dataset: Path) -> Iterator[GpsMeasurement]:
    """Read latitude/longitude and horizontal covariance from ``gps.csv``."""
    path = dataset / "sensor_data" / "gps.csv"
    previous_ns = -1
    for line_number, row in checked_rows(path, 13):
        try:
            timestamp_ns = int(row[0])
            latitude_deg, longitude_deg = float(row[1]), float(row[2])
            covariance_xx_m2 = float(row[4])
            covariance_xy_m2 = 0.5 * (float(row[5]) + float(row[7]))
            covariance_yy_m2 = float(row[8])
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: invalid GPS value") from exc
        if timestamp_ns <= previous_ns:
            raise ValueError(f"{path}:{line_number}: timestamp is not increasing")
        values = (
            latitude_deg,
            longitude_deg,
            covariance_xx_m2,
            covariance_xy_m2,
            covariance_yy_m2,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{line_number}: non-finite GPS value")
        easting_m, northing_m = wgs84_to_utm(latitude_deg, longitude_deg)
        previous_ns = timestamp_ns
        yield GpsMeasurement(
            timestamp_ns,
            easting_m,
            northing_m,
            covariance_xx_m2,
            covariance_xy_m2,
            covariance_yy_m2,
        )

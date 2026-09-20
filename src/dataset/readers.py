"""Streaming readers for the headerless Complex Urban files.

Column order: https://sites.google.com/view/complex-urban-dataset/format
"""

from __future__ import annotations

import csv
import math
from collections.abc import Iterator
from pathlib import Path

from src.common.models import EncoderSample, PositionReference


def checked_rows(path: Path, expected_columns: int) -> Iterator[tuple[int, list[str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if len(row) != expected_columns:
                raise ValueError(
                    f"{path}:{line_number}: expected {expected_columns} columns, "
                    f"got {len(row)}"
                )
            yield line_number, row


def read_encoders(dataset: Path) -> Iterator[EncoderSample]:
    """Read cumulative left/right pulse counts (columns 1/2)."""
    path = dataset / "sensor_data" / "encoder.csv"
    previous_ns = -1
    for line_number, row in checked_rows(path, 3):
        try:
            timestamp_ns, left, right = map(int, row)
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: invalid integer") from exc
        if timestamp_ns <= previous_ns:
            raise ValueError(f"{path}:{line_number}: timestamp is not increasing")
        previous_ns = timestamp_ns
        yield EncoderSample(timestamp_ns, left, right)


def read_vrs_reference(dataset: Path) -> Iterator[PositionReference]:
    """Read UTM easting/northing (columns 3/4) and fix state (column 6)."""
    path = dataset / "sensor_data" / "vrs_gps.csv"
    previous_ns = -1
    with path.open(newline="", encoding="utf-8") as stream:
        for line_number, row in enumerate(csv.reader(stream), start=1):
            if len(row) not in (17, 18):
                raise ValueError(
                    f"{path}:{line_number}: expected 17 or 18 columns, got {len(row)}"
                )
            try:
                timestamp_ns = int(row[0])
                easting_m = float(row[3])
                northing_m = float(row[4])
                fix_state = int(row[6])
            except ValueError as exc:
                raise ValueError(f"{path}:{line_number}: invalid VRS value") from exc
            if timestamp_ns <= previous_ns:
                raise ValueError(f"{path}:{line_number}: timestamp is not increasing")
            if not math.isfinite(easting_m) or not math.isfinite(northing_m):
                raise ValueError(f"{path}:{line_number}: non-finite VRS position")
            previous_ns = timestamp_ns
            yield PositionReference(timestamp_ns, easting_m, northing_m, fix_state)

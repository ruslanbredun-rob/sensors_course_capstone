"""Reference-only evaluation; this module cannot feed the estimator."""

from __future__ import annotations

from collections.abc import Iterable

from .models import Estimate, PositionReference


def position_rmse(
    estimates: Iterable[Estimate],
    reference: Iterable[PositionReference],
    *,
    valid_fix_state: int,
    tolerance_ns: int,
) -> float:
    """RMSE across all matched valid VRS epochs in a common coordinate frame."""
    raise NotImplementedError("Implement frame alignment and timestamp pairing")

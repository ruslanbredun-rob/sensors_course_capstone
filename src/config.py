"""Repository-relative configuration and CLI overrides."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class RunConfig:
    dataset: Path
    output: Path
    reference_fix_state: int
    reference_tolerance_ns: int


def load_config(
    config_path: Path, *, dataset: Path | None = None, output: Path | None = None
) -> RunConfig:
    with config_path.open(encoding="utf-8") as stream:
        values = json.load(stream)

    def resolve(value: str | Path) -> Path:
        path = Path(value).expanduser()
        return path.resolve() if path.is_absolute() else (PROJECT_ROOT / path).resolve()

    return RunConfig(
        dataset=resolve(dataset or values["dataset"]),
        output=resolve(output or values["output"]),
        reference_fix_state=int(values["reference_fix_state"]),
        reference_tolerance_ns=int(values["reference_tolerance_ms"] * 1_000_000),
    )

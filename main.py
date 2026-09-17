"""Run the Complex Urban wheel + IMU localization prototype."""

from __future__ import annotations

import argparse
from pathlib import Path

from src.config import load_config
from src.pipeline import run


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "config" / "default.json",
    )
    parser.add_argument("--dataset", type=Path, help="Override dataset directory")
    parser.add_argument("--output", type=Path, help="Override result directory")
    parser.add_argument(
        "--mode",
        choices=("base", "slip", "visual", "full", "all"),
        default="base",
    )
    parser.add_argument(
        "--max-events",
        type=int,
        help="Process only this many timestamp-ordered events (for inspection)",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Evaluate the output trajectory against valid VRS-GPS fixes",
    )
    args = parser.parse_args()
    if args.max_events is not None and args.max_events <= 0:
        parser.error("--max-events must be positive")
    config = load_config(args.config, dataset=args.dataset, output=args.output)
    run(config, args.mode, max_events=args.max_events, validate=args.validate)


if __name__ == "__main__":
    main()

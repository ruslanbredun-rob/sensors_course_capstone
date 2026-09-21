"""Run the Complex Urban localization experiments."""

from __future__ import annotations

import argparse
from pathlib import Path

from src.common.config import PROJECT_ROOT, load_config
from src.fusion.pipeline import run


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "config" / "default.json",
    )
    parser.add_argument("--dataset", type=Path, help="Override dataset directory")
    parser.add_argument("--output", type=Path, help="Override result directory")
    parser.add_argument(
        "--mode",
        choices=(
            "base",
            "visual",
            "vio",
            "gps",
            "gps_dropout",
            "gps_sparse",
            "all",
        ),
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
    parser.add_argument(
        "--reuse-frontends",
        action="store_true",
        help="Reuse compatible cached VO and VIO results from --output",
    )
    args = parser.parse_args()
    if args.max_events is not None and args.max_events <= 0:
        parser.error("--max-events must be positive")
    config = load_config(args.config, dataset=args.dataset, output=args.output)
    run(
        config,
        args.mode,
        max_events=args.max_events,
        validate=args.validate,
        reuse_frontends=args.reuse_frontends,
    )


if __name__ == "__main__":
    main()

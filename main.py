"""Entry point for the planned Complex Urban localization pipeline."""

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
    args = parser.parse_args()
    config = load_config(args.config, dataset=args.dataset, output=args.output)
    run(config, args.mode)


if __name__ == "__main__":
    main()

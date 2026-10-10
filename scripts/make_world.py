#!/usr/bin/env python3
"""Generate one seeded SkyScan world.

Usage:
    python scripts/make_world.py --seed 1
    python scripts/make_world.py --seed 2 --out-dir /tmp/worlds

Writes `<out-dir>/<seed>/` with `world.sdf`, `textures/`,
`mission_config_field.yaml` and `truth.json`.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from aero.core.errors import ConfigError  # noqa: E402 - needs REPO_ROOT on sys.path
from aero.sim.make_world import DEFAULT_WORLDS_DIR, generate_world  # noqa: E402
from aero.sim.world_params import load_world_params  # noqa: E402


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True, help="layout seed")
    parser.add_argument(
        "--out-dir", type=Path, default=DEFAULT_WORLDS_DIR, help="parent directory for <seed>/"
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=None,
        help="world params YAML (default sim/world_params.yaml)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Generate the world and print the paths written."""
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s %(message)s", stream=sys.stderr
    )
    try:
        params = load_world_params(args.params) if args.params else None
        generated = generate_world(args.seed, args.out_dir, params)
    except ConfigError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"world: {generated.world_path}")
    print(f"field: {generated.field_config_path}")
    print(f"truth: {generated.truth_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

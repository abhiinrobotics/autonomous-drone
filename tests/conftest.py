"""Shared pytest fixtures.

The `sitl` fixture starts PX4 + Gazebo per test, writes a run directory under
`logs/<timestamp>_<seed>_<pid>/`, and guarantees teardown of both processes even
when the test fails. It is function-scoped on purpose: a shared instance would
carry arm state, flight mode, parameters and position between tests and make
results order-dependent (`docs/rules/testing.md`).

Overrides come from the environment so that `scripts/batch.py` can drive
repeated runs:

    AERO_PX4_DIR     PX4-Autopilot root (default ~/PX4-Autopilot)
    AERO_SEED        world/augmentation seed recorded in meta.json (default 0)
    AERO_SIM_SPEED   PX4_SIM_SPEED_FACTOR (default: real time)
"""

from __future__ import annotations

import json
import os
from collections.abc import Iterator
from datetime import datetime, timezone
from pathlib import Path

import pytest

from aero.sim.px4_sitl import DEFAULT_PX4_DIR, Px4Sitl, SitlConfig, SitlError

REPO_ROOT = Path(__file__).resolve().parents[1]
# Budget for a `sitl`-marked test: simulator start-up plus the test's own work.
# Applied centrally so no test has to repeat a literal, and so the global
# `timeout` in pyproject.toml cannot kill SITL during bring-up.
SITL_TEST_TIMEOUT_S = 300


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """Give every `sitl`-marked test the SITL timeout unless it sets its own."""
    for item in items:
        if item.get_closest_marker("sitl") and not item.get_closest_marker("timeout"):
            item.add_marker(pytest.mark.timeout(SITL_TEST_TIMEOUT_S))


def _env_number(name: str, default: str, cast: type[int] | type[float]) -> int | float:
    """Read a numeric environment override, failing with a clear message.

    Args:
        name: Environment variable name.
        default: Value used when the variable is unset.
        cast: `int` or `float`.

    Returns:
        The parsed number.

    Raises:
        pytest.UsageError: The variable is set but not a number.
    """
    raw = os.environ.get(name, default)
    try:
        return cast(raw)
    except ValueError as error:
        raise pytest.UsageError(f"{name}={raw!r} is not a valid {cast.__name__}") from error


@pytest.fixture(scope="session")
def px4_dir() -> Path:
    """PX4-Autopilot checkout used by SITL tests."""
    return Path(os.environ.get("AERO_PX4_DIR", str(DEFAULT_PX4_DIR)))


@pytest.fixture(scope="session")
def sim_seed() -> int:
    """Seed recorded with the run (world + vision augmentation)."""
    return int(_env_number("AERO_SEED", "0", int))


@pytest.fixture(scope="session")
def sim_speed_factor() -> float | None:
    """`PX4_SIM_SPEED_FACTOR` override, or None for real time."""
    if "AERO_SIM_SPEED" not in os.environ:
        return None
    return float(_env_number("AERO_SIM_SPEED", "1.0", float))


@pytest.fixture
def sitl_run_dir(sim_seed: int) -> Path:
    """Fresh directory for this test's SITL logs.

    The pid is part of the name so two sessions started in the same second with
    the same seed cannot overwrite each other.
    """
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = REPO_ROOT / "logs" / f"{stamp}_{sim_seed}_{os.getpid()}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


@pytest.fixture
def sitl(
    px4_dir: Path,
    sim_seed: int,
    sim_speed_factor: float | None,
    sitl_run_dir: Path,
) -> Iterator[Px4Sitl]:
    """Running, ready PX4 SITL + gz instance; torn down after the test.

    Skips (rather than fails) when PX4 is not built on this machine, so a fresh
    clone reports "environment missing" instead of a red suite.
    """
    config = SitlConfig(px4_dir=px4_dir, speed_factor=sim_speed_factor)
    try:
        runner = Px4Sitl(config, sitl_run_dir / "px4.log")
    except SitlError as error:
        pytest.skip(f"PX4 SITL unavailable: {error}")
    meta = {
        "seed": sim_seed,
        "model": config.model,
        "world": config.world,
        "speed_factor": config.speed_factor,
        "offboard_udp_port": config.offboard_udp_port,
        "px4_dir": str(px4_dir),
        "started_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    (sitl_run_dir / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    with runner:
        yield runner

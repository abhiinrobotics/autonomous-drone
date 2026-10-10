"""Domain errors shared across layers.

Raising these (instead of built-ins) lets callers distinguish a bad
configuration from a dead link or a stale sensor, and keeps `except` clauses
narrow.
"""

from __future__ import annotations


class AeroError(Exception):
    """Base class for every error this project raises deliberately."""


class ConfigError(AeroError):
    """Configuration is missing, malformed, or inconsistent."""


# Name fixed by docs/rules/engineering_standards.md; ruff's N818 wants an
# "Error" suffix, but the rules doc names this exception explicitly.
class SensorTimeout(AeroError):  # noqa: N818
    """A sensor produced no fresh sample within its allowed age."""


class LinkLostError(AeroError):
    """The MAVLink connection to the flight controller is gone."""

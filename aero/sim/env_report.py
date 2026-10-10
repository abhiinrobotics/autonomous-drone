"""Pure helpers for the toolchain environment report (no I/O, no globals).

Version strings are compared on their numeric dotted prefix only; any
pre-release or packaging suffix (``-alpha1``, ``-96-g1512a55``, ``-4build2``)
is ignored. A pre-release of X.Y.Z therefore counts as satisfying a minimum of
X.Y.Z, which is what we want: PX4 ``v1.18.0-alpha1`` is accepted by a
``1.18.0`` floor, and the exact commit is recorded separately in the report.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass

# `gz sim --versions` prints one version per line, newest first. PX4's
# px4-rc.gzsim takes `head -n 1`, so the first line is the one that will run.
_GZ_VERSION_LINE = re.compile(r"^\s*(\d+(?:\.\d+)*)\s*$")
_NUMERIC_PREFIX = re.compile(r"^(\d+(?:\.\d+)*)")


@dataclass(frozen=True, slots=True)
class ToolCheck:
    """One row of the environment report.

    Args:
        name: Tool or package name as shown to the user.
        found: Version/identifier found, or None if the tool is missing.
        minimum: Minimum acceptable version, or None if any version passes.
        ok: Whether the check passed.
        note: Extra context (path, commit, why it failed).
    """

    name: str
    found: str | None
    minimum: str | None
    ok: bool
    note: str = ""


def parse_version(text: str | None) -> tuple[int, ...] | None:
    """Parse the leading dotted-numeric part of a version string.

    Args:
        text: e.g. ``"8.15.0"``, ``"v1.18.0-alpha1-96-g1512a552f9"``, None.

    Returns:
        Tuple of ints (``(1, 18, 0)``), or None if no numeric prefix exists.
    """
    if not text:
        return None
    stripped = text.strip().lstrip("vV")
    match = _NUMERIC_PREFIX.match(stripped)
    if match is None:
        return None
    return tuple(int(part) for part in match.group(1).split("."))


def meets_minimum(found: str | None, minimum: str | None) -> bool:
    """Whether `found` is at least `minimum` on their numeric prefixes.

    Args:
        found: Version found, or None (missing tool -> False).
        minimum: Required floor, or None (then any non-None `found` passes).

    Returns:
        True if the requirement is satisfied.
    """
    if found is None:
        return False
    if minimum is None:
        return True
    found_parts = parse_version(found)
    min_parts = parse_version(minimum)
    if found_parts is None or min_parts is None:
        return False
    width = max(len(found_parts), len(min_parts))
    padded_found = found_parts + (0,) * (width - len(found_parts))
    padded_min = min_parts + (0,) * (width - len(min_parts))
    return padded_found >= padded_min


def make_check(
    name: str,
    found: str | None,
    minimum: str | None = None,
    note: str = "",
) -> ToolCheck:
    """Build a `ToolCheck`, deriving `ok` from `found` vs `minimum`."""
    ok = meets_minimum(found, minimum)
    if not ok and not note:
        note = "not found" if found is None else f"needs >= {minimum}"
    return ToolCheck(name=name, found=found, minimum=minimum, ok=ok, note=note)


def parse_gz_versions(output: str) -> list[str]:
    """Extract version lines from `gz sim --versions` output, order preserved.

    Args:
        output: Raw stdout of ``gz sim --versions``.

    Returns:
        Versions in the order printed (the first is the one PX4 will use).
    """
    versions: list[str] = []
    for line in output.splitlines():
        match = _GZ_VERSION_LINE.match(line)
        if match is not None:
            versions.append(match.group(1))
    return versions


def select_gz_version(versions: Sequence[str]) -> str | None:
    """Return the gz version PX4 would pick (first line), or None if empty."""
    return versions[0] if versions else None


def failed(checks: Sequence[ToolCheck]) -> list[ToolCheck]:
    """Return the checks that did not pass."""
    return [check for check in checks if not check.ok]


def render_document(
    title: str,
    checks: Sequence[ToolCheck],
    extra_lines: Sequence[str] = (),
) -> str:
    """Assemble the full markdown report.

    Args:
        title: Heading line (without the leading `#`), usually with a timestamp.
        checks: Rows of the version table.
        extra_lines: Lines appended after the table (SITL smoke-test outcome).

    Returns:
        Markdown document ending in a newline.
    """
    parts = [f"# {title}", "", render_report(checks), ""]
    parts.extend(extra_lines)
    return "\n".join(parts) + "\n"


def render_report(checks: Sequence[ToolCheck]) -> str:
    """Render checks as a GitHub-flavoured markdown table for docs/STATUS.md."""
    lines = ["| Item | Found | Required | OK | Note |", "|---|---|---|---|---|"]
    for check in checks:
        lines.append(
            "| {name} | {found} | {minimum} | {ok} | {note} |".format(
                name=check.name,
                found=check.found or "-",
                minimum=check.minimum or "any",
                ok="yes" if check.ok else "NO",
                note=check.note or "",
            )
        )
    return "\n".join(lines)

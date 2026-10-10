"""Unit tests for the pure environment-report helpers."""

from __future__ import annotations

import pytest

from aero.sim.env_report import (
    ToolCheck,
    failed,
    make_check,
    meets_minimum,
    parse_gz_versions,
    parse_version,
    render_document,
    render_report,
    select_gz_version,
)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("8.15.0", (8, 15, 0)),
        ("v1.18.0-alpha1-96-g1512a552f9", (1, 18, 0)),
        ("3.10.12", (3, 10, 12)),
        ("0.23.92-4build2", (0, 23, 92)),
        ("7", (7,)),
        ("  8.15.0  ", (8, 15, 0)),
        ("unknown", None),
        ("", None),
        (None, None),
    ],
)
def test_parse_version(text: str | None, expected: tuple[int, ...] | None) -> None:
    assert parse_version(text) == expected


@pytest.mark.parametrize(
    ("found", "minimum", "expected"),
    [
        ("8.15.0", "8.0.0", True),
        ("7.9.0", "8.0.0", False),
        ("3.10", "3.10", True),
        ("3.10.12", "3.10", True),
        ("3.9.18", "3.10", False),
        # A pre-release of the floor version counts as meeting it.
        ("v1.18.0-alpha1-96-g1512a552f9", "1.15.0", True),
        ("1.14.3", "1.15.0", False),
        # No floor: any found version passes; a missing tool never does.
        ("anything", None, True),
        (None, None, False),
        (None, "1.0.0", False),
        ("unknown", "1.0.0", False),
    ],
)
def test_meets_minimum(found: str | None, minimum: str | None, expected: bool) -> None:
    assert meets_minimum(found, minimum) is expected


def test_make_check_passes_and_keeps_note() -> None:
    check = make_check("gz sim", "8.15.0", "8.0.0", note="installed: 8.15.0, 7.9.0")
    assert check == ToolCheck(
        name="gz sim",
        found="8.15.0",
        minimum="8.0.0",
        ok=True,
        note="installed: 8.15.0, 7.9.0",
    )


def test_make_check_explains_missing_tool() -> None:
    check = make_check("pyzbar", None)
    assert check.ok is False
    assert check.note == "not found"


def test_make_check_explains_too_old() -> None:
    check = make_check("gz sim", "7.9.0", "8.0.0")
    assert check.ok is False
    assert check.note == "needs >= 8.0.0"


def test_parse_gz_versions_keeps_order_and_ignores_prose() -> None:
    output = "8.15.0\n7.9.0\nThe 'gz' command provides a command line interface\n\n"
    assert parse_gz_versions(output) == ["8.15.0", "7.9.0"]


def test_parse_gz_versions_empty_when_not_installed() -> None:
    assert parse_gz_versions("") == []


def test_select_gz_version_takes_first_line() -> None:
    # PX4's px4-rc.gzsim uses `head -n 1`, so the first line is what runs.
    assert select_gz_version(["8.15.0", "7.9.0"]) == "8.15.0"
    assert select_gz_version([]) is None


def test_failed_returns_only_failures() -> None:
    ok_check = make_check("numpy", "1.26.4")
    bad_check = make_check("pyzbar", None)
    assert failed([ok_check, bad_check]) == [bad_check]


def test_render_report_marks_failures() -> None:
    table = render_report([make_check("numpy", "1.26.4"), make_check("pyzbar", None)])
    lines = table.splitlines()
    assert lines[0].startswith("| Item |")
    assert "| numpy | 1.26.4 | any | yes |" in lines[2]
    assert lines[3] == "| pyzbar | - | any | NO | not found |"


def test_render_document_includes_title_table_and_extra_lines() -> None:
    document = render_document(
        "Environment report (2026-10-10 06:28 UTC)",
        [make_check("numpy", "1.26.4")],
        ["SITL smoke test: PASS — heartbeat from sysid=1"],
    )
    lines = document.splitlines()
    assert lines[0] == "# Environment report (2026-10-10 06:28 UTC)"
    assert lines[2].startswith("| Item |")
    assert lines[-1] == "SITL smoke test: PASS — heartbeat from sysid=1"
    assert document.endswith("\n")


def test_render_document_without_extra_lines() -> None:
    document = render_document("t", [make_check("numpy", "1.26.4")])
    assert document.endswith("\n")
    assert "SITL" not in document

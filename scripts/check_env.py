#!/usr/bin/env python3
"""Part 0 environment bring-up check.

Reports the toolchain versions this project needs and (unless --no-sitl) runs a
PX4 SITL smoke test: start `gz_x500` headless, connect with pymavlink, receive
a heartbeat, tear the simulator down.

Usage:
    python scripts/check_env.py                 # versions + SITL smoke test
    python scripts/check_env.py --no-sitl        # versions only
Exit code 0 only if every check passed.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import logging
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

# Run from a plain checkout without `pip install -e .` as well: Part 0 exists to
# diagnose an unfinished environment, so it must not depend on being installed.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from aero.sim.env_report import (  # noqa: E402 - needs REPO_ROOT on sys.path first
    ToolCheck,
    failed,
    make_check,
    parse_gz_versions,
    render_document,
    select_gz_version,
)
from aero.sim.px4_sitl import (  # noqa: E402
    DEFAULT_PX4_DIR,
    Px4Sitl,
    SitlConfig,
    SitlError,
    probe_gz_worlds,
)

logger = logging.getLogger("check_env")

# Minimum versions. Sources: pyproject `requires-python`; PX4's
# ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim enforces gz >= 8.0.0
# (Gazebo Harmonic); PX4 1.15 is the first release with the gz_bridge layout
# this project relies on.
MIN_PYTHON = "3.10"
MIN_GZ_SIM = "8.0.0"
MIN_PX4 = "1.15.0"
# Python modules to import, with the distribution that provides them.
REQUIRED_MODULES: tuple[tuple[str, str], ...] = (
    ("pymavlink", "pymavlink"),
    ("numpy", "numpy"),
    ("cv2", "opencv-python"),
    ("pyzbar", "pyzbar"),
    ("yaml", "PyYAML"),
    ("qrcode", "qrcode"),
    ("pytest", "pytest"),
)
SUBPROCESS_TIMEOUT_S = 10.0
HEARTBEAT_TIMEOUT_S = 30.0
QR_ROUNDTRIP_PAYLOAD = "AEROTHON-2026-ENV-CHECK"


@dataclass(frozen=True, slots=True)
class SmokeResult:
    """Outcome of the SITL heartbeat smoke test.

    Args:
        ok: Whether a heartbeat was received.
        detail: Human-readable summary or failure cause.
        log_path: PX4 console log of the run, if it was started.
    """

    ok: bool
    detail: str
    log_path: Path | None


def _run(argv: list[str], cwd: Path | None = None) -> str:
    """Run a command and return stdout, or "" on any failure."""
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=SUBPROCESS_TIMEOUT_S,
            cwd=cwd,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        logger.debug("command_failed argv=%s error=%s", argv, error)
        return ""
    return result.stdout


def check_python() -> ToolCheck:
    """Check the running interpreter against MIN_PYTHON."""
    return make_check("python", platform.python_version(), MIN_PYTHON, note=sys.executable)


def check_module(module_name: str, distribution: str) -> ToolCheck:
    """Import a module and report the installed distribution version."""
    try:
        importlib.import_module(module_name)
    except ImportError as error:
        return make_check(distribution, None, note=f"import {module_name} failed: {error}")
    try:
        version = importlib.metadata.version(distribution)
    except importlib.metadata.PackageNotFoundError:
        version = "unknown"
    return make_check(distribution, version)


def check_qr_roundtrip() -> ToolCheck:
    """Generate a QR image and decode it, proving libzbar is usable.

    Covers the native `libzbar0` dependency, which a bare `import pyzbar`
    already exercises but which only fails visibly on an actual decode.
    """
    try:
        # Imported here on purpose: a missing dependency must be reported as a
        # failed check, not crash the checker at import time.
        import numpy as np  # noqa: PLC0415
        import qrcode  # noqa: PLC0415
        from pyzbar.pyzbar import decode  # noqa: PLC0415
    except ImportError as error:
        return make_check("qr encode/decode", None, note=f"missing dependency: {error}")
    try:
        image = qrcode.make(QR_ROUNDTRIP_PAYLOAD).convert("L")
        decoded = decode(np.array(image))
    except (OSError, ValueError) as error:
        return make_check("qr encode/decode", None, note=f"decode raised: {error}")
    payloads = [item.data.decode("utf-8", errors="replace") for item in decoded]
    if QR_ROUNDTRIP_PAYLOAD not in payloads:
        return make_check("qr encode/decode", None, note=f"decoded {payloads!r}")
    return make_check("qr encode/decode", "ok", note="libzbar reachable")


def check_gz_sim() -> ToolCheck:
    """Check the `gz sim` version PX4 would use."""
    versions = parse_gz_versions(_run(["gz", "sim", "--versions"]))
    selected = select_gz_version(versions)
    note = f"installed: {', '.join(versions)}" if len(versions) > 1 else ""
    return make_check("gz sim", selected, MIN_GZ_SIM, note=note)


def check_px4(px4_dir: Path) -> ToolCheck:
    """Check the PX4 checkout version and that the SITL binary is built."""
    if not px4_dir.is_dir():
        return make_check("PX4-Autopilot", None, MIN_PX4, note=f"{px4_dir} not found")
    describe = _run(["git", "describe", "--tags", "--always"], cwd=px4_dir).strip()
    commit = _run(["git", "rev-parse", "--short", "HEAD"], cwd=px4_dir).strip()
    binary = SitlConfig(px4_dir=px4_dir).px4_binary
    if not binary.is_file():
        return make_check(
            "PX4-Autopilot",
            describe or None,
            MIN_PX4,
            note=f"SITL not built; run: make -C {px4_dir} px4_sitl",
        )
    built_note = f"commit {commit}, built" if commit else "built"
    return make_check("PX4-Autopilot", describe or None, MIN_PX4, note=built_note)


def check_gz_idle() -> ToolCheck:
    """Check that Gazebo is reachable and running no world.

    A stale world matters: PX4 attaches to an existing one instead of starting
    its own, so the next run would fly in the wrong world.
    """
    probe = probe_gz_worlds()
    if not probe.probe_ok:
        return make_check("gz world idle", None, note="'gz topic -l' failed; cannot tell")
    if probe.worlds:
        return make_check(
            "gz world idle",
            None,
            note=f"already running: {list(probe.worlds)}; kill leftover 'gz sim' processes",
        )
    return make_check("gz world idle", "ok")


def collect_checks(px4_dir: Path) -> list[ToolCheck]:
    """Run every non-SITL check, in report order."""
    checks = [
        make_check("os", f"{platform.system()} {platform.release()}"),
        check_python(),
    ]
    checks.extend(check_module(module, dist) for module, dist in REQUIRED_MODULES)
    checks.append(check_qr_roundtrip())
    checks.append(check_gz_sim())
    checks.append(check_px4(px4_dir))
    checks.append(check_gz_idle())
    return checks


def heartbeat_smoke_test(config: SitlConfig, log_path: Path) -> SmokeResult:
    """Start SITL, wait for a MAVLink heartbeat, and shut everything down.

    Args:
        config: SITL launch configuration.
        log_path: Where PX4's console output is written.

    Returns:
        SmokeResult with the heartbeat summary or the failure cause.
    """
    # Deferred so that `check_env.py` still runs (and reports the missing
    # package) on a machine where pymavlink is not installed yet.
    from aero.sim.mavlink_probe import ProbeError, wait_for_heartbeat  # noqa: PLC0415

    try:
        runner = Px4Sitl(config, log_path)
    except SitlError as error:
        return SmokeResult(False, str(error), None)

    try:
        with runner:
            start_monotonic_s = time.monotonic()
            heartbeat = wait_for_heartbeat(config.offboard_udp_port, HEARTBEAT_TIMEOUT_S)
            if heartbeat is None:
                return SmokeResult(
                    False,
                    f"no heartbeat on port {config.offboard_udp_port} after ready "
                    f"within {HEARTBEAT_TIMEOUT_S:.0f} s",
                    log_path,
                )
            elapsed_s = time.monotonic() - start_monotonic_s
            detail = (
                f"heartbeat from sysid={heartbeat.system_id} "
                f"compid={heartbeat.component_id} type={heartbeat.mav_type} "
                f"autopilot={heartbeat.autopilot} (px4={heartbeat.is_px4}) in {elapsed_s:.1f} s"
            )
            return SmokeResult(True, detail, log_path)
    except (SitlError, ProbeError) as error:
        return SmokeResult(False, str(error), log_path)


def smoke_lines(smoke: SmokeResult | None, skip_reason: str | None) -> list[str]:
    """Report lines describing the SITL smoke test outcome (or why it was skipped).

    Args:
        smoke: Result of the smoke test, or None if it did not run.
        skip_reason: Why it did not run; required when `smoke` is None.

    Returns:
        Markdown lines to append after the version table.
    """
    if smoke is None:
        return [f"SITL smoke test: skipped ({skip_reason or 'unknown reason'})"]
    lines = [f"SITL smoke test: {'PASS' if smoke.ok else 'FAIL'} — {smoke.detail}"]
    if smoke.log_path is not None:
        lines.append(f"PX4 log: `{smoke.log_path}`")
    return lines


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-dir", type=Path, default=DEFAULT_PX4_DIR, help="PX4-Autopilot root")
    parser.add_argument("--model", default="gz_x500", help="PX4_SIM_MODEL to launch")
    parser.add_argument("--world", default="default", help="PX4_GZ_WORLD to launch")
    parser.add_argument("--no-sitl", action="store_true", help="skip the SITL smoke test")
    parser.add_argument("--gui", action="store_true", help="run gz with the GUI (not headless)")
    parser.add_argument(
        "--out",
        type=Path,
        default=REPO_ROOT / "artifacts" / "part0" / "env_report.md",
        help="where to write the markdown report",
    )
    return parser.parse_args(argv)


def run_smoke(
    args: argparse.Namespace, checks: list[ToolCheck]
) -> tuple[SmokeResult | None, str | None]:
    """Run the SITL smoke test unless it is skipped.

    Args:
        args: Parsed command line.
        checks: Version checks already collected; a failure here skips the test.

    Returns:
        (smoke result or None, reason it was skipped or None).
    """
    blocking = failed(checks)
    if args.no_sitl:
        logger.info("sitl_smoke_skipped reason=--no-sitl")
        return None, "--no-sitl"
    if blocking:
        failed_names = ", ".join(check.name for check in blocking)
        logger.error("sitl_smoke_skipped reason=failed_checks items=%s", failed_names)
        return None, f"failed checks: {failed_names}"
    run_stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    log_path = REPO_ROOT / "logs" / f"{run_stamp}_env_check" / "px4.log"
    config = SitlConfig(
        px4_dir=args.px4_dir,
        model=args.model,
        world=args.world,
        headless=not args.gui,
    )
    return heartbeat_smoke_test(config, log_path), None


def main(argv: list[str] | None = None) -> int:
    """Run all checks, write the report, and return the process exit code."""
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
        stream=sys.stderr,
    )

    checks = collect_checks(args.px4_dir)
    smoke: SmokeResult | None = None
    skip_reason: str | None = None
    try:
        smoke, skip_reason = run_smoke(args, checks)
    except Exception as error:
        # Writing the report is the point of this script, so an unexpected
        # failure is logged with its traceback and reported as a FAIL row.
        logger.exception("sitl_smoke_crashed")
        smoke = SmokeResult(False, f"unexpected error: {error!r}", None)

    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    report = render_document(
        f"Environment report ({stamp})", checks, smoke_lines(smoke, skip_reason)
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(report, encoding="utf-8")
    print(report)
    print(f"report written to {args.out}")

    blocking = failed(checks)
    if blocking:
        print(f"FAILED checks: {', '.join(check.name for check in blocking)}", file=sys.stderr)
        return 1
    if smoke is not None and not smoke.ok:
        print("FAILED: SITL smoke test", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Managed child process groups for the simulator stack.

Both PX4 SITL and the standalone gz server spawn helper processes of their own
(PX4's rcS starts `gz sim`; gz starts render/physics threads and, with a GUI,
another process). Killing the direct child therefore leaks siblings, so every
simulator process is started in its own session — which makes its pid the
process-group id — and torn down with `killpg`.

Teardown rules learned the hard way (see docs/DECISIONS.md D-009):

* record the pgid at spawn time; after `Popen.poll()` reaps the child its pid
  is gone and may be recycled, so the group must never be derived from it later;
* escalate SIGINT -> SIGKILL, and run the final SIGKILL from a `finally` block
  so a `KeyboardInterrupt` cannot abort teardown half-way;
* write the pgid next to the log, so a run killed with SIGKILL can still be
  cleaned up by hand.
"""

from __future__ import annotations

import contextlib
import logging
import os
import signal
import subprocess
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from types import TracebackType
from typing import BinaryIO

from aero.core.errors import AeroError

logger = logging.getLogger(__name__)

# Name of the file holding the process-group id, written next to the log.
PGID_FILENAME_SUFFIX = ".pgid"
DEFAULT_STOP_GRACE_S = 5.0


class ProcessError(AeroError):
    """A managed process could not be started or behaved unexpectedly."""


class ProcessGroup:
    """One child process plus everything it spawns, owned as a unit."""

    def __init__(
        self,
        name: str,
        argv: Sequence[str],
        log_path: Path,
        *,
        cwd: Path | None = None,
        env: Mapping[str, str] | None = None,
        stop_grace_s: float = DEFAULT_STOP_GRACE_S,
    ) -> None:
        """Describe the process; nothing is started until `start()`.

        Args:
            name: Short label used in log messages.
            argv: Command line to execute.
            log_path: File receiving stdout and stderr.
            cwd: Working directory, or None to inherit.
            env: Full environment for the child, or None to inherit.
            stop_grace_s: Time SIGINT is given before SIGKILL.
        """
        self._name = name
        self._argv = list(argv)
        self._log_path = log_path
        self._cwd = cwd
        self._env = None if env is None else dict(env)
        self._stop_grace_s = stop_grace_s
        self._process: subprocess.Popen[bytes] | None = None
        self._log_file: BinaryIO | None = None
        self._pgid: int | None = None

    @property
    def name(self) -> str:
        """Short label of this process."""
        return self._name

    @property
    def log_path(self) -> Path:
        """File receiving the child's stdout and stderr."""
        return self._log_path

    @property
    def pgid_path(self) -> Path:
        """File holding the recorded process-group id."""
        return self._log_path.with_suffix(self._log_path.suffix + PGID_FILENAME_SUFFIX)

    @property
    def pgid(self) -> int | None:
        """Process-group id recorded at spawn time, or None before `start()`."""
        return self._pgid

    @property
    def is_running(self) -> bool:
        """True while the direct child is alive."""
        return self._process is not None and self._process.poll() is None

    def poll(self) -> int | None:
        """Exit code of the child, or None while it is running.

        Calling this reaps the child once it has exited, which is why the pgid
        is recorded at spawn time rather than looked up here.
        """
        if self._process is None:
            return None
        return self._process.poll()

    def start(self) -> None:
        """Spawn the process in its own session.

        Raises:
            ProcessError: Already started, or the executable cannot be run.
        """
        if self._process is not None:
            raise ProcessError(f"{self._name} already started")
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = self._log_path.open("wb")
        self._log_file = log_file
        logger.info("process_start name=%s log=%s", self._name, self._log_path)
        try:
            self._process = subprocess.Popen(
                self._argv,
                cwd=self._cwd,
                env=self._env,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as error:
            log_file.close()
            self._log_file = None
            raise ProcessError(f"failed to execute {self._argv[0]}: {error}") from error
        self._pgid = self._process.pid
        with contextlib.suppress(OSError):
            self.pgid_path.write_text(f"{self._pgid}\n", encoding="utf-8")

    def read_log(self) -> str:
        """Current log contents, decoded leniently (empty if not written yet)."""
        try:
            return self._log_path.read_bytes().decode("utf-8", errors="replace")
        except OSError:
            return ""

    def stop(self) -> None:
        """Terminate the whole group, then close the log.

        Safe before `start()`, after the child exited, and when called twice.
        """
        pgid = self._pgid
        process = self._process
        try:
            if pgid is not None:
                self._kill_group(pgid, process)
        finally:
            self._pgid = None
            self._process = None
            if process is not None:
                with contextlib.suppress(OSError, ValueError, subprocess.TimeoutExpired):
                    process.wait(timeout=0)
            log_file = self._log_file
            if log_file is not None:
                log_file.close()
                self._log_file = None

    def _kill_group(self, pgid: int, process: subprocess.Popen[bytes] | None) -> None:
        """SIGINT the group, then SIGKILL it, never leaving it signalled once."""
        killed = False
        try:
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(pgid, signal.SIGINT)
            if process is not None:
                with contextlib.suppress(subprocess.TimeoutExpired):
                    process.wait(timeout=self._stop_grace_s)
            else:
                # No child handle (already reaped): give the group the same
                # grace period before escalating.
                time.sleep(self._stop_grace_s)
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(pgid, signal.SIGKILL)
            killed = True
        finally:
            if not killed:
                with contextlib.suppress(ProcessLookupError, PermissionError):
                    os.killpg(pgid, signal.SIGKILL)
        if process is not None:
            with contextlib.suppress(subprocess.TimeoutExpired):
                process.wait(timeout=self._stop_grace_s)

    def __enter__(self) -> ProcessGroup:
        """Start the process."""
        self.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Guarantee teardown."""
        self.stop()

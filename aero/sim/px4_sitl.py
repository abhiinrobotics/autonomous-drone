"""Start and stop PX4 SITL, either spawning its own Gazebo or attaching to ours.

Used by `scripts/check_env.py`, `scripts/capture_part1.py` and the pytest
fixtures. Mission code must never import this module.

Two modes:

* **Self-hosted gz** (`standalone_gz=False`): PX4's `rcS` starts `gz sim` with
  one of PX4's own worlds and spawns the stock airframe model. Used by the
  Part 0 bring-up check. The `gz sim` server is a sibling process, so teardown
  must kill the whole process group (see `aero.sim.process`).
* **Attach** (`standalone_gz=True` plus `gz_model_name`): we start the gz server
  ourselves (`aero.sim.gz_server`), which lets us pass `--headless-rendering`
  for the cameras and load a generated world, and PX4 only connects its
  gz_bridge to a model that already exists. `PX4_SIM_MODEL` still selects the
  PX4 *airframe* (`gz_x500`), which is why our vehicle model can be called
  something else without touching PX4's ROMFS.

Readiness is decided by a MAVLink heartbeat, not by scraping the console log:
PX4's stdout is block-buffered into a file, so a marker line can sit unflushed
while the vehicle is already alive. The log is read only to explain a failure.
"""

from __future__ import annotations

import logging
import os
import subprocess
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType

from aero.core.errors import AeroError
from aero.sim.mavlink_probe import Heartbeat, offboard_udp_port, wait_for_heartbeat
from aero.sim.process import ProcessError, ProcessGroup

logger = logging.getLogger(__name__)

# PX4 init scripts print this marker for failures; some are non-fatal, so it is
# logged as a warning and never aborts the wait on its own.
_INIT_ERROR_MARKER = "ERROR [init]"
LOOPBACK_IP = "127.0.0.1"
_DEFAULT_BUILD_DIR = "px4_sitl_default"
DEFAULT_PX4_DIR = Path.home() / "PX4-Autopilot"
# Kept for callers that locate the recorded process-group id file.
PGID_FILENAME = "px4.log.pgid"


class SitlError(AeroError):
    """PX4 SITL could not be started, became ready, or exited unexpectedly."""


@dataclass(frozen=True, slots=True)
class GzProbe:
    """Result of asking Gazebo which worlds are running.

    Args:
        worlds: World names found publishing a clock topic.
        probe_ok: False if `gz topic -l` could not be run at all, in which case
            `worlds` is empty but says nothing about the machine's state.
    """

    worlds: tuple[str, ...]
    probe_ok: bool


@dataclass(frozen=True, slots=True)
class SitlConfig:
    """Everything needed to launch one PX4 SITL instance.

    Args:
        px4_dir: Root of the PX4-Autopilot checkout.
        model: PX4 `PX4_SIM_MODEL`, i.e. the *airframe* name (e.g. `gz_x500`).
        world: Gazebo world name without `.sdf` (`PX4_GZ_WORLD`).
        headless: True to run PX4's own gz server without the gz GUI. Ignored
            in attach mode, where the server is started by `gz_server`.
        instance: PX4 instance id (`-i`); also offsets the MAVLink port.
        speed_factor: `PX4_SIM_SPEED_FACTOR`, or None for real time.
        build_dir_name: Build directory under `px4_dir/build`.
        ready_timeout_s: Max wait for the first MAVLink heartbeat.
        heartbeat_attempt_s: Per-attempt heartbeat wait inside that budget.
        stop_grace_s: Time given to SIGINT before SIGKILL on teardown.
        allow_existing_world: If False, refuse to start while another gz world
            is already running (PX4 would silently attach to it). Attach mode
            ignores this: there the running world is ours on purpose.
        standalone_gz: True to attach to an already running gz server.
        gz_model_name: Name of the pre-spawned model to attach gz_bridge to.
        extra_env: Additional environment variables as (key, value) pairs.
    """

    px4_dir: Path = DEFAULT_PX4_DIR
    model: str = "gz_x500"
    world: str = "default"
    headless: bool = True
    instance: int = 0
    speed_factor: float | None = None
    build_dir_name: str = _DEFAULT_BUILD_DIR
    ready_timeout_s: float = 120.0
    heartbeat_attempt_s: float = 2.0
    stop_grace_s: float = 5.0
    allow_existing_world: bool = False
    standalone_gz: bool = False
    gz_model_name: str | None = None
    extra_env: tuple[tuple[str, str], ...] = ()

    @property
    def rootfs_dir(self) -> Path:
        """Working directory PX4 must run from (holds `etc/`, `gz_env.sh`)."""
        return self.px4_dir / "build" / self.build_dir_name / "rootfs"

    @property
    def px4_binary(self) -> Path:
        """Path of the built SITL `px4` binary."""
        return self.px4_dir / "build" / self.build_dir_name / "bin" / "px4"

    @property
    def offboard_udp_port(self) -> int:
        """UDP port this instance streams the onboard/offboard link to."""
        return offboard_udp_port(self.instance)

    def validate(self) -> None:
        """Fail fast with an actionable message if the setup cannot work.

        Raises:
            SitlError: Binary or rootfs absent, the airframe is unknown, or
                attach mode was requested without a model name.
        """
        if not self.px4_binary.is_file():
            raise SitlError(
                f"px4 SITL binary not found at {self.px4_binary}; "
                f"build it with: make -C {self.px4_dir} px4_sitl"
            )
        if not self.rootfs_dir.is_dir():
            raise SitlError(f"PX4 rootfs not found at {self.rootfs_dir}")
        airframes = self.rootfs_dir / "etc" / "init.d-posix" / "airframes"
        if airframes.is_dir() and not any(
            path.name.endswith(f"_{self.model}") for path in airframes.iterdir()
        ):
            raise SitlError(f"unknown PX4_SIM_MODEL={self.model!r} (no airframe in {airframes})")
        if self.standalone_gz and not self.gz_model_name:
            raise SitlError("standalone_gz requires gz_model_name (the pre-spawned model)")

    def env(self, base_env: Mapping[str, str]) -> dict[str, str]:
        """Build the child environment.

        Args:
            base_env: Environment to inherit from (normally `os.environ`).

        Returns:
            New environment dict with the PX4/gz variables applied.
        """
        env = dict(base_env)
        env["PX4_SIM_MODEL"] = self.model
        env["PX4_GZ_WORLD"] = self.world
        env["GZ_IP"] = LOOPBACK_IP
        if self.standalone_gz:
            # PX4 then waits for an existing world instead of starting one.
            env["PX4_GZ_STANDALONE"] = "1"
        if self.gz_model_name is not None:
            env["PX4_GZ_MODEL_NAME"] = self.gz_model_name
        if self.headless and not self.standalone_gz:
            env["HEADLESS"] = "1"
            # No GUI to steer: skip the follow-camera topic publish in rcS.
            env["PX4_GZ_NO_FOLLOW"] = "1"
        if self.speed_factor is not None:
            env["PX4_SIM_SPEED_FACTOR"] = str(self.speed_factor)
        env.update(dict(self.extra_env))
        return env

    def argv(self) -> list[str]:
        """Command line for the PX4 binary (`-d`: no interactive pxh shell).

        `-d` matters: with a non-tty stdin the pxh shell spins on EOF and
        writes gigabytes of prompts into the log.
        """
        return [str(self.px4_binary), "-d", "-i", str(self.instance)]


def parse_world_names(topic_list: str) -> list[str]:
    """Extract world names from `gz topic -l` output.

    Mirrors the detection in PX4's `px4-rc.gzsim`, which looks for a
    `/world/<name>/clock` topic.

    Args:
        topic_list: Raw stdout of `gz topic -l`.

    Returns:
        World names found, in the order listed.
    """
    worlds: list[str] = []
    for line in topic_list.splitlines():
        parts = line.strip().split("/")
        if len(parts) == 4 and parts[0] == "" and parts[1] == "world" and parts[3] == "clock":
            worlds.append(parts[2])
    return worlds


def probe_gz_worlds(timeout_s: float = 5.0) -> GzProbe:
    """Ask Gazebo which worlds are currently running.

    Args:
        timeout_s: Timeout for the `gz topic -l` call.

    Returns:
        GzProbe; `probe_ok` is False when gz could not be reached at all, so
        callers never mistake "cannot tell" for "nothing running".
    """
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        logger.warning("gz_probe_failed error=%s", error)
        return GzProbe(worlds=(), probe_ok=False)
    return GzProbe(worlds=tuple(parse_world_names(result.stdout)), probe_ok=True)


class Px4Sitl:
    """Context manager owning one PX4 SITL process group."""

    def __init__(self, config: SitlConfig, log_path: Path) -> None:
        """Create the runner.

        Args:
            config: Launch configuration (validated here).
            log_path: File that receives PX4 stdout+stderr.
        """
        config.validate()
        self._config = config
        self._process = ProcessGroup(
            name="px4",
            argv=config.argv(),
            log_path=log_path,
            cwd=config.rootfs_dir,
            env=config.env(os.environ),
            stop_grace_s=config.stop_grace_s,
        )
        self._heartbeat: Heartbeat | None = None

    @property
    def config(self) -> SitlConfig:
        """Launch configuration of this run."""
        return self._config

    @property
    def log_path(self) -> Path:
        """Path of the PX4 console log for this run."""
        return self._process.log_path

    @property
    def heartbeat(self) -> Heartbeat | None:
        """Heartbeat that marked the instance ready, if it is ready."""
        return self._heartbeat

    @property
    def is_running(self) -> bool:
        """True while the PX4 process is alive."""
        return self._process.is_running

    def start(self) -> None:
        """Launch PX4 SITL in its own process group.

        Raises:
            SitlError: A stale gz world is running (self-hosted mode only), gz
                could not be probed, or the binary cannot be executed.
        """
        if not self._config.standalone_gz and not self._config.allow_existing_world:
            self._require_idle_gz()
        try:
            self._process.start()
        except ProcessError as error:
            raise SitlError(str(error)) from error

    def wait_until_ready(self, timeout_s: float | None = None) -> Heartbeat:
        """Block until the instance sends a MAVLink heartbeat.

        Args:
            timeout_s: Override for `SitlConfig.ready_timeout_s`.

        Returns:
            The first heartbeat received.

        Raises:
            SitlError: PX4 exited, or no heartbeat within the budget.
        """
        if self._process.pgid is None:
            raise SitlError("SITL not started")
        budget_s = self._config.ready_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + budget_s
        warned_init_error = False
        while time.monotonic() < deadline:
            exit_code = self._process.poll()
            if exit_code is not None:
                raise SitlError(
                    f"PX4 exited with code {exit_code} before sending a heartbeat; "
                    f"{self._log_hint()}"
                )
            if not warned_init_error and _INIT_ERROR_MARKER in self._process.read_log():
                logger.warning("px4_init_error line=%r", self._last_error_line())
                warned_init_error = True
            heartbeat = wait_for_heartbeat(
                self._config.offboard_udp_port, self._config.heartbeat_attempt_s
            )
            if heartbeat is not None:
                logger.info(
                    "sitl_ready sysid=%d autopilot=%d log=%s",
                    heartbeat.system_id,
                    heartbeat.autopilot,
                    self.log_path,
                )
                self._heartbeat = heartbeat
                return heartbeat
        raise SitlError(
            f"no MAVLink heartbeat on port {self._config.offboard_udp_port} within "
            f"{budget_s:.0f} s; {self._log_hint()}"
        )

    def stop(self) -> None:
        """Terminate PX4 and everything it started."""
        self._process.stop()

    def _require_idle_gz(self) -> None:
        """Raise unless Gazebo is reachable and running no world.

        Raises:
            SitlError: A world is already running, or gz could not be probed
                (never assume "idle" from missing data).
        """
        probe = probe_gz_worlds()
        if not probe.probe_ok:
            raise SitlError(
                "could not ask gz which worlds are running ('gz topic -l' failed); "
                "refusing to start because PX4 would silently attach to a stale world. "
                "Pass allow_existing_world=True to override."
            )
        if probe.worlds:
            raise SitlError(
                f"gz world(s) already running: {list(probe.worlds)}; PX4 would attach to a "
                "stale world. Kill leftover 'gz sim' processes first."
            )

    def _last_error_line(self) -> str:
        """Last `ERROR [init]` line in the log, for logs and exception text."""
        matches = [
            line.strip()
            for line in self._process.read_log().splitlines()
            if _INIT_ERROR_MARKER in line
        ]
        return matches[-1] if matches else _INIT_ERROR_MARKER

    def _log_hint(self) -> str:
        """Failure suffix: the last init error if there is one, plus the log path."""
        if _INIT_ERROR_MARKER in self._process.read_log():
            return f"last error: {self._last_error_line()!r}; see {self.log_path}"
        return f"see {self.log_path}"

    def __enter__(self) -> Px4Sitl:
        """Start SITL and wait until a heartbeat arrives."""
        self.start()
        try:
            self.wait_until_ready()
        except BaseException:
            self.stop()
            raise
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Guarantee teardown of PX4 and anything it spawned."""
        self.stop()

"""Run the Gazebo server ourselves and spawn the vehicle into it.

PX4 can start Gazebo on its own, but then we cannot choose the server flags or
load a generated world. Running the server here buys three things:

* `--headless-rendering`, without which the camera sensors never publish on a
  machine with no GUI attached to the server;
* a generated world from `sim/worlds/<seed>/`, with its texture directory on
  the resource path so relative `albedo_map` paths resolve;
* control over the vehicle: we spawn `sim/models/aero_x500` through gz's
  `create` service and let PX4 attach to it (`PX4_GZ_STANDALONE`), so PX4's
  ROMFS needs no new airframe entry for our model.

The system plugins (physics, sensors, scene broadcaster, IMU, NavSat,
magnetometer, air pressure, contact) come from PX4's own `server.config`, which
is why the generated world file carries no `<plugin>` block.
"""

from __future__ import annotations

import logging
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import TracebackType

from aero.core.errors import AeroError, ConfigError
from aero.sim.process import ProcessError, ProcessGroup
from aero.sim.px4_sitl import DEFAULT_PX4_DIR, LOOPBACK_IP, probe_gz_worlds

logger = logging.getLogger(__name__)

GZ_BINARY = "gz"
# Service call budget for spawning a model, milliseconds (gz expects ms).
_SPAWN_TIMEOUT_MS = 10_000
_SPAWN_SUBPROCESS_TIMEOUT_S = 30.0
_SPAWN_OK_MARKER = "data: true"


class GzError(AeroError):
    """The Gazebo server could not be started, became ready, or refused a call."""


def world_name_from_sdf(world_path: Path) -> str:
    """Read `<world name=...>` from an SDF file.

    Args:
        world_path: Path of the world SDF.

    Returns:
        The world name, which gz uses in every topic and service path.

    Raises:
        ConfigError: The file is unreadable, not XML, or has no named world.
    """
    try:
        tree = ET.parse(world_path)
    except OSError as error:
        raise ConfigError(f"cannot read world {world_path}: {error}") from error
    except ET.ParseError as error:
        raise ConfigError(f"{world_path} is not valid XML: {error}") from error
    world = tree.getroot().find("world")
    if world is None or not world.get("name"):
        raise ConfigError(f"{world_path} has no <world name=...> element")
    return str(world.get("name"))


@dataclass(frozen=True, slots=True)
class GzServerConfig:
    """How to run the gz server for one world.

    Args:
        world_path: Generated `world.sdf` to load.
        px4_dir: PX4 checkout, used for its models, gz plugins and server config.
        headless_rendering: True to render offscreen (required without a GUI).
        verbosity: gz `-v` level.
        ready_timeout_s: Max wait for the world's clock topic to appear.
        poll_interval_s: Poll period while waiting.
        stop_grace_s: Time given to SIGINT before SIGKILL on teardown.
        extra_resource_paths: Additional directories for `GZ_SIM_RESOURCE_PATH`.
    """

    world_path: Path
    px4_dir: Path = DEFAULT_PX4_DIR
    headless_rendering: bool = True
    verbosity: int = 1
    ready_timeout_s: float = 90.0
    poll_interval_s: float = 0.5
    stop_grace_s: float = 5.0
    extra_resource_paths: tuple[Path, ...] = ()

    @property
    def px4_models_dir(self) -> Path:
        """PX4's gz model directory (holds `x500`, which our model includes)."""
        return self.px4_dir / "Tools" / "simulation" / "gz" / "models"

    @property
    def px4_plugin_dir(self) -> Path:
        """Built PX4 gz plugins (motor models, optical flow)."""
        return (
            self.px4_dir
            / "build"
            / "px4_sitl_default"
            / "src"
            / "modules"
            / "simulation"
            / ("gz_plugins")
        )

    @property
    def px4_server_config(self) -> Path:
        """PX4's gz `server.config`, which lists the system plugins to load."""
        return self.px4_dir / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"

    @property
    def resource_paths(self) -> tuple[Path, ...]:
        """Directories gz searches for models, worlds and textures.

        The world's own directory is included so that the relative
        `textures/*.png` paths inside the generated SDF resolve.
        """
        return (
            self.world_path.parent,
            *self.extra_resource_paths,
            self.px4_models_dir,
        )

    def validate(self) -> None:
        """Check that the world and PX4 assets exist.

        Raises:
            GzError: A required path is missing.
        """
        if not self.world_path.is_file():
            raise GzError(f"world not found: {self.world_path}; generate it with make_world.py")
        for path, what in (
            (self.px4_models_dir, "PX4 gz models"),
            (self.px4_server_config, "PX4 gz server.config"),
        ):
            if not path.exists():
                raise GzError(f"{what} not found at {path}")

    def env(self, base_env: Mapping[str, str]) -> dict[str, str]:
        """Build the gz server environment.

        Args:
            base_env: Environment to inherit from (normally `os.environ`).

        Returns:
            New environment with the gz search paths applied.
        """
        env = dict(base_env)
        existing = env.get("GZ_SIM_RESOURCE_PATH", "")
        paths = [str(path) for path in self.resource_paths]
        if existing:
            paths.append(existing)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(paths)
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(self.px4_plugin_dir)
        env["GZ_SIM_SERVER_CONFIG_PATH"] = str(self.px4_server_config)
        env["GZ_IP"] = LOOPBACK_IP
        return env

    def argv(self) -> list[str]:
        """Command line for the gz server (`-s`: server only, `-r`: run)."""
        argv = [GZ_BINARY, "sim", "-s", "-r", f"-v{self.verbosity}"]
        if self.headless_rendering:
            argv.append("--headless-rendering")
        argv.append(str(self.world_path))
        return argv


def spawn_request(
    name: str, model_sdf: Path, xyz_m: tuple[float, float, float], yaw_rad: float
) -> str:
    """Build the `gz.msgs.EntityFactory` request text for a model spawn.

    Args:
        name: Instance name to give the model in the world.
        model_sdf: Absolute path of the model's `model.sdf`.
        xyz_m: Spawn position in world ENU metres.
        yaw_rad: Spawn heading about +z (gz convention: 0 faces +x).

    Returns:
        The request in protobuf text format.
    """
    x, y, z = xyz_m
    # Yaw-only quaternion.
    quaternion_z = math.sin(yaw_rad / 2.0)
    quaternion_w = math.cos(yaw_rad / 2.0)
    return (
        f'name: "{name}", allow_renaming: false, sdf_filename: "{model_sdf}", '
        f"pose: {{position: {{x: {x:.6g}, y: {y:.6g}, z: {z:.6g}}}, "
        f"orientation: {{x: 0, y: 0, z: {quaternion_z:.9g}, w: {quaternion_w:.9g}}}}}"
    )


class GzServer:
    """Context manager owning one gz server process group."""

    def __init__(self, config: GzServerConfig, log_path: Path) -> None:
        """Create the runner.

        Args:
            config: Server configuration (validated here).
            log_path: File that receives gz stdout+stderr.

        Raises:
            GzError: The world or PX4 assets are missing.
        """
        config.validate()
        self._config = config
        self._world_name = world_name_from_sdf(config.world_path)
        self._process = ProcessGroup(
            name="gz",
            argv=config.argv(),
            log_path=log_path,
            env=config.env(os.environ),
            stop_grace_s=config.stop_grace_s,
        )

    @property
    def world_name(self) -> str:
        """Name of the loaded world, as used in gz topic and service paths."""
        return self._world_name

    @property
    def log_path(self) -> Path:
        """Path of the gz console log for this run."""
        return self._process.log_path

    @property
    def is_running(self) -> bool:
        """True while the gz server process is alive."""
        return self._process.is_running

    def start(self) -> None:
        """Launch the gz server in its own process group.

        Raises:
            GzError: Another world is already running, or gz cannot be executed.
        """
        probe = probe_gz_worlds()
        if probe.worlds:
            raise GzError(
                f"gz world(s) already running: {list(probe.worlds)}; "
                "kill leftover 'gz sim' processes first (make sitl-clean)"
            )
        try:
            self._process.start()
        except ProcessError as error:
            raise GzError(str(error)) from error

    def wait_until_ready(self, timeout_s: float | None = None) -> None:
        """Block until the world publishes its clock topic.

        Args:
            timeout_s: Override for `GzServerConfig.ready_timeout_s`.

        Raises:
            GzError: The server exited, or the world never appeared.
        """
        budget_s = self._config.ready_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            exit_code = self._process.poll()
            if exit_code is not None:
                raise GzError(f"gz server exited with code {exit_code}; see {self.log_path}")
            if self._world_name in probe_gz_worlds().worlds:
                logger.info("gz_ready world=%s log=%s", self._world_name, self.log_path)
                return
            time.sleep(self._config.poll_interval_s)
        raise GzError(
            f"gz world {self._world_name!r} did not start within {budget_s:.0f} s; "
            f"see {self.log_path}"
        )

    def spawn_model(
        self,
        name: str,
        model_sdf: Path,
        xyz_m: tuple[float, float, float],
        yaw_rad: float = 0.0,
    ) -> None:
        """Spawn a model into the running world through gz's `create` service.

        Args:
            name: Instance name for the model.
            model_sdf: Absolute path of the model's `model.sdf`.
            xyz_m: Spawn position in world ENU metres.
            yaw_rad: Spawn heading about +z.

        Raises:
            GzError: The service call failed or the world rejected the model.
        """
        if not model_sdf.is_file():
            raise GzError(f"model not found: {model_sdf}; generate it with make_model")
        self._call_service(
            service=f"/world/{self._world_name}/create",
            request_type="gz.msgs.EntityFactory",
            request=spawn_request(name, model_sdf, xyz_m, yaw_rad),
            what=f"spawn of {name!r}",
        )
        logger.info("model_spawned name=%s world=%s pose=%s", name, self._world_name, xyz_m)

    def set_model_pose(
        self,
        name: str,
        xyz_m: tuple[float, float, float],
        yaw_rad: float = 0.0,
    ) -> None:
        """Teleport an existing model (gz `set_pose`, from the user-commands system).

        Physics keeps running, so a dynamic model starts falling again
        immediately; callers that need it held at an altitude must repeat the
        call. This exists so sensor captures can look at a part of the field
        without flying there, which needs offboard control (Part 2).

        Args:
            name: Model instance name in the world.
            xyz_m: Target position in world ENU metres.
            yaw_rad: Target heading about +z.

        Raises:
            GzError: The service call failed or was refused.
        """
        x, y, z = xyz_m
        quaternion_z = math.sin(yaw_rad / 2.0)
        quaternion_w = math.cos(yaw_rad / 2.0)
        request = (
            f'name: "{name}", position: {{x: {x:.6g}, y: {y:.6g}, z: {z:.6g}}}, '
            f"orientation: {{x: 0, y: 0, z: {quaternion_z:.9g}, w: {quaternion_w:.9g}}}"
        )
        self._call_service(
            service=f"/world/{self._world_name}/set_pose",
            request_type="gz.msgs.Pose",
            request=request,
            what=f"set_pose of {name!r}",
        )

    def set_paused(self, paused: bool) -> None:
        """Pause or resume the simulation (gz `WorldControl`).

        Pausing is how sensor captures stay deterministic: a teleported
        dynamic model otherwise keeps the velocity gravity gave it, tumbles,
        and the frame is taken from somewhere unintended. Never pause while
        PX4 is attached — PX4 runs in lockstep with the simulation clock.

        Args:
            paused: True to pause, False to resume.

        Raises:
            GzError: The service call failed or was refused.
        """
        self._call_service(
            service=f"/world/{self._world_name}/control",
            request_type="gz.msgs.WorldControl",
            request=f"pause: {str(paused).lower()}",
            what=f"{'pause' if paused else 'resume'} of {self._world_name!r}",
        )

    def step(self, iterations: int) -> None:
        """Advance a paused simulation by `iterations` physics steps.

        Args:
            iterations: Number of steps; one step is the world's
                `max_step_size` (4 ms with the generated worlds).

        Raises:
            GzError: The call failed, or `iterations` is not positive.
        """
        if iterations < 1:
            raise GzError(f"iterations must be >= 1, got {iterations}")
        self._call_service(
            service=f"/world/{self._world_name}/control",
            request_type="gz.msgs.WorldControl",
            request=f"pause: true, multi_step: {iterations}",
            what=f"step of {self._world_name!r}",
        )

    def _call_service(self, service: str, request_type: str, request: str, what: str) -> None:
        """Call a gz service that replies with `gz.msgs.Boolean`.

        Args:
            service: Service path.
            request_type: Request message type name.
            request: Request in protobuf text format.
            what: Description used in error messages.

        Raises:
            GzError: The call could not run, or the reply was not `true`.
        """
        argv = [
            GZ_BINARY,
            "service",
            "-s",
            service,
            "--reqtype",
            request_type,
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(_SPAWN_TIMEOUT_MS),
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                argv,
                capture_output=True,
                text=True,
                timeout=_SPAWN_SUBPROCESS_TIMEOUT_S,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise GzError(f"{what} failed to run: {error}") from error
        if _SPAWN_OK_MARKER not in result.stdout:
            raise GzError(
                f"gz refused {what}: stdout={result.stdout.strip()!r} "
                f"stderr={result.stderr.strip()!r}"
            )

    def stop(self) -> None:
        """Terminate the gz server and everything it started."""
        self._process.stop()

    def __enter__(self) -> GzServer:
        """Start the server and wait until the world is up."""
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
        """Guarantee teardown of the gz server."""
        self.stop()

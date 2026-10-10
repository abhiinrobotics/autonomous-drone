"""Part 1 SITL acceptance tests: generated world, vehicle sensors, frame mapping.

Two fixtures, because the two questions need opposite simulator treatment:

* `field_sim` runs only the gz server with the generated world and the vehicle
  spawned into it. Captures pause the world and step it, which is the only way
  to read a sensor at a chosen pose before offboard control exists (Part 2).
* `field_sim_with_px4` additionally attaches PX4 and never pauses, because PX4
  runs in lockstep with the simulation clock. It is used for the ENU/NED
  mapping assertion that `docs/rules/sim_sdf.md` requires.
"""

from __future__ import annotations

import json
import logging
import math
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from pathlib import Path

import cv2
import pytest
from pymavlink import mavutil
from pyzbar.pyzbar import decode as zbar_decode

from aero.core.geo import Enu, LatLon, enu_to_ned, gps_from_enu, wrap_pi, yaw_enu_to_ned
from aero.sim.capture import SensorRig, SensorSnapshot, decode_qr_payloads, observe_pose, save_frame
from aero.sim.field import build_field
from aero.sim.gz_server import GzServer, GzServerConfig
from aero.sim.gz_truth import ModelPoseWatcher
from aero.sim.make_model import DEFAULT_MODELS_DIR, generate_model, topics
from aero.sim.make_world import GeneratedWorld, generate_world
from aero.sim.mavlink_probe import connection_string
from aero.sim.px4_sitl import Px4Sitl, SitlConfig
from aero.sim.world_params import WorldParams, load_world_params

logger = logging.getLogger("part1_sitl")

SEED = 1
CAPTURE_ALT_M = 5.0
SENSOR_TIMEOUT_S = 90.0
MAVLINK_MESSAGE_TIMEOUT_S = 20.0
# PX4 yaw includes the magnetic declination at the world's lat/lon, which the
# gz pose does not, so the two headings differ by a few degrees by design.
YAW_TOLERANCE_RAD = math.radians(12.0)
# Flight-based mapping check: tolerances cover EKF error plus the position the
# vehicle actually settles at, not the one it was asked for.
MAPPING_TOLERANCE_M = 1.5
TAKEOFF_ALT_M = 5.0
REPOSITION_NORTH_M = 8.0
ARM_TIMEOUT_S = 90.0
CLIMB_TIMEOUT_S = 90.0
REPOSITION_TIMEOUT_S = 120.0
# PX4 denies a reposition until the takeoff it is still flying has finished.
REPOSITION_ACCEPT_TIMEOUT_S = 60.0
# MAV_DO_REPOSITION param2 bit 0: "change to the mode needed for the move".
_REPOSITION_CHANGE_MODE = 1.0
FLIGHT_POLL_S = 0.5


@dataclass(frozen=True, slots=True)
class FieldSim:
    """A running world with the vehicle spawned and its sensors subscribed."""

    server: GzServer
    rig: SensorRig
    pose_watcher: ModelPoseWatcher
    world: GeneratedWorld
    params: WorldParams
    model_instance: str

    @property
    def spawn_yaw_rad(self) -> float:
        """Heading the vehicle was spawned with, gz convention."""
        return math.radians(self.params.vehicle.spawn_yaw_deg)

    def observe(self, xyz_m: tuple[float, float, float]) -> SensorSnapshot:
        """Read every sensor with the vehicle held at `xyz_m`."""
        return observe_pose(
            self.server,
            self.model_instance,
            xyz_m,
            yaw_rad=self.spawn_yaw_rad,
            rig=self.rig,
            pose_watcher=self.pose_watcher,
            timeout_s=SENSOR_TIMEOUT_S,
        )

    @property
    def start_qr_view(self) -> tuple[float, float, float]:
        """Observation pose directly above the start QR."""
        return (self.params.start_qr.offset_x_m, 0.0, CAPTURE_ALT_M)


@pytest.fixture(scope="module")
def params() -> WorldParams:
    """Checked-in world parameters."""
    return load_world_params()


@pytest.fixture(scope="module")
def generated(tmp_path_factory: pytest.TempPathFactory, params: WorldParams) -> GeneratedWorld:
    """World generated once for this module."""
    return generate_world(SEED, tmp_path_factory.mktemp("worlds"), params)


@pytest.fixture(scope="module")
def model_path(params: WorldParams) -> Path:
    """Vehicle model, regenerated so the test matches the current parameters."""
    return generate_model(DEFAULT_MODELS_DIR, params)


@pytest.fixture
def field_sim(
    generated: GeneratedWorld, model_path: Path, params: WorldParams, sitl_run_dir: Path
) -> Iterator[FieldSim]:
    """gz server with the generated world and the vehicle spawned into it."""
    config = GzServerConfig(
        world_path=generated.world_path, extra_resource_paths=(DEFAULT_MODELS_DIR,)
    )
    instance = f"{params.vehicle.model_name}_0"
    with GzServer(config, sitl_run_dir / "gz.log") as server:
        server.spawn_model(
            instance,
            model_path,
            (0.0, 0.0, params.vehicle.spawn_z_m),
            math.radians(params.vehicle.spawn_yaw_deg),
        )
        rig = SensorRig(topics(params.vehicle.model_name))
        watcher = ModelPoseWatcher(server.world_name, instance)
        try:
            yield FieldSim(
                server=server,
                rig=rig,
                pose_watcher=watcher,
                world=generated,
                params=params,
                model_instance=instance,
            )
        finally:
            rig.close()
            watcher.close()


@pytest.fixture
def field_sim_with_px4(
    field_sim: FieldSim, px4_dir: Path, sitl_run_dir: Path
) -> Iterator[tuple[FieldSim, Px4Sitl]]:
    """The field sim with PX4 attached to the already-spawned vehicle."""
    # PX4 is in lockstep with the simulation clock: it must never be attached
    # to a paused world.
    field_sim.server.set_paused(False)
    config = SitlConfig(
        px4_dir=px4_dir,
        world=field_sim.server.world_name,
        standalone_gz=True,
        gz_model_name=field_sim.model_instance,
    )
    with Px4Sitl(config, sitl_run_dir / "px4.log") as px4:
        yield field_sim, px4


@pytest.mark.sitl
def test_all_four_sensors_publish(field_sim: FieldSim) -> None:
    snapshot = field_sim.observe(field_sim.start_qr_view)
    params = field_sim.params
    assert snapshot.down_frame.width_px == params.camera_down.width_px
    assert snapshot.down_frame.height_px == params.camera_down.height_px
    assert snapshot.front_frame.width_px == params.camera_front.width_px
    assert snapshot.front_frame.height_px == params.camera_front.height_px
    # Hovering at 5 m the down beam must see the ground. The front beam looks
    # along the horizontal at 5 m, where the 3.05 m walls cannot reach it, so
    # "no return" there is the correct answer, not a failure.
    assert snapshot.down_range.valid is True
    assert snapshot.down_range.distance_m == pytest.approx(CAPTURE_ALT_M, abs=0.6)
    assert 0.0 <= snapshot.front_range.distance_m <= params.range_front.max_m


@pytest.mark.sitl
def test_front_range_sees_an_obstacle_in_the_return_lane(field_sim: FieldSim) -> None:
    """The front beam must measure a real obstacle, not just report `max`."""
    layout = build_field(field_sim.params, SEED)
    assert layout.obstacles, "seed has no obstacles to aim at"
    obstacle = layout.obstacles[0]
    obstacle_x, obstacle_y, _ = obstacle.center_m
    stand_off_m = 3.0
    # Line up with the block's own y so the narrow beam cannot pass beside it.
    snapshot = field_sim.observe((obstacle_x - stand_off_m, obstacle_y, 1.5))
    expected_m = stand_off_m - obstacle.size_m[0] / 2.0
    assert snapshot.front_range.valid is True
    assert snapshot.front_range.distance_m == pytest.approx(expected_m, abs=0.6)


@pytest.mark.sitl
def test_start_qr_decodes_from_the_down_camera(field_sim: FieldSim, sitl_run_dir: Path) -> None:
    snapshot = field_sim.observe(field_sim.start_qr_view)
    save_frame(sitl_run_dir / "down_start_qr.png", snapshot.down_frame)
    payloads = decode_qr_payloads(snapshot.down_frame)
    truth = json.loads(field_sim.world.truth_path.read_text(encoding="utf-8"))
    assert truth["target_payload"] in payloads, payloads


@pytest.mark.sitl
def test_down_camera_image_axes_match_the_body_frame(field_sim: FieldSim) -> None:
    """image-up is body forward and image-right is body right.

    Checked by displacing the vehicle from the start QR along one body axis at
    a time and watching where the QR lands in the frame. The vehicle faces +x
    (east), so north is its left.
    """
    params = field_sim.params
    qr_x, qr_y = params.start_qr.offset_x_m, 0.0
    offset_m = 1.0

    behind = _qr_centre_px(field_sim.observe((qr_x - offset_m, qr_y, CAPTURE_ALT_M)))
    north_of = _qr_centre_px(field_sim.observe((qr_x, qr_y + offset_m, CAPTURE_ALT_M)))

    width = params.camera_down.width_px
    height = params.camera_down.height_px
    # Vehicle behind the QR: the QR is ahead of it, so above the image centre.
    assert behind[1] < height / 2
    # Vehicle north of the QR: north is the vehicle's left, so the QR is to its
    # right and must appear right of the image centre.
    assert north_of[0] > width / 2


def _qr_centre_px(snapshot: SensorSnapshot) -> tuple[float, float]:
    """Pixel centre of the first QR found in a down frame."""
    found = zbar_decode(cv2.cvtColor(snapshot.down_frame.image_bgr, cv2.COLOR_BGR2GRAY))
    assert found, "no QR visible in the down frame"
    rect = found[0].rect
    return (rect.left + rect.width / 2.0, rect.top + rect.height / 2.0)


@pytest.mark.sitl
def test_gz_enu_pose_maps_to_px4_ned(field_sim_with_px4: tuple[FieldSim, Px4Sitl]) -> None:
    """The ENU->NED mapping in `aero.core.geo` matches the real PX4 + gz stack.

    The vehicle is flown, not teleported: PX4's estimator would reject a jump,
    and its local frame origin is wherever the EKF initialised. Comparing the
    *displacement* between two samples removes the origin from the question.

    The MAVLink calls here are deliberately minimal and local to this test;
    `core/flight_link.py` (Part 2) is the real implementation.
    """
    field_sim, _px4 = field_sim_with_px4
    params = field_sim.params
    watcher = ModelPoseWatcher(field_sim.server.world_name, field_sim.model_instance)
    port = SitlConfig().offboard_udp_port
    connection = mavutil.mavlink_connection(connection_string(port), source_system=255)
    try:
        connection.wait_heartbeat(timeout=MAVLINK_MESSAGE_TIMEOUT_S)
        watcher.wait_for_pose(MAVLINK_MESSAGE_TIMEOUT_S)

        _arm(connection)
        _take_off(connection, TAKEOFF_ALT_M)
        _wait_for_pose(
            connection,
            watcher,
            lambda pose: pose.up_m >= TAKEOFF_ALT_M - 1.0,
            f"climb to {TAKEOFF_ALT_M - 1.0} m",
            CLIMB_TIMEOUT_S,
        )
        first_enu, _ = watcher.read()
        first_ned = _latest(connection, "LOCAL_POSITION_NED")

        origin = LatLon(params.origin.lat_deg, params.origin.lon_deg)
        target = gps_from_enu(
            origin, Enu(east_m=first_enu.east_m, north_m=REPOSITION_NORTH_M, up_m=0.0)
        )
        _reposition(connection, target, TAKEOFF_ALT_M)
        wanted_north_m = first_enu.north_m + REPOSITION_NORTH_M / 2.0
        _wait_for_pose(
            connection,
            watcher,
            lambda pose: pose.north_m >= wanted_north_m,
            f"fly to {wanted_north_m:.1f} m north",
            REPOSITION_TIMEOUT_S,
        )
        second_enu, _ = watcher.read()
        second_ned = _latest(connection, "LOCAL_POSITION_NED")
        # Yaw is compared from one instant: PX4 turns the nose towards the
        # waypoint while repositioning, so a before/after pair would differ
        # for reasons that have nothing to do with the frame mapping.
        attitude = _latest(connection, "ATTITUDE")
        settled_enu_yaw_rad = watcher.read()[1]
    finally:
        watcher.close()
        connection.close()

    moved = Enu(
        east_m=second_enu.east_m - first_enu.east_m,
        north_m=second_enu.north_m - first_enu.north_m,
        up_m=second_enu.up_m - first_enu.up_m,
    )
    expected = enu_to_ned(moved)
    assert second_ned.x - first_ned.x == pytest.approx(expected.north_m, abs=MAPPING_TOLERANCE_M)
    assert second_ned.y - first_ned.y == pytest.approx(expected.east_m, abs=MAPPING_TOLERANCE_M)
    assert second_ned.z - first_ned.z == pytest.approx(expected.down_m, abs=MAPPING_TOLERANCE_M)
    # The vehicle really moved, so the comparison above is not trivially true.
    assert abs(moved.north_m) > REPOSITION_NORTH_M / 2.0

    assert abs(wrap_pi(attitude.yaw - yaw_enu_to_ned(settled_enu_yaw_rad))) < YAW_TOLERANCE_RAD


def _latest(connection: mavutil.mavfile, message_type: str) -> mavutil.mavlink.MAVLink_message:
    """Block for one freshly published message of `message_type`.

    The socket buffers everything PX4 sends while the test is waiting on
    something else, so a plain `recv_match` hands back a message from minutes
    ago. Draining first is what makes the sample current -- the same trap
    `docs/rules/pitfalls.md` describes for a shared MAVLink connection.
    """
    while connection.recv_match(blocking=False) is not None:
        pass
    message = connection.recv_match(
        type=message_type, blocking=True, timeout=MAVLINK_MESSAGE_TIMEOUT_S
    )
    assert message is not None, f"no {message_type} within {MAVLINK_MESSAGE_TIMEOUT_S} s"
    return message


def _send_gcs_heartbeat(connection: mavutil.mavfile) -> None:
    """Announce this connection as a GCS.

    PX4's arming checks include "connection to the GCS"; without heartbeats
    from us, arming is denied with "Resolve system health failures first".
    """
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
    )


def _command_long(
    connection: mavutil.mavfile, command: int, params: tuple[float, ...]
) -> int | None:
    """Send a COMMAND_LONG and return the result from its COMMAND_ACK.

    Args:
        connection: Open MAVLink connection.
        command: `MAV_CMD_*` identifier.
        params: The seven command parameters.

    Returns:
        The `MAV_RESULT` value, or None if no ack arrived.
    """
    connection.mav.command_long_send(
        connection.target_system, connection.target_component, command, 0, *params
    )
    ack = connection.recv_match(
        type="COMMAND_ACK", blocking=True, timeout=MAVLINK_MESSAGE_TIMEOUT_S
    )
    while ack is not None and ack.command != command:
        ack = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=MAVLINK_MESSAGE_TIMEOUT_S
        )
    return None if ack is None else int(ack.result)


def _autopilot_heartbeat(connection: mavutil.mavfile) -> mavutil.mavlink.MAVLink_message:
    """Block for a heartbeat from the autopilot itself.

    PX4 forwards heartbeats from several components on the onboard link, and
    their mode flags mean nothing for the vehicle. Filtering by source is the
    difference between "armed" and "some component set that bit".
    """
    deadline = time.monotonic() + MAVLINK_MESSAGE_TIMEOUT_S
    while time.monotonic() < deadline:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=FLIGHT_POLL_S)
        if (
            message is not None
            and message.get_srcSystem() == connection.target_system
            and message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        ):
            return message
    raise AssertionError(f"no autopilot heartbeat within {MAVLINK_MESSAGE_TIMEOUT_S} s")


def _arm(connection: mavutil.mavfile) -> None:
    """Arm the vehicle, retrying while PX4's health checks settle."""
    deadline = time.monotonic() + ARM_TIMEOUT_S
    last_result: int | None = None
    while time.monotonic() < deadline:
        _send_gcs_heartbeat(connection)
        heartbeat = _autopilot_heartbeat(connection)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return
        last_result = _command_long(
            connection, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1, 0, 0, 0, 0, 0, 0)
        )
        logger.info("arm_attempt base_mode=%d ack=%s", heartbeat.base_mode, last_result)
        time.sleep(FLIGHT_POLL_S)
    raise AssertionError(f"vehicle did not arm within {ARM_TIMEOUT_S} s (last ack {last_result})")


def _take_off(connection: mavutil.mavfile, altitude_m: float) -> None:
    """Command PX4's own takeoff to `altitude_m` above the current position."""
    global_position = _latest(connection, "GLOBAL_POSITION_INT")
    target_amsl_m = global_position.alt / 1000.0 + altitude_m
    nan = float("nan")
    deadline = time.monotonic() + ARM_TIMEOUT_S
    last_result: int | None = None
    while time.monotonic() < deadline:
        _send_gcs_heartbeat(connection)
        last_result = _command_long(
            connection,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            (0, 0, 0, nan, nan, nan, target_amsl_m),
        )
        logger.info("takeoff_attempt ack=%s", last_result)
        if last_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return
        time.sleep(FLIGHT_POLL_S)
    raise AssertionError(f"takeoff was not accepted within {ARM_TIMEOUT_S} s (ack {last_result})")


def _reposition(connection: mavutil.mavfile, target: LatLon, altitude_m: float) -> None:
    """Command PX4 to fly to `target` at `altitude_m` above home.

    PX4 denies a reposition while AUTO.TAKEOFF is still running, so the
    command is retried until it is accepted.

    Args:
        connection: Open MAVLink connection.
        target: Destination latitude/longitude.
        altitude_m: Altitude above home, metres.

    Raises:
        AssertionError: PX4 never accepted the command.
    """
    deadline = time.monotonic() + REPOSITION_ACCEPT_TIMEOUT_S
    last_result: int | None = None
    while time.monotonic() < deadline:
        _send_gcs_heartbeat(connection)
        connection.mav.command_int_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            0,
            -1.0,
            # param2 bit 0 asks PX4 to switch to AUTO.LOITER for the move;
            # without it PX4 1.18 answers UNSUPPORTED (Commander.cpp).
            _REPOSITION_CHANGE_MODE,
            0,
            float("nan"),
            int(target.lat_deg * 1e7),
            int(target.lon_deg * 1e7),
            altitude_m,
        )
        ack = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=MAVLINK_MESSAGE_TIMEOUT_S
        )
        if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_DO_REPOSITION:
            last_result = int(ack.result)
            logger.info("reposition_attempt ack=%s", last_result)
            if last_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return
        time.sleep(FLIGHT_POLL_S)
    raise AssertionError(
        f"reposition was not accepted within {REPOSITION_ACCEPT_TIMEOUT_S} s (ack {last_result})"
    )


def _wait_for_pose(
    connection: mavutil.mavfile,
    watcher: ModelPoseWatcher,
    reached: Callable[[Enu], bool],
    what: str,
    timeout_s: float,
) -> None:
    """Block until the ground-truth pose satisfies `reached`.

    GCS heartbeats keep flowing meanwhile: PX4 would otherwise start a
    GCS-loss failsafe in the middle of the manoeuvre.

    Args:
        connection: Open MAVLink connection, used for the heartbeats.
        watcher: Ground-truth pose source.
        reached: Predicate on the vehicle position.
        what: Description for the failure message.
        timeout_s: How long to wait.

    Raises:
        AssertionError: The predicate never held.
    """
    deadline = time.monotonic() + timeout_s
    position = Enu(0.0, 0.0, 0.0)
    while time.monotonic() < deadline:
        _send_gcs_heartbeat(connection)
        position, _ = watcher.read()
        if reached(position):
            return
        time.sleep(FLIGHT_POLL_S)
    raise AssertionError(f"vehicle did not {what} within {timeout_s} s (last pose {position})")

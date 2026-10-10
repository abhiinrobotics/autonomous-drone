"""Sensor capture against a generated world, without flying.

Part 1 has to prove that the vehicle's four sensors work and that the start QR
is decodable from the air, but autonomous flight only arrives with offboard
control in Part 2. So the vehicle is teleported to the observation pose with
gz's `set_pose` — which only gives a usable frame if the simulation is
**paused** around the move: a teleported dynamic model keeps the velocity
gravity has already given it, so with physics running it tumbles away and the
frame shows sky or the inside of the ground.

The capture therefore pauses the world, sets the pose, and steps the
simulation just far enough for every sensor to publish a fresh message. Never
use this while PX4 is attached: PX4 runs in lockstep with the simulation clock.

This is simulator tooling: nothing here may be used by mission code.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
from pyzbar.pyzbar import decode as zbar_decode

from aero.core.errors import SensorTimeout
from aero.core.geo import Enu
from aero.hal.interfaces import CameraFrame, RangeReading
from aero.hal.sitl.gz_camera import GzCamera
from aero.hal.sitl.gz_range import GzRangeSensor
from aero.sim.gz_server import GzServer
from aero.sim.gz_truth import ModelPoseWatcher
from aero.sim.make_model import SensorTopics

logger = logging.getLogger(__name__)

# A paused simulation publishes only while it is being stepped, so wall-clock
# freshness limits do not apply to capture: the rig accepts whatever the last
# step produced. Mission code keeps the drivers' own tight limits.
CAPTURE_MAX_AGE_S = 600.0
# gz applies a `set_pose` on a later simulation update, not when the service
# replies, so a capture needs two stepped intervals: the first lets the
# teleport take effect (and is checked against the ground-truth pose), the
# second produces the sensor data that is actually read. Without the check,
# a frame from *before* the teleport can be returned.
SETTLE_STEPS = 10
POSE_TOLERANCE_M = 0.4
# The generated worlds step at 4 ms, so 25 steps is 0.1 s of simulated time:
# two frames from the slowest sensor (the 20 Hz down camera), and only ~5 cm
# of fall between the teleport and the read.
STEPS_PER_ATTEMPT = 25
# Wall-clock patience for the sensor messages produced by those steps.
MESSAGE_WAIT_S = 2.0
# Poll period while waiting for the stepped messages to arrive.
_SEQUENCE_POLL_S = 0.02


@dataclass(frozen=True, slots=True)
class SensorSnapshot:
    """One synchronous reading of every sensor.

    Args:
        down_frame: Latest down-camera frame.
        front_frame: Latest front-camera frame.
        front_range: Latest front rangefinder reading.
        down_range: Latest down rangefinder reading.
    """

    down_frame: CameraFrame
    front_frame: CameraFrame
    front_range: RangeReading
    down_range: RangeReading


class SensorRig:
    """The four SITL sensors of one spawned vehicle, opened together."""

    def __init__(self, topics: SensorTopics, max_age_s: float = CAPTURE_MAX_AGE_S) -> None:
        """Subscribe to every sensor topic of the model.

        Args:
            topics: Topic names from `aero.sim.make_model.topics`.
            max_age_s: Freshness limit for the drivers. The default is
                deliberately long because a paused world only publishes while
                it is stepped; `observe_pose` guarantees freshness instead, by
                requiring every sensor's message counter to advance.

        Raises:
            SensorTimeout: A subscription was refused.
        """
        self.down_camera = GzCamera(topics.down_image, max_age_s)
        self.front_camera = GzCamera(topics.front_image, max_age_s)
        self.front_range = GzRangeSensor(topics.front_scan, max_age_s)
        self.down_range = GzRangeSensor(topics.down_scan, max_age_s)

    def wait_until_streaming(self, timeout_s: float) -> None:
        """Block until every sensor has produced at least one message.

        Args:
            timeout_s: Budget shared by all four sensors.

        Raises:
            SensorTimeout: A sensor stayed silent.
        """
        deadline = time.monotonic() + timeout_s
        for camera in (self.down_camera, self.front_camera):
            camera.wait_for_first_frame(max(0.1, deadline - time.monotonic()))
        for sensor in (self.front_range, self.down_range):
            sensor.wait_for_first_sample(max(0.1, deadline - time.monotonic()))

    def sequences(self) -> tuple[int, int, int, int]:
        """Message counters of the four sensors, in rig order."""
        return (
            self.down_camera.sequence,
            self.front_camera.sequence,
            self.front_range.sequence,
            self.down_range.sequence,
        )

    def wait_for_new_messages(self, previous: tuple[int, int, int, int], timeout_s: float) -> None:
        """Block until every sensor has published past `previous`.

        Args:
            previous: Counters from an earlier `sequences()` call.
            timeout_s: How long to wait.

        Raises:
            SensorTimeout: A sensor published nothing new in time.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if all(now > before for now, before in zip(self.sequences(), previous, strict=True)):
                return
            time.sleep(_SEQUENCE_POLL_S)
        stalled = [
            name
            for name, now, before in zip(
                ("down_camera", "front_camera", "front_range", "down_range"),
                self.sequences(),
                previous,
                strict=True,
            )
            if now <= before
        ]
        raise SensorTimeout(f"no new messages from {stalled} within {timeout_s:.1f} s")

    def snapshot(self) -> SensorSnapshot:
        """Read every sensor once.

        Returns:
            The four latest readings.

        Raises:
            SensorTimeout: A sensor has no fresh data.
        """
        return SensorSnapshot(
            down_frame=self.down_camera.read(),
            front_frame=self.front_camera.read(),
            front_range=self.front_range.read(),
            down_range=self.down_range.read(),
        )

    def close(self) -> None:
        """Close every subscription. Safe to call more than once."""
        for sensor in (self.down_camera, self.front_camera, self.front_range, self.down_range):
            sensor.close()


def observe_pose(
    server: GzServer,
    model_name: str,
    xyz_m: tuple[float, float, float],
    *,
    yaw_rad: float,
    rig: SensorRig,
    pose_watcher: ModelPoseWatcher,
    timeout_s: float,
) -> SensorSnapshot:
    """Place the vehicle at an observation pose and read every sensor there.

    The world is paused for the whole call. The pose is verified against the
    simulator's own ground truth before the data is taken, so the returned
    readings always belong to `xyz_m` rather than to wherever the vehicle was
    before the teleport.

    Args:
        server: Running gz server (must not have PX4 attached).
        model_name: Spawned model instance name.
        xyz_m: Observation pose in world ENU metres.
        yaw_rad: Observation heading about +z.
        rig: Sensors to read.
        pose_watcher: Ground-truth pose source for the same model.
        timeout_s: Overall budget.

    Returns:
        One reading of every sensor, taken at the observation pose.

    Raises:
        SensorTimeout: The vehicle never settled at the pose, or the sensors
            produced no fresh data in time.
    """
    deadline = time.monotonic() + timeout_s
    server.set_paused(True)
    last_error: SensorTimeout | None = None
    while time.monotonic() < deadline:
        server.set_model_pose(model_name, xyz_m, yaw_rad)
        server.step(SETTLE_STEPS)
        try:
            pose_watcher.wait_for_pose(MESSAGE_WAIT_S)
            position, _ = pose_watcher.read()
        except SensorTimeout as error:
            last_error = error
            continue
        if not _is_at(position, xyz_m):
            last_error = SensorTimeout(
                f"{model_name!r} is at {position} instead of {xyz_m}"
            )
            continue
        before = rig.sequences()
        server.step(STEPS_PER_ATTEMPT)
        try:
            rig.wait_for_new_messages(before, MESSAGE_WAIT_S)
        except SensorTimeout as error:
            last_error = error
            continue
        return rig.snapshot()
    raise SensorTimeout(
        f"could not observe {model_name!r} at {xyz_m} within {timeout_s:.0f} s: {last_error}"
    )


def _is_at(position: Enu, xyz_m: tuple[float, float, float]) -> bool:
    """Whether a ground-truth position matches a requested pose."""
    east_m, north_m, up_m = xyz_m
    return (
        abs(position.east_m - east_m) <= POSE_TOLERANCE_M
        and abs(position.north_m - north_m) <= POSE_TOLERANCE_M
        and abs(position.up_m - up_m) <= POSE_TOLERANCE_M
    )


def decode_qr_payloads(frame: CameraFrame) -> list[str]:
    """Decode every QR code in a frame.

    Decoding runs on the grayscale image, which is what the Part 3 vision
    pipeline will enhance; here it is deliberately plain, so the test proves
    the *world* is readable rather than that the enhancement is good.

    Args:
        frame: Camera frame to decode.

    Returns:
        Payload strings found, in the order pyzbar reports them.
    """
    grayscale = cv2.cvtColor(frame.image_bgr, cv2.COLOR_BGR2GRAY)
    return [item.data.decode("utf-8", errors="replace") for item in zbar_decode(grayscale)]


def save_frame(path: Path, frame: CameraFrame) -> None:
    """Write a frame to `path` as PNG.

    Args:
        path: Destination file.
        frame: Frame to write.

    Raises:
        OSError: The image could not be written.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), frame.image_bgr):
        raise OSError(f"could not write frame to {path}")

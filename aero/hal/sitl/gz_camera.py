"""SITL camera driver: a Gazebo image topic behind the `Camera` interface."""

from __future__ import annotations

import logging

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image

from aero.core.errors import ConfigError, SensorTimeout
from aero.hal.interfaces import BgrImage, Camera, CameraFrame
from aero.hal.sitl.gz_subscriber import GzSubscriber

logger = logging.getLogger(__name__)

# gz pixel formats we can decode, from gz.msgs.PixelFormatType. The model
# generator asks for R8G8B8, so RGB_INT8 is the expected one; the others are
# accepted because a changed SDF should not silently break the driver.
_RGB_INT8 = 3
_RGBA_INT8 = 4
_BGRA_INT8 = 5
_BGR_INT8 = 8
_L_INT8 = 1
_CHANNELS_BY_FORMAT = {_L_INT8: 1, _RGB_INT8: 3, _BGR_INT8: 3, _RGBA_INT8: 4, _BGRA_INT8: 4}
# Default freshness limit: a camera running at 20 Hz may skip a couple of
# frames while the renderer catches up, but anything older is not "now".
DEFAULT_MAX_AGE_S = 1.0


def decode_image(message: Image) -> BgrImage:
    """Convert a gz `Image` message to an 8-bit BGR array.

    Args:
        message: Image message as received from gz.

    Returns:
        BGR image of shape (height, width, 3).

    Raises:
        ConfigError: The pixel format is one we cannot decode, or the payload
            size does not match the declared geometry.
    """
    channels = _CHANNELS_BY_FORMAT.get(message.pixel_format_type)
    if channels is None:
        raise ConfigError(
            f"unsupported gz pixel_format_type {message.pixel_format_type}; "
            f"the model SDF should request R8G8B8"
        )
    expected_bytes = message.width * message.height * channels
    if len(message.data) != expected_bytes:
        raise ConfigError(
            f"gz image payload is {len(message.data)} bytes but "
            f"{message.width}x{message.height}x{channels} needs {expected_bytes}"
        )
    flat = np.frombuffer(message.data, dtype=np.uint8)
    image = flat.reshape(message.height, message.width, channels)
    if message.pixel_format_type == _L_INT8:
        return np.asarray(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), dtype=np.uint8)
    if message.pixel_format_type == _RGB_INT8:
        return np.asarray(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), dtype=np.uint8)
    if message.pixel_format_type == _RGBA_INT8:
        return np.asarray(cv2.cvtColor(image, cv2.COLOR_RGBA2BGR), dtype=np.uint8)
    if message.pixel_format_type == _BGRA_INT8:
        return np.asarray(cv2.cvtColor(image, cv2.COLOR_BGRA2BGR), dtype=np.uint8)
    return np.asarray(image, dtype=np.uint8)


class GzCamera(Camera):
    """Latest-frame camera backed by a gz image topic."""

    def __init__(self, topic: str, max_age_s: float = DEFAULT_MAX_AGE_S) -> None:
        """Subscribe to a gz image topic.

        Args:
            topic: Full gz topic name, e.g. `/aero_x500/down_cam/image`.
            max_age_s: Oldest frame `read()` will still return.

        Raises:
            SensorTimeout: The subscription was refused.
        """
        self._subscriber: GzSubscriber[Image] | None = GzSubscriber(topic, Image, max_age_s)
        self._topic = topic

    @property
    def sequence(self) -> int:
        """Number of messages received so far (0 before the first)."""
        subscriber = self._subscriber
        return 0 if subscriber is None else subscriber.sequence

    @property
    def topic(self) -> str:
        """gz topic this camera reads."""
        return self._topic

    def wait_for_first_frame(self, timeout_s: float) -> None:
        """Block until the camera has produced a frame.

        Args:
            timeout_s: How long to wait.

        Raises:
            SensorTimeout: No frame arrived in time, or the camera is closed.
        """
        self._require_open().wait_for_message(timeout_s)

    def read(self) -> CameraFrame:
        """Return the latest frame decoded to BGR.

        Returns:
            The most recent frame.

        Raises:
            SensorTimeout: No fresh frame, or the camera is closed.
            ConfigError: The frame cannot be decoded.
        """
        message, received_s, sequence = self._require_open().latest()
        return CameraFrame(
            image_bgr=decode_image(message), timestamp_s=received_s, sequence=sequence
        )

    def close(self) -> None:
        """Drop the subscription. Safe to call more than once."""
        if self._subscriber is not None:
            self._subscriber.close()
            self._subscriber = None

    def _require_open(self) -> GzSubscriber[Image]:
        """Return the subscriber or explain that the camera is closed.

        Raises:
            SensorTimeout: The camera has been closed.
        """
        if self._subscriber is None:
            raise SensorTimeout(f"camera on {self._topic!r} is closed")
        return self._subscriber

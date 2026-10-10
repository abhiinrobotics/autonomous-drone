"""Sensor interfaces the mission depends on, and the data they return.

Only these types may appear in phase and mission code. Each implementation
(SITL via gz topics, hardware via the Pi's cameras and UART rangefinders) is
constructed once at start-up and injected.

Timing: every reading carries the `time.monotonic()` value at which it was
received, so phases can pair a frame with the vehicle state of that instant and
reject stale data. Never use wall-clock time for these intervals.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from types import TracebackType

import numpy as np
import numpy.typing as npt

# 8-bit BGR, the convention OpenCV and the vision code use.
BgrImage = npt.NDArray[np.uint8]


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """One camera image with the time it arrived.

    Args:
        image_bgr: Image as 8-bit BGR, shape (height, width, 3).
        timestamp_s: `time.monotonic()` when the frame was received.
        sequence: Monotonically increasing counter of frames from this camera.
    """

    image_bgr: BgrImage
    timestamp_s: float
    sequence: int

    @property
    def width_px(self) -> int:
        """Image width in pixels."""
        return int(self.image_bgr.shape[1])

    @property
    def height_px(self) -> int:
        """Image height in pixels."""
        return int(self.image_bgr.shape[0])


@dataclass(frozen=True, slots=True)
class RangeReading:
    """One rangefinder sample.

    `valid` is False when the beam saw nothing inside the sensor's range. The
    distance is then clamped to `max_m`, so callers that ignore `valid` at
    least do not read a small distance where there is no obstacle — but phase
    code must check `valid`, never assume "no data means clear"
    (docs/rules/engineering_standards.md).

    Args:
        distance_m: Measured distance in metres.
        timestamp_s: `time.monotonic()` when the sample was received.
        valid: Whether the sample is a real return inside the sensor's range.
    """

    distance_m: float
    timestamp_s: float
    valid: bool


class Camera(ABC):
    """A camera that always offers its most recent frame."""

    @abstractmethod
    def read(self) -> CameraFrame:
        """Return the latest frame.

        Returns:
            The most recent frame received.

        Raises:
            SensorTimeout: No frame has arrived, or the newest one is older
                than this camera's allowed age.
        """

    @abstractmethod
    def close(self) -> None:
        """Release the camera. Safe to call more than once."""

    def __enter__(self) -> Camera:
        """Return self so cameras can be used as context managers."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the camera."""
        self.close()


class RangeSensor(ABC):
    """A single-beam rangefinder that always offers its most recent sample."""

    @abstractmethod
    def read(self) -> RangeReading:
        """Return the latest sample.

        Returns:
            The most recent reading received.

        Raises:
            SensorTimeout: No sample has arrived, or the newest one is older
                than this sensor's allowed age.
        """

    @abstractmethod
    def close(self) -> None:
        """Release the sensor. Safe to call more than once."""

    def __enter__(self) -> RangeSensor:
        """Return self so sensors can be used as context managers."""
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        """Close the sensor."""
        self.close()

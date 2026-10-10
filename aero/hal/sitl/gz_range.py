"""SITL rangefinder driver: a narrow gz lidar behind the `RangeSensor` interface.

A real TFmini Plus / TF-Luna reports a single distance over a small cone. The
model emulates that with a small grid of lidar rays, so this driver reduces the
scan to one distance by taking the **minimum** valid return: for obstacle
avoidance the nearest thing in the beam is what matters.
"""

from __future__ import annotations

import logging
import math

from gz.msgs10.laserscan_pb2 import LaserScan

from aero.core.errors import SensorTimeout
from aero.hal.interfaces import RangeReading, RangeSensor
from aero.hal.sitl.gz_subscriber import GzSubscriber

logger = logging.getLogger(__name__)

# A 100 Hz sensor that has not spoken for this long is not reporting "now".
DEFAULT_MAX_AGE_S = 0.5


def reduce_scan(scan: LaserScan) -> tuple[float, bool]:
    """Reduce a narrow lidar scan to one distance.

    gz reports `inf` for rays that hit nothing within range, and may report
    values slightly outside `[range_min, range_max]`; both are treated as "no
    return".

    Args:
        scan: LaserScan message from gz.

    Returns:
        (distance_m, valid). When no ray returned, the distance is the sensor's
        maximum range and `valid` is False.
    """
    nearest_m = math.inf
    for value in scan.ranges:
        if not math.isfinite(value):
            continue
        if value < scan.range_min or value > scan.range_max:
            continue
        nearest_m = min(nearest_m, float(value))
    if math.isinf(nearest_m):
        return float(scan.range_max), False
    return nearest_m, True


class GzRangeSensor(RangeSensor):
    """Latest-sample rangefinder backed by a gz LaserScan topic."""

    def __init__(self, topic: str, max_age_s: float = DEFAULT_MAX_AGE_S) -> None:
        """Subscribe to a gz LaserScan topic.

        Args:
            topic: Full gz topic name, e.g. `/aero_x500/down_range/scan`.
            max_age_s: Oldest sample `read()` will still return.

        Raises:
            SensorTimeout: The subscription was refused.
        """
        self._subscriber: GzSubscriber[LaserScan] | None = GzSubscriber(topic, LaserScan, max_age_s)
        self._topic = topic

    @property
    def sequence(self) -> int:
        """Number of messages received so far (0 before the first)."""
        subscriber = self._subscriber
        return 0 if subscriber is None else subscriber.sequence

    @property
    def topic(self) -> str:
        """gz topic this sensor reads."""
        return self._topic

    def wait_for_first_sample(self, timeout_s: float) -> None:
        """Block until the sensor has produced a sample.

        Args:
            timeout_s: How long to wait.

        Raises:
            SensorTimeout: Nothing arrived in time, or the sensor is closed.
        """
        self._require_open().wait_for_message(timeout_s)

    def read(self) -> RangeReading:
        """Return the latest reading.

        Returns:
            The most recent sample, with `valid=False` when the beam saw
            nothing inside the sensor's range.

        Raises:
            SensorTimeout: No fresh sample, or the sensor is closed.
        """
        message, received_s, _ = self._require_open().latest()
        distance_m, valid = reduce_scan(message)
        return RangeReading(distance_m=distance_m, timestamp_s=received_s, valid=valid)

    def close(self) -> None:
        """Drop the subscription. Safe to call more than once."""
        if self._subscriber is not None:
            self._subscriber.close()
            self._subscriber = None

    def _require_open(self) -> GzSubscriber[LaserScan]:
        """Return the subscriber or explain that the sensor is closed.

        Raises:
            SensorTimeout: The sensor has been closed.
        """
        if self._subscriber is None:
            raise SensorTimeout(f"range sensor on {self._topic!r} is closed")
        return self._subscriber

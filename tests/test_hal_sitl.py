"""Unit tests for the SITL sensor drivers' pure decoding paths.

These build gz protobuf messages by hand, so no simulator is needed.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan

from aero.core.errors import ConfigError
from aero.hal.interfaces import CameraFrame, RangeReading
from aero.hal.sitl.gz_camera import decode_image
from aero.hal.sitl.gz_range import reduce_scan

_RGB_INT8 = 3
_BGR_INT8 = 8
_L_INT8 = 1
_RGBA_INT8 = 4


def _image(width: int, height: int, pixel_format: int, payload: bytes) -> Image:
    """Build a gz Image message."""
    message = Image()
    message.width = width
    message.height = height
    message.pixel_format_type = pixel_format
    message.data = payload
    return message


def _scan(ranges: list[float], range_min: float = 0.1, range_max: float = 12.0) -> LaserScan:
    """Build a gz LaserScan message."""
    message = LaserScan()
    message.range_min = range_min
    message.range_max = range_max
    message.count = len(ranges)
    message.ranges.extend(ranges)
    return message


def test_decode_rgb_image_swaps_channels_to_bgr() -> None:
    # One red pixel in RGB must come back as BGR (0, 0, 255).
    image = decode_image(_image(1, 1, _RGB_INT8, bytes([255, 0, 0])))
    assert image.shape == (1, 1, 3)
    assert list(image[0, 0]) == [0, 0, 255]


def test_decode_bgr_image_is_passed_through() -> None:
    image = decode_image(_image(1, 1, _BGR_INT8, bytes([255, 0, 0])))
    assert list(image[0, 0]) == [255, 0, 0]


def test_decode_grayscale_image_expands_to_three_channels() -> None:
    image = decode_image(_image(2, 1, _L_INT8, bytes([10, 200])))
    assert image.shape == (1, 2, 3)
    assert list(image[0, 0]) == [10, 10, 10]
    assert list(image[0, 1]) == [200, 200, 200]


def test_decode_rgba_image_drops_alpha() -> None:
    image = decode_image(_image(1, 1, _RGBA_INT8, bytes([255, 0, 0, 128])))
    assert image.shape == (1, 1, 3)
    assert list(image[0, 0]) == [0, 0, 255]


def test_decode_preserves_geometry() -> None:
    payload = bytes(range(4 * 3 * 3))
    image = decode_image(_image(4, 3, _RGB_INT8, payload))
    assert image.shape == (3, 4, 3)


def test_unsupported_pixel_format_is_a_config_error() -> None:
    with pytest.raises(ConfigError, match="unsupported gz pixel_format_type"):
        decode_image(_image(1, 1, 14, bytes(12)))


def test_truncated_payload_is_a_config_error() -> None:
    with pytest.raises(ConfigError, match="payload is 2 bytes"):
        decode_image(_image(4, 4, _RGB_INT8, bytes(2)))


def test_reduce_scan_takes_the_nearest_valid_return() -> None:
    distance_m, valid = reduce_scan(_scan([5.0, 2.5, 7.0]))
    assert valid is True
    assert distance_m == pytest.approx(2.5)


def test_reduce_scan_ignores_infinite_rays() -> None:
    distance_m, valid = reduce_scan(_scan([math.inf, 4.0, math.inf]))
    assert valid is True
    assert distance_m == pytest.approx(4.0)


def test_reduce_scan_reports_no_return_when_nothing_is_in_range() -> None:
    # "No return" must never look like a close obstacle, and must be flagged
    # invalid so phase code cannot read it as "clear" by accident.
    distance_m, valid = reduce_scan(_scan([math.inf, math.inf], range_max=12.0))
    assert valid is False
    assert distance_m == pytest.approx(12.0)


def test_reduce_scan_rejects_values_outside_the_sensor_range() -> None:
    distance_m, valid = reduce_scan(_scan([0.05, 13.0, 6.0], range_min=0.1, range_max=12.0))
    assert valid is True
    assert distance_m == pytest.approx(6.0)


def test_reduce_scan_handles_nan() -> None:
    distance_m, valid = reduce_scan(_scan([math.nan, 3.0]))
    assert valid is True
    assert distance_m == pytest.approx(3.0)


def test_reduce_scan_with_no_rays_is_invalid() -> None:
    distance_m, valid = reduce_scan(_scan([], range_max=8.0))
    assert valid is False
    assert distance_m == pytest.approx(8.0)


def test_camera_frame_reports_its_geometry() -> None:
    frame = CameraFrame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8), timestamp_s=1.0, sequence=3
    )
    assert frame.width_px == 640
    assert frame.height_px == 480
    assert frame.sequence == 3


def test_range_reading_is_immutable() -> None:
    reading = RangeReading(distance_m=1.0, timestamp_s=2.0, valid=True)
    with pytest.raises(AttributeError):
        reading.distance_m = 2.0  # type: ignore[misc]

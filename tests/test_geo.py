"""Unit tests for frame conversions and geodesy, including sign conventions."""

from __future__ import annotations

import math

import pytest

from aero.core.geo import (
    EARTH_RADIUS_M,
    Enu,
    LatLon,
    Ned,
    enu_from_gps,
    enu_to_ned,
    frd_to_gz_pose,
    gps_from_enu,
    haversine_m,
    ned_to_enu,
    point_in_polygon,
    rect_corners,
    wrap_pi,
    yaw_enu_to_ned,
    yaw_ned_to_enu,
)

# PX4's default SITL home (Zurich), used by PX4's own worlds.
_ORIGIN = LatLon(lat_deg=47.397971, lon_deg=8.546164)


@pytest.mark.parametrize(
    ("angle_rad", "expected"),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),
        (3 * math.pi, math.pi),
        (math.tau, 0.0),
        (math.pi / 2, math.pi / 2),
        (-math.pi / 2, -math.pi / 2),
        (1.5 * math.pi, -math.pi / 2),
    ],
)
def test_wrap_pi(angle_rad: float, expected: float) -> None:
    assert wrap_pi(angle_rad) == pytest.approx(expected)


def test_enu_to_ned_swaps_axes_and_flips_z() -> None:
    # 10 m east, 20 m north, 5 m up  ->  20 m north, 10 m east, 5 m down.
    assert enu_to_ned(Enu(east_m=10.0, north_m=20.0, up_m=5.0)) == Ned(
        north_m=20.0, east_m=10.0, down_m=-5.0
    )


def test_ned_to_enu_is_the_inverse() -> None:
    original = Enu(east_m=-3.5, north_m=7.25, up_m=12.0)
    assert ned_to_enu(enu_to_ned(original)) == original


def test_yaw_enu_zero_is_east_which_is_px4_ninety_degrees() -> None:
    # Verified against live SITL: a model spawned with gz yaw 0 faces east and
    # PX4 reports yaw ~ +pi/2 (plus magnetic declination).
    assert yaw_enu_to_ned(0.0) == pytest.approx(math.pi / 2)


@pytest.mark.parametrize(
    ("yaw_enu_rad", "yaw_ned_rad"),
    [
        (0.0, math.pi / 2),  # east
        (math.pi / 2, 0.0),  # north
        (math.pi, -math.pi / 2),  # west
        (-math.pi / 2, math.pi),  # south
    ],
)
def test_yaw_conversions_round_trip(yaw_enu_rad: float, yaw_ned_rad: float) -> None:
    assert yaw_enu_to_ned(yaw_enu_rad) == pytest.approx(yaw_ned_rad)
    assert yaw_ned_to_enu(yaw_ned_rad) == pytest.approx(yaw_enu_rad)


def test_gps_from_enu_moves_north_and_east_in_the_right_direction() -> None:
    moved = gps_from_enu(_ORIGIN, Enu(east_m=100.0, north_m=200.0, up_m=0.0))
    assert moved.lat_deg > _ORIGIN.lat_deg
    assert moved.lon_deg > _ORIGIN.lon_deg
    # 200 m north is 200/R radians of latitude.
    assert moved.lat_deg - _ORIGIN.lat_deg == pytest.approx(math.degrees(200.0 / EARTH_RADIUS_M))


def test_gps_from_enu_ignores_altitude() -> None:
    flat = gps_from_enu(_ORIGIN, Enu(east_m=5.0, north_m=5.0, up_m=0.0))
    high = gps_from_enu(_ORIGIN, Enu(east_m=5.0, north_m=5.0, up_m=50.0))
    assert flat == high


def test_enu_from_gps_round_trips() -> None:
    original = Enu(east_m=-42.0, north_m=137.0, up_m=0.0)
    point = gps_from_enu(_ORIGIN, original)
    recovered = enu_from_gps(_ORIGIN, point)
    assert recovered.east_m == pytest.approx(original.east_m, abs=1e-6)
    assert recovered.north_m == pytest.approx(original.north_m, abs=1e-6)


def test_haversine_matches_the_local_offset_for_short_distances() -> None:
    point = gps_from_enu(_ORIGIN, Enu(east_m=0.0, north_m=150.0, up_m=0.0))
    assert haversine_m(_ORIGIN, point) == pytest.approx(150.0, abs=0.05)


def test_haversine_is_zero_for_identical_points() -> None:
    assert haversine_m(_ORIGIN, _ORIGIN) == pytest.approx(0.0)


_SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


@pytest.mark.parametrize(
    ("point", "expected"),
    [
        ((5.0, 5.0), True),
        ((0.5, 0.5), True),
        ((-1.0, 5.0), False),
        ((11.0, 5.0), False),
        ((5.0, -0.1), False),
        ((5.0, 10.1), False),
    ],
)
def test_point_in_polygon_square(point: tuple[float, float], expected: bool) -> None:
    assert point_in_polygon(point, _SQUARE) is expected


def test_point_in_polygon_concave_shape() -> None:
    # An L shape: the notch must read as outside.
    shape = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (4.0, 4.0), (4.0, 10.0), (0.0, 10.0)]
    assert point_in_polygon((2.0, 8.0), shape) is True
    assert point_in_polygon((8.0, 8.0), shape) is False


def test_point_in_polygon_vertex_order_does_not_matter() -> None:
    assert point_in_polygon((5.0, 5.0), list(reversed(_SQUARE))) is True


def test_point_in_polygon_rejects_degenerate_polygon() -> None:
    with pytest.raises(ValueError, match=">= 3 vertices"):
        point_in_polygon((0.0, 0.0), [(0.0, 0.0), (1.0, 1.0)])


def test_rect_corners_are_centred() -> None:
    corners = rect_corners(center_x=2.0, center_y=-1.0, size_x=4.0, size_y=2.0)
    assert corners == [(0.0, -2.0), (4.0, -2.0), (4.0, 0.0), (0.0, 0.0)]
    assert point_in_polygon((2.0, -1.0), corners) is True


def test_frd_to_gz_pose_mirrors_y_and_z() -> None:
    xyz, rpy = frd_to_gz_pose((0.15, 0.2, 0.1), (0.0, 0.0, 0.0))
    assert xyz == (0.15, -0.2, -0.1)
    assert rpy == (0.0, 0.0, 0.0)


def test_frd_down_facing_sensor_becomes_gz_pitch_plus_ninety() -> None:
    # PX4's own x500_mono_cam_down mounts its down camera at gz pitch +1.5707.
    _, rpy = frd_to_gz_pose((0.0, 0.0, 0.1), (0.0, -math.pi / 2, 0.0))
    assert rpy[1] == pytest.approx(math.pi / 2)


def test_frd_to_gz_pose_flips_yaw_sign() -> None:
    # A sensor yawed to the right in FRD is yawed negatively in gz (y is left).
    _, rpy = frd_to_gz_pose((0.0, 0.0, 0.0), (0.1, 0.2, 0.3))
    assert rpy == pytest.approx((0.1, -0.2, -0.3))

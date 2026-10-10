"""Frame conversions and geodesy. Pure functions only — no I/O, no globals.

Frames used across the project:

* **ENU** (Gazebo world): x east, y north, z up. Angles measured from +x_east,
  positive counter-clockwise (towards north).
* **NED** (PX4 local position): x north, y east, z down. Yaw measured from
  north, positive clockwise (towards east).
* **Body FRD**: x forward, y right, z down.
* **Image**: u right, v down, origin top-left.

The ENU/NED mapping and the yaw relation are asserted against a live PX4 SITL
instance in `tests/test_part1_sitl.py`, as required by `docs/rules/sim_sdf.md`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# PX4 converts between global and local positions with a spherical earth of this
# radius (`CONSTANTS_RADIUS_OF_EARTH` in PX4's geo.h). Using the same value
# keeps our GPS<->local conversions consistent with the autopilot's.
EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True, slots=True)
class Enu:
    """Position in the Gazebo world frame, metres.

    Args:
        east_m: Along +x of the gz world.
        north_m: Along +y of the gz world.
        up_m: Along +z of the gz world.
    """

    east_m: float
    north_m: float
    up_m: float


@dataclass(frozen=True, slots=True)
class Ned:
    """Position in the PX4 local frame, metres.

    Args:
        north_m: Along +x of the local frame.
        east_m: Along +y of the local frame.
        down_m: Along +z of the local frame (positive downwards).
    """

    north_m: float
    east_m: float
    down_m: float


@dataclass(frozen=True, slots=True)
class LatLon:
    """Geodetic position in degrees.

    Args:
        lat_deg: Latitude, degrees north.
        lon_deg: Longitude, degrees east.
    """

    lat_deg: float
    lon_deg: float


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to (-pi, pi].

    Args:
        angle_rad: Angle in radians.

    Returns:
        Equivalent angle in (-pi, pi].
    """
    wrapped = math.remainder(angle_rad, math.tau)
    # math.remainder maps exactly -pi to -pi; normalise to +pi for a half-open
    # interval so that equal angles always compare equal.
    return math.pi if wrapped == -math.pi else wrapped


def enu_to_ned(position: Enu) -> Ned:
    """Convert an ENU position to NED.

    Args:
        position: Position in the gz world frame.

    Returns:
        The same point in the PX4 local frame.
    """
    return Ned(north_m=position.north_m, east_m=position.east_m, down_m=-position.up_m)


def ned_to_enu(position: Ned) -> Enu:
    """Convert a NED position to ENU.

    Args:
        position: Position in the PX4 local frame.

    Returns:
        The same point in the gz world frame.
    """
    return Enu(east_m=position.east_m, north_m=position.north_m, up_m=-position.down_m)


def yaw_enu_to_ned(yaw_enu_rad: float) -> float:
    """Convert a gz heading to a PX4 heading.

    gz measures yaw from +x_east counter-clockwise; PX4 measures it from north
    clockwise. A model spawned with gz yaw 0 (facing east) therefore reports a
    PX4 yaw of +pi/2.

    Args:
        yaw_enu_rad: Heading in the gz world frame, radians.

    Returns:
        Heading in the PX4 local frame, radians, wrapped to (-pi, pi].
    """
    return wrap_pi(math.pi / 2.0 - yaw_enu_rad)


def yaw_ned_to_enu(yaw_ned_rad: float) -> float:
    """Convert a PX4 heading to a gz heading.

    Args:
        yaw_ned_rad: Heading in the PX4 local frame, radians.

    Returns:
        Heading in the gz world frame, radians, wrapped to (-pi, pi].
    """
    return wrap_pi(math.pi / 2.0 - yaw_ned_rad)


def frd_to_gz_pose(
    xyz_frd_m: tuple[float, float, float],
    rpy_frd_rad: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Convert a body-FRD mount pose to the gz model frame.

    Gazebo model frames are FLU (x forward, y left, z up); this project states
    every mount in aerospace body FRD (x forward, y right, z down, positive
    pitch nose-up). The two frames differ by a 180 deg rotation about x, so
    positions mirror in y and z and the pitch and yaw signs flip. A down-facing
    sensor is therefore pitch -90 deg in FRD and pitch +90 deg in gz, which is
    what PX4's own `x500_mono_cam_down` uses.

    Args:
        xyz_frd_m: Mount offset (forward, right, down) in metres.
        rpy_frd_rad: Mount rotation (roll, pitch, yaw) in radians, FRD signs.

    Returns:
        ((x, y, z), (roll, pitch, yaw)) for an SDF `<pose>` in the model frame.
    """
    forward_m, right_m, down_m = xyz_frd_m
    roll_rad, pitch_rad, yaw_rad = rpy_frd_rad
    return ((forward_m, -right_m, -down_m), (roll_rad, -pitch_rad, -yaw_rad))


def gps_from_enu(origin: LatLon, position: Enu) -> LatLon:
    """Offset a geodetic origin by a local ENU displacement.

    Uses the same spherical, locally-flat approximation as PX4; good to well
    under a metre over the few hundred metres this project covers.

    Args:
        origin: Geodetic position of the local frame origin.
        position: Displacement from that origin, metres (up is ignored).

    Returns:
        Geodetic position of the displaced point.
    """
    lat_deg = origin.lat_deg + math.degrees(position.north_m / EARTH_RADIUS_M)
    cos_lat = math.cos(math.radians(origin.lat_deg))
    lon_deg = origin.lon_deg + math.degrees(position.east_m / (EARTH_RADIUS_M * cos_lat))
    return LatLon(lat_deg=lat_deg, lon_deg=lon_deg)


def enu_from_gps(origin: LatLon, point: LatLon, up_m: float = 0.0) -> Enu:
    """Express a geodetic point as a local ENU displacement from an origin.

    Args:
        origin: Geodetic position of the local frame origin.
        point: Geodetic position to convert.
        up_m: Value to carry into the `up_m` field.

    Returns:
        Displacement of `point` from `origin`, metres.
    """
    north_m = math.radians(point.lat_deg - origin.lat_deg) * EARTH_RADIUS_M
    cos_lat = math.cos(math.radians(origin.lat_deg))
    east_m = math.radians(point.lon_deg - origin.lon_deg) * EARTH_RADIUS_M * cos_lat
    return Enu(east_m=east_m, north_m=north_m, up_m=up_m)


def haversine_m(first: LatLon, second: LatLon) -> float:
    """Great-circle distance between two geodetic points.

    Args:
        first: First position.
        second: Second position.

    Returns:
        Distance in metres.
    """
    lat1 = math.radians(first.lat_deg)
    lat2 = math.radians(second.lat_deg)
    delta_lat = lat2 - lat1
    delta_lon = math.radians(second.lon_deg - first.lon_deg)
    inner = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(inner))


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    """Whether a 2-D point lies inside a polygon (ray casting, even-odd rule).

    The polygon is treated as closed; vertex order does not matter. Points
    exactly on an edge may fall either way, so callers that care about the
    boundary must add an explicit buffer.

    Args:
        point: (x, y) in the same frame and units as the polygon.
        polygon: At least three (x, y) vertices.

    Returns:
        True if the point is inside.

    Raises:
        ValueError: The polygon has fewer than three vertices.
    """
    if len(polygon) < 3:
        raise ValueError(f"polygon needs >= 3 vertices, got {len(polygon)}")
    x, y = point
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        straddles = (current_y > y) != (previous_y > y)
        if straddles:
            crossing_x = current_x + (y - current_y) / (previous_y - current_y) * (
                previous_x - current_x
            )
            if x < crossing_x:
                inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def rect_corners(
    center_x: float, center_y: float, size_x: float, size_y: float
) -> list[tuple[float, float]]:
    """Corners of an axis-aligned rectangle, counter-clockwise from the minimum.

    Args:
        center_x: Centre along x.
        center_y: Centre along y.
        size_x: Full extent along x.
        size_y: Full extent along y.

    Returns:
        Four (x, y) corners.
    """
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    return [
        (center_x - half_x, center_y - half_y),
        (center_x + half_x, center_y - half_y),
        (center_x + half_x, center_y + half_y),
        (center_x - half_x, center_y + half_y),
    ]

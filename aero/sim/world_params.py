"""Typed, validated loader for `sim/world_params.yaml`.

Every geometric and sensor tunable of the simulated field lives in that file;
nothing in the generator may invent a number. Parameters are loaded once into
frozen dataclasses and passed explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from aero.core.errors import ConfigError
from aero.core.geo import LatLon

DEFAULT_PARAMS_PATH = Path(__file__).resolve().parents[2] / "sim" / "world_params.yaml"

Rgb = tuple[float, float, float]
Vec3 = tuple[float, float, float]
Range = tuple[float, float]


class _Reader:
    """Reads one YAML mapping, reporting the full key path on any problem."""

    def __init__(self, data: Any, path: str) -> None:
        """Wrap `data`, which must be a mapping found at `path`."""
        if not isinstance(data, dict):
            raise ConfigError(f"{path}: expected a mapping, got {type(data).__name__}")
        self._data: dict[str, Any] = data
        self._path = path

    def section(self, key: str) -> _Reader:
        """Return a reader for a nested mapping."""
        return _Reader(self._raw(key), f"{self._path}.{key}")

    def number(self, key: str, minimum: float | None = None) -> float:
        """Read a float, optionally enforcing a lower bound."""
        value = self._raw(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{self._path}.{key}: expected a number, got {value!r}")
        number = float(value)
        if minimum is not None and number < minimum:
            raise ConfigError(f"{self._path}.{key}: {number} is below the minimum {minimum}")
        return number

    def integer(self, key: str, minimum: int | None = None) -> int:
        """Read an int, optionally enforcing a lower bound."""
        value = self._raw(key)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{self._path}.{key}: expected an integer, got {value!r}")
        if minimum is not None and value < minimum:
            raise ConfigError(f"{self._path}.{key}: {value} is below the minimum {minimum}")
        return value

    def text(self, key: str) -> str:
        """Read a non-empty string."""
        value = self._raw(key)
        if not isinstance(value, str) or not value:
            raise ConfigError(f"{self._path}.{key}: expected a non-empty string, got {value!r}")
        return value

    def floats(self, key: str, count: int) -> tuple[float, ...]:
        """Read a fixed-length sequence of numbers."""
        value = self._raw(key)
        if not isinstance(value, list) or len(value) != count:
            raise ConfigError(f"{self._path}.{key}: expected {count} numbers, got {value!r}")
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ConfigError(f"{self._path}.{key}: {item!r} is not a number")
        return tuple(float(item) for item in value)

    def rgb(self, key: str) -> Rgb:
        """Read an RGB triple, each channel in [0, 1]."""
        red, green, blue = self.floats(key, 3)
        for channel in (red, green, blue):
            if not 0.0 <= channel <= 1.0:
                raise ConfigError(f"{self._path}.{key}: {channel} is outside [0, 1]")
        return (red, green, blue)

    def vec3(self, key: str) -> Vec3:
        """Read an (x, y, z) triple."""
        x, y, z = self.floats(key, 3)
        return (x, y, z)

    def span(self, key: str, minimum: float | None = None) -> Range:
        """Read an inclusive [low, high] range with low <= high."""
        low, high = self.floats(key, 2)
        if low > high:
            raise ConfigError(f"{self._path}.{key}: low {low} exceeds high {high}")
        if minimum is not None and low < minimum:
            raise ConfigError(f"{self._path}.{key}: low {low} is below the minimum {minimum}")
        return (low, high)

    def int_span(self, key: str, minimum: int | None = None) -> tuple[int, int]:
        """Read an inclusive [low, high] integer range."""
        value = self._raw(key)
        if not isinstance(value, list) or len(value) != 2:
            raise ConfigError(f"{self._path}.{key}: expected 2 integers, got {value!r}")
        low, high = value
        if isinstance(low, bool) or isinstance(high, bool):
            raise ConfigError(f"{self._path}.{key}: expected integers, got {value!r}")
        if not isinstance(low, int) or not isinstance(high, int):
            raise ConfigError(f"{self._path}.{key}: expected integers, got {value!r}")
        if low > high:
            raise ConfigError(f"{self._path}.{key}: low {low} exceeds high {high}")
        if minimum is not None and low < minimum:
            raise ConfigError(f"{self._path}.{key}: low {low} is below the minimum {minimum}")
        return (low, high)

    def _raw(self, key: str) -> Any:
        """Fetch a key or raise with its full path."""
        if key not in self._data:
            raise ConfigError(f"{self._path}.{key}: missing")
        return self._data[key]


@dataclass(frozen=True, slots=True)
class GroundParams:
    """Ground plane and its generated grass texture."""

    size_m: float
    texture_px: int
    base_rgb: Rgb
    noise_strength: float


@dataclass(frozen=True, slots=True)
class TakeoffZoneParams:
    """Blue takeoff zone centred on home."""

    size_x_m: float
    size_y_m: float
    rgb: Rgb


@dataclass(frozen=True, slots=True)
class StartQrParams:
    """The 1 m start QR on the ground ahead of home."""

    size_m: float
    offset_x_m: float
    texture_px: int
    quiet_zone_modules: int


@dataclass(frozen=True, slots=True)
class LaneParams:
    """The two walled lanes (forward and return)."""

    width_m: float
    length_m: float
    wall_height_m: float
    wall_thickness_m: float
    start_x_m: float
    start_x_jitter_m: float
    wall_rgb: Rgb


@dataclass(frozen=True, slots=True)
class BannerParams:
    """Green banner above each lane entrance."""

    width_m: float
    height_m: float
    bottom_z_m: float
    text: str
    texture_px: int
    hue_deg_range: Range
    saturation_range: Range
    value_range: Range


@dataclass(frozen=True, slots=True)
class ObstacleParams:
    """Black blocks placed in the return lane only."""

    count_range: tuple[int, int]
    width_range_m: Range
    depth_m: float
    min_spacing_x_m: float
    end_clearance_m: float
    min_free_gap_m: float
    rgb: Rgb


@dataclass(frozen=True, slots=True)
class DeliveryZoneParams:
    """The 40 x 30 m delivery zone and its QR targets."""

    size_x_m: float
    size_y_m: float
    gap_beyond_lane_m: float
    texture_px: int
    base_rgb: Rgb
    noise_strength: float
    qr_count_range: tuple[int, int]
    qr_size_m: float
    qr_min_spacing_m: float
    qr_margin_m: float
    qr_payload_prefix: str


@dataclass(frozen=True, slots=True)
class RedZoneParams:
    """Red no-fly rectangles inside the delivery zone."""

    count_range: tuple[int, int]
    size_x_m: float
    size_y_m: float
    min_clearance_to_qr_m: float
    rgb: Rgb


@dataclass(frozen=True, slots=True)
class VehicleParams:
    """Spawn pose and size of the simulated vehicle."""

    model_name: str
    spawn_z_m: float
    spawn_yaw_deg: float
    radius_m: float


@dataclass(frozen=True, slots=True)
class CameraParams:
    """One simulated camera.

    `mount_rpy_deg` is applied in the gz model frame; a pitch of +90 deg points
    the lens at the ground.
    """

    width_px: int
    height_px: int
    hfov_deg: float
    update_rate_hz: float
    mount_xyz_m: Vec3
    mount_rpy_deg: Vec3
    noise_stddev: float

    @property
    def aspect_ratio(self) -> float:
        """Image width divided by height."""
        return self.width_px / self.height_px


@dataclass(frozen=True, slots=True)
class RangeParams:
    """One simulated single-beam rangefinder (a narrow gz lidar)."""

    min_m: float
    max_m: float
    beam_deg: float
    rays: int
    update_rate_hz: float
    noise_stddev_m: float
    mount_xyz_m: Vec3
    mount_rpy_deg: Vec3


@dataclass(frozen=True, slots=True)
class WorldParams:
    """All parameters of the generated field."""

    origin: LatLon
    elevation_m: float
    ground: GroundParams
    takeoff_zone: TakeoffZoneParams
    start_qr: StartQrParams
    lanes: LaneParams
    banner: BannerParams
    obstacles: ObstacleParams
    delivery_zone: DeliveryZoneParams
    red_zones: RedZoneParams
    geofence_margin_m: float
    vehicle: VehicleParams
    camera_down: CameraParams
    camera_front: CameraParams
    range_front: RangeParams
    range_down: RangeParams


def _camera(reader: _Reader) -> CameraParams:
    """Build camera parameters from a reader."""
    return CameraParams(
        width_px=reader.integer("width_px", minimum=1),
        height_px=reader.integer("height_px", minimum=1),
        hfov_deg=reader.number("hfov_deg", minimum=1.0),
        update_rate_hz=reader.number("update_rate_hz", minimum=1.0),
        mount_xyz_m=reader.vec3("mount_xyz_m"),
        mount_rpy_deg=reader.vec3("mount_rpy_deg"),
        noise_stddev=reader.number("noise_stddev", minimum=0.0),
    )


def _range(reader: _Reader) -> RangeParams:
    """Build rangefinder parameters from a reader."""
    minimum_m = reader.number("min_m", minimum=0.0)
    maximum_m = reader.number("max_m", minimum=0.0)
    if maximum_m <= minimum_m:
        raise ConfigError(f"range max_m {maximum_m} must exceed min_m {minimum_m}")
    return RangeParams(
        min_m=minimum_m,
        max_m=maximum_m,
        beam_deg=reader.number("beam_deg", minimum=0.0),
        rays=reader.integer("rays", minimum=1),
        update_rate_hz=reader.number("update_rate_hz", minimum=1.0),
        noise_stddev_m=reader.number("noise_stddev_m", minimum=0.0),
        mount_xyz_m=reader.vec3("mount_xyz_m"),
        mount_rpy_deg=reader.vec3("mount_rpy_deg"),
    )


def parse_world_params(data: Any) -> WorldParams:
    """Validate a parsed YAML mapping into `WorldParams`.

    Args:
        data: Result of `yaml.safe_load` on the params file.

    Returns:
        Validated parameters.

    Raises:
        ConfigError: Any key is missing, mistyped, or out of range.
    """
    root = _Reader(data, "world_params")

    origin = root.section("origin")
    ground = root.section("ground")
    takeoff = root.section("takeoff_zone")
    start_qr = root.section("start_qr")
    lanes = root.section("lanes")
    banner = root.section("banner")
    obstacles = root.section("obstacles")
    zone = root.section("delivery_zone")
    red = root.section("red_zones")
    vehicle = root.section("vehicle")
    cameras = root.section("cameras")
    ranges = root.section("ranges")

    params = WorldParams(
        origin=LatLon(lat_deg=origin.number("lat_deg"), lon_deg=origin.number("lon_deg")),
        elevation_m=origin.number("elevation_m"),
        ground=GroundParams(
            size_m=ground.number("size_m", minimum=1.0),
            texture_px=ground.integer("texture_px", minimum=16),
            base_rgb=ground.rgb("base_rgb"),
            noise_strength=ground.number("noise_strength", minimum=0.0),
        ),
        takeoff_zone=TakeoffZoneParams(
            size_x_m=takeoff.number("size_x_m", minimum=0.1),
            size_y_m=takeoff.number("size_y_m", minimum=0.1),
            rgb=takeoff.rgb("rgb"),
        ),
        start_qr=StartQrParams(
            size_m=start_qr.number("size_m", minimum=0.1),
            offset_x_m=start_qr.number("offset_x_m"),
            texture_px=start_qr.integer("texture_px", minimum=64),
            quiet_zone_modules=start_qr.integer("quiet_zone_modules", minimum=1),
        ),
        lanes=LaneParams(
            width_m=lanes.number("width_m", minimum=0.5),
            length_m=lanes.number("length_m", minimum=1.0),
            wall_height_m=lanes.number("wall_height_m", minimum=0.1),
            wall_thickness_m=lanes.number("wall_thickness_m", minimum=0.01),
            start_x_m=lanes.number("start_x_m"),
            start_x_jitter_m=lanes.number("start_x_jitter_m", minimum=0.0),
            wall_rgb=lanes.rgb("wall_rgb"),
        ),
        banner=BannerParams(
            width_m=banner.number("width_m", minimum=0.1),
            height_m=banner.number("height_m", minimum=0.1),
            bottom_z_m=banner.number("bottom_z_m", minimum=0.0),
            text=banner.text("text"),
            texture_px=banner.integer("texture_px", minimum=64),
            hue_deg_range=banner.span("hue_deg_range", minimum=0.0),
            saturation_range=banner.span("saturation_range", minimum=0.0),
            value_range=banner.span("value_range", minimum=0.0),
        ),
        obstacles=ObstacleParams(
            count_range=obstacles.int_span("count_range", minimum=0),
            width_range_m=obstacles.span("width_range_m", minimum=0.05),
            depth_m=obstacles.number("depth_m", minimum=0.05),
            min_spacing_x_m=obstacles.number("min_spacing_x_m", minimum=0.0),
            end_clearance_m=obstacles.number("end_clearance_m", minimum=0.0),
            min_free_gap_m=obstacles.number("min_free_gap_m", minimum=0.0),
            rgb=obstacles.rgb("rgb"),
        ),
        delivery_zone=DeliveryZoneParams(
            size_x_m=zone.number("size_x_m", minimum=1.0),
            size_y_m=zone.number("size_y_m", minimum=1.0),
            gap_beyond_lane_m=zone.number("gap_beyond_lane_m", minimum=0.0),
            texture_px=zone.integer("texture_px", minimum=16),
            base_rgb=zone.rgb("base_rgb"),
            noise_strength=zone.number("noise_strength", minimum=0.0),
            qr_count_range=zone.int_span("qr_count_range", minimum=1),
            qr_size_m=zone.number("qr_size_m", minimum=0.1),
            qr_min_spacing_m=zone.number("qr_min_spacing_m", minimum=0.0),
            qr_margin_m=zone.number("qr_margin_m", minimum=0.0),
            qr_payload_prefix=zone.text("qr_payload_prefix"),
        ),
        red_zones=RedZoneParams(
            count_range=red.int_span("count_range", minimum=0),
            size_x_m=red.number("size_x_m", minimum=0.1),
            size_y_m=red.number("size_y_m", minimum=0.1),
            min_clearance_to_qr_m=red.number("min_clearance_to_qr_m", minimum=0.0),
            rgb=red.rgb("rgb"),
        ),
        geofence_margin_m=root.section("geofence").number("margin_m", minimum=0.0),
        vehicle=VehicleParams(
            model_name=vehicle.text("model_name"),
            spawn_z_m=vehicle.number("spawn_z_m", minimum=0.0),
            spawn_yaw_deg=vehicle.number("spawn_yaw_deg"),
            radius_m=vehicle.number("radius_m", minimum=0.01),
        ),
        camera_down=_camera(cameras.section("down")),
        camera_front=_camera(cameras.section("front")),
        range_front=_range(ranges.section("front")),
        range_down=_range(ranges.section("down")),
    )
    _check_consistency(params)
    return params


def _check_consistency(params: WorldParams) -> None:
    """Cross-field checks that a single value cannot catch.

    Raises:
        ConfigError: The combination of parameters cannot produce a valid field.
    """
    lanes = params.lanes
    obstacles = params.obstacles
    widest_obstacle_m = obstacles.width_range_m[1]
    if lanes.width_m - widest_obstacle_m < obstacles.min_free_gap_m:
        raise ConfigError(
            f"lane width {lanes.width_m} m minus widest obstacle {widest_obstacle_m} m "
            f"leaves less than the required free gap {obstacles.min_free_gap_m} m"
        )
    max_count = obstacles.count_range[1]
    needed_m = (
        2 * obstacles.end_clearance_m
        + obstacles.depth_m
        + max(0, max_count - 1) * obstacles.min_spacing_x_m
    )
    if needed_m > lanes.length_m:
        raise ConfigError(
            f"{max_count} obstacles need {needed_m:.1f} m of lane but the lane is "
            f"{lanes.length_m} m long"
        )
    if params.banner.bottom_z_m < lanes.wall_height_m:
        raise ConfigError(
            f"banner bottom {params.banner.bottom_z_m} m would block the lane "
            f"(wall height {lanes.wall_height_m} m)"
        )
    zone = params.delivery_zone
    field_extent_m = (
        lanes.start_x_m + lanes.start_x_jitter_m + lanes.length_m + zone.gap_beyond_lane_m
    ) + zone.size_x_m
    half_ground_m = params.ground.size_m / 2.0
    if field_extent_m + params.geofence_margin_m > half_ground_m:
        raise ConfigError(
            f"field reaches {field_extent_m:.1f} m but the ground plane only extends "
            f"{half_ground_m:.1f} m from the origin"
        )


def load_world_params(path: Path | None = None) -> WorldParams:
    """Load and validate the world parameters.

    Args:
        path: Params file; defaults to `sim/world_params.yaml`.

    Returns:
        Validated parameters.

    Raises:
        ConfigError: The file is missing, unparseable, or invalid.
    """
    params_path = DEFAULT_PARAMS_PATH if path is None else path
    try:
        text = params_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ConfigError(f"cannot read world params at {params_path}: {error}") from error
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ConfigError(f"{params_path} is not valid YAML: {error}") from error
    return parse_world_params(data)

"""Seeded field layout: parameters in, geometry out. Pure, no I/O.

All coordinates are gz world ENU metres with the origin at the takeoff point
(home): +x is the lane direction ("forward", which is east), +y is left of it,
+z is up. The same layout object drives the SDF generator, the field GPS file
and the judge's ground truth, so there is exactly one source of geometry.

Determinism: every random choice comes from `random.Random(seed)`, so a seed
always yields the identical layout.
"""

from __future__ import annotations

import colorsys
import string
from dataclasses import dataclass
from random import Random

from aero.core.errors import ConfigError
from aero.sim.world_params import Rgb, Vec3, WorldParams

# Rejection sampling bound: with the default parameters a placement succeeds in
# a handful of tries, so hitting this means the parameters are inconsistent.
_MAX_PLACEMENT_ATTEMPTS = 2000


@dataclass(frozen=True, slots=True)
class Box:
    """Axis-aligned static box.

    Args:
        name: Unique model/link name in the world.
        center_m: Centre (x, y, z) in ENU metres; z is the centre height.
        size_m: Full extents (x, y, z) in metres.
        rgb: Flat colour.
    """

    name: str
    center_m: Vec3
    size_m: Vec3
    rgb: Rgb

    @property
    def footprint(self) -> tuple[float, float, float, float]:
        """(min_x, min_y, max_x, max_y) of the box footprint, metres."""
        x, y, _ = self.center_m
        size_x, size_y, _ = self.size_m
        return (x - size_x / 2, y - size_y / 2, x + size_x / 2, y + size_y / 2)


@dataclass(frozen=True, slots=True)
class GroundPatch:
    """Flat textured or coloured patch lying on the ground.

    Args:
        name: Unique model name.
        center_xy_m: Centre (x, y) in ENU metres.
        size_xy_m: Extents (x, y) in metres.
        texture: Texture file relative to the world directory, or None.
        rgb: Flat colour, used when `texture` is None.
        z_m: Height of the patch above the ground plane, metres.
    """

    name: str
    center_xy_m: tuple[float, float]
    size_xy_m: tuple[float, float]
    texture: str | None
    rgb: Rgb
    z_m: float


@dataclass(frozen=True, slots=True)
class QrMarker:
    """A QR code lying flat on the ground.

    Args:
        name: Unique model name.
        payload: Decoded string the QR encodes.
        center_xy_m: Centre (x, y) in ENU metres.
        size_m: Side length in metres (also the physical size vision assumes).
        texture: Texture file relative to the world directory.
        is_target: True for the delivery QR that matches the start QR.
    """

    name: str
    payload: str
    center_xy_m: tuple[float, float]
    size_m: float
    texture: str
    is_target: bool


@dataclass(frozen=True, slots=True)
class Banner:
    """Vertical green banner board above a lane entrance.

    Args:
        name: Unique model name.
        center_m: Centre (x, y, z) in ENU metres.
        size_m: (width, height) of the board in metres.
        yaw_deg: Board normal direction in the gz world: 0 faces +x, 180 faces -x.
        texture: Texture file relative to the world directory.
    """

    name: str
    center_m: Vec3
    size_m: tuple[float, float]
    yaw_deg: float
    texture: str


@dataclass(frozen=True, slots=True)
class Rect:
    """Axis-aligned rectangle on the ground.

    Args:
        name: Unique name.
        center_xy_m: Centre (x, y) in ENU metres.
        size_xy_m: Extents (x, y) in metres.
    """

    name: str
    center_xy_m: tuple[float, float]
    size_xy_m: tuple[float, float]

    @property
    def corners(self) -> tuple[tuple[float, float], ...]:
        """Corners counter-clockwise from the (min_x, min_y) corner."""
        x, y = self.center_xy_m
        size_x, size_y = self.size_xy_m
        half_x, half_y = size_x / 2, size_y / 2
        return (
            (x - half_x, y - half_y),
            (x + half_x, y - half_y),
            (x + half_x, y + half_y),
            (x - half_x, y + half_y),
        )


@dataclass(frozen=True, slots=True)
class Lane:
    """One walled lane.

    Args:
        name: "forward" or "return".
        center_y_m: Lane centreline y in ENU metres.
        start_x_m: Entrance x (the home side).
        end_x_m: Exit x (the delivery-zone side).
        width_m: Clear width between walls.
    """

    name: str
    center_y_m: float
    start_x_m: float
    end_x_m: float
    width_m: float

    @property
    def entry_xy_m(self) -> tuple[float, float]:
        """Centre of the lane entrance (home side)."""
        return (self.start_x_m, self.center_y_m)

    @property
    def exit_xy_m(self) -> tuple[float, float]:
        """Centre of the lane exit (delivery-zone side)."""
        return (self.end_x_m, self.center_y_m)


@dataclass(frozen=True, slots=True)
class FieldLayout:
    """Everything the generator needs to emit, for one seed."""

    seed: int
    forward_lane: Lane
    return_lane: Lane
    walls: tuple[Box, ...]
    obstacles: tuple[Box, ...]
    banners: tuple[Banner, ...]
    banner_rgb: Rgb
    takeoff_zone: GroundPatch
    start_qr: QrMarker
    delivery_zone: GroundPatch
    delivery_zone_rect: Rect
    delivery_qrs: tuple[QrMarker, ...]
    red_zones: tuple[Rect, ...]
    geofence_xy_m: tuple[tuple[float, float], ...]

    @property
    def target_payload(self) -> str:
        """Payload of the start QR, i.e. the delivery target to find."""
        return self.start_qr.payload

    @property
    def target_qr(self) -> QrMarker:
        """The delivery QR whose payload matches the start QR."""
        for marker in self.delivery_qrs:
            if marker.is_target:
                return marker
        raise ConfigError("layout has no target delivery QR")

    @property
    def all_qrs(self) -> tuple[QrMarker, ...]:
        """Start QR plus every delivery QR."""
        return (self.start_qr, *self.delivery_qrs)


def _payload_names(prefix: str, count: int) -> list[str]:
    """Generate `count` payload strings `<prefix>A`, `<prefix>B`, ...

    Raises:
        ConfigError: More payloads requested than single letters available.
    """
    if count > len(string.ascii_uppercase):
        raise ConfigError(f"cannot name {count} QR payloads with single letters")
    return [f"{prefix}{letter}" for letter in string.ascii_uppercase[:count]]


def _lane_walls(params: WorldParams, forward: Lane, ret: Lane) -> tuple[Box, ...]:
    """Build the three walls shared by the two side-by-side lanes.

    The wall between the lanes is shared, so two 3.5 m lanes need three walls.
    """
    lanes = params.lanes
    thickness = lanes.wall_thickness_m
    height = lanes.wall_height_m
    length = forward.end_x_m - forward.start_x_m
    center_x = (forward.start_x_m + forward.end_x_m) / 2.0
    half_width = lanes.width_m / 2.0
    wall_ys = {
        "outer_left": forward.center_y_m + half_width + thickness / 2.0,
        "shared": forward.center_y_m - half_width - thickness / 2.0,
        "outer_right": ret.center_y_m - half_width - thickness / 2.0,
    }
    return tuple(
        Box(
            name=f"wall_{name}",
            center_m=(center_x, y, height / 2.0),
            size_m=(length, thickness, height),
            rgb=lanes.wall_rgb,
        )
        for name, y in wall_ys.items()
    )


def _obstacles(params: WorldParams, lane: Lane, rng: Random) -> tuple[Box, ...]:
    """Place black blocks in the return lane, alternating sides.

    Each block spans the full wall height and is attached to one side wall, so
    the free gap is on the other side; `min_free_gap_m` bounds the width.
    """
    settings = params.obstacles
    count = rng.randint(*settings.count_range)
    if count == 0:
        return ()
    height = params.lanes.wall_height_m
    half_width = lane.width_m / 2.0
    # Keep blocks clear of both lane mouths so entry and exit stay open.
    usable_start = lane.start_x_m + settings.end_clearance_m
    usable_end = lane.end_x_m - settings.end_clearance_m
    span = usable_end - usable_start
    needed = settings.depth_m + (count - 1) * settings.min_spacing_x_m
    slack = span - needed
    if slack < 0:
        raise ConfigError(
            f"cannot fit {count} obstacles {settings.min_spacing_x_m} m apart in {span:.1f} m "
            f"of usable return lane (needs {needed:.1f} m)"
        )
    # Spend the slack as random gaps: one before the first block and one on top
    # of each minimum spacing. Consecutive centres are then always at least
    # `min_spacing_x_m` apart, and the last block still fits inside the lane.
    weights = [rng.random() for _ in range(count)]
    total_weight = sum(weights) or 1.0
    extra_gaps = [weight / total_weight * slack for weight in weights]

    boxes: list[Box] = []
    side = 1.0 if rng.random() < 0.5 else -1.0
    center_x = usable_start + settings.depth_m / 2.0
    for index in range(count):
        width = rng.uniform(*settings.width_range_m)
        center_x += extra_gaps[index] + (settings.min_spacing_x_m if index > 0 else 0.0)
        # Flush against the wall on `side`, so the free gap is on the far side.
        center_y = lane.center_y_m + side * (half_width - width / 2.0)
        boxes.append(
            Box(
                name=f"obstacle_{index}",
                center_m=(center_x, center_y, height / 2.0),
                size_m=(settings.depth_m, width, height),
                rgb=settings.rgb,
            )
        )
        side = -side
    return tuple(boxes)


def _banner_rgb(params: WorldParams, rng: Random) -> Rgb:
    """Pick the banner green for this seed, jittered inside the HSV ranges."""
    hue_deg = rng.uniform(*params.banner.hue_deg_range)
    saturation = rng.uniform(*params.banner.saturation_range)
    value = rng.uniform(*params.banner.value_range)
    red, green, blue = colorsys.hsv_to_rgb(hue_deg / 360.0, saturation, value)
    return (red, green, blue)


def _banners(params: WorldParams, forward: Lane, ret: Lane) -> tuple[Banner, ...]:
    """Banner above each lane entrance, facing the approaching drone.

    The forward banner is seen when flying out from home (so it faces -x); the
    return banner is seen when coming back from the delivery zone (faces +x).
    """
    settings = params.banner
    center_z = settings.bottom_z_m + settings.height_m / 2.0
    return (
        Banner(
            name="banner_forward",
            center_m=(forward.start_x_m, forward.center_y_m, center_z),
            size_m=(settings.width_m, settings.height_m),
            yaw_deg=180.0,
            texture="textures/banner.png",
        ),
        Banner(
            name="banner_return",
            center_m=(ret.end_x_m, ret.center_y_m, center_z),
            size_m=(settings.width_m, settings.height_m),
            yaw_deg=0.0,
            texture="textures/banner.png",
        ),
    )


def _place_delivery_qrs(
    params: WorldParams, zone: Rect, rng: Random
) -> tuple[tuple[QrMarker, ...], str]:
    """Scatter the delivery QRs inside the zone and pick the target.

    Returns:
        (markers, target payload).

    Raises:
        ConfigError: No valid placement was found within the attempt budget.
    """
    settings = params.delivery_zone
    count = rng.randint(*settings.qr_count_range)
    payloads = _payload_names(settings.qr_payload_prefix, count)
    center_x, center_y = zone.center_xy_m
    size_x, size_y = zone.size_xy_m
    margin = settings.qr_margin_m + settings.qr_size_m / 2.0
    low_x, high_x = center_x - size_x / 2 + margin, center_x + size_x / 2 - margin
    low_y, high_y = center_y - size_y / 2 + margin, center_y + size_y / 2 - margin
    if low_x >= high_x or low_y >= high_y:
        raise ConfigError("delivery zone is too small for the configured QR margin")

    placed: list[tuple[float, float]] = []
    attempts = 0
    while len(placed) < count:
        attempts += 1
        if attempts > _MAX_PLACEMENT_ATTEMPTS:
            raise ConfigError(
                f"could not place {count} QRs {settings.qr_min_spacing_m} m apart inside the "
                f"{size_x} x {size_y} m delivery zone"
            )
        candidate = (rng.uniform(low_x, high_x), rng.uniform(low_y, high_y))
        if all(_distance(candidate, other) >= settings.qr_min_spacing_m for other in placed):
            placed.append(candidate)

    target_index = rng.randrange(count)
    markers = tuple(
        QrMarker(
            name=f"delivery_qr_{index}",
            payload=payload,
            center_xy_m=position,
            size_m=settings.qr_size_m,
            texture=f"textures/qr_{payload}.png",
            is_target=index == target_index,
        )
        for index, (payload, position) in enumerate(zip(payloads, placed, strict=True))
    )
    return markers, payloads[target_index]


def _place_red_zones(
    params: WorldParams, zone: Rect, qrs: tuple[QrMarker, ...], rng: Random
) -> tuple[Rect, ...]:
    """Place red no-fly rectangles, clear of the QRs and of each other.

    Raises:
        ConfigError: No valid placement was found within the attempt budget.
    """
    settings = params.red_zones
    count = rng.randint(*settings.count_range)
    if count == 0:
        return ()
    center_x, center_y = zone.center_xy_m
    size_x, size_y = zone.size_xy_m
    margin_x = settings.size_x_m / 2.0 + params.delivery_zone.qr_margin_m
    margin_y = settings.size_y_m / 2.0 + params.delivery_zone.qr_margin_m
    low_x, high_x = center_x - size_x / 2 + margin_x, center_x + size_x / 2 - margin_x
    low_y, high_y = center_y - size_y / 2 + margin_y, center_y + size_y / 2 - margin_y
    if low_x >= high_x or low_y >= high_y:
        raise ConfigError("delivery zone is too small for the configured red zones")

    # A red zone must stay `min_clearance_to_qr_m` from any QR edge, and the
    # same distance from another red zone, so neither overlaps nor crowds.
    qr_clearance = settings.min_clearance_to_qr_m + params.delivery_zone.qr_size_m / 2.0
    zone_clearance = settings.min_clearance_to_qr_m + max(settings.size_x_m, settings.size_y_m)
    placed: list[tuple[float, float]] = []
    attempts = 0
    while len(placed) < count:
        attempts += 1
        if attempts > _MAX_PLACEMENT_ATTEMPTS:
            raise ConfigError(
                f"could not place {count} red zones clear of {len(qrs)} QRs in the delivery zone"
            )
        candidate = (rng.uniform(low_x, high_x), rng.uniform(low_y, high_y))
        half_x, half_y = settings.size_x_m / 2.0, settings.size_y_m / 2.0
        too_close_to_qr = any(
            abs(candidate[0] - marker.center_xy_m[0]) < half_x + qr_clearance
            and abs(candidate[1] - marker.center_xy_m[1]) < half_y + qr_clearance
            for marker in qrs
        )
        if too_close_to_qr:
            continue
        if any(_distance(candidate, other) < zone_clearance for other in placed):
            continue
        placed.append(candidate)

    return tuple(
        Rect(
            name=f"red_zone_{index}",
            center_xy_m=position,
            size_xy_m=(settings.size_x_m, settings.size_y_m),
        )
        for index, position in enumerate(placed)
    )


def _distance(first: tuple[float, float], second: tuple[float, float]) -> float:
    """Euclidean distance between two (x, y) points, metres."""
    return float(((first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2) ** 0.5)


def _geofence(
    params: WorldParams, takeoff: GroundPatch, zone: Rect, walls: tuple[Box, ...]
) -> tuple[tuple[float, float], ...]:
    """Bounding rectangle of all field features plus the configured margin."""
    xs: list[float] = []
    ys: list[float] = []
    for min_x, min_y, max_x, max_y in [wall.footprint for wall in walls]:
        xs.extend((min_x, max_x))
        ys.extend((min_y, max_y))
    for center, size in (
        (takeoff.center_xy_m, takeoff.size_xy_m),
        (zone.center_xy_m, zone.size_xy_m),
    ):
        xs.extend((center[0] - size[0] / 2, center[0] + size[0] / 2))
        ys.extend((center[1] - size[1] / 2, center[1] + size[1] / 2))
    margin = params.geofence_margin_m
    low_x, high_x = min(xs) - margin, max(xs) + margin
    low_y, high_y = min(ys) - margin, max(ys) + margin
    return ((low_x, low_y), (high_x, low_y), (high_x, high_y), (low_x, high_y))


def build_field(params: WorldParams, seed: int) -> FieldLayout:
    """Build the complete field layout for one seed.

    Args:
        params: Validated world parameters.
        seed: Layout seed; the same seed always yields the same layout.

    Returns:
        The layout.

    Raises:
        ConfigError: The parameters cannot produce a valid layout.
    """
    rng = Random(seed)
    lanes = params.lanes

    lane_start_x = lanes.start_x_m + rng.uniform(-lanes.start_x_jitter_m, lanes.start_x_jitter_m)
    lane_end_x = lane_start_x + lanes.length_m
    forward_lane = Lane(
        name="forward",
        center_y_m=0.0,
        start_x_m=lane_start_x,
        end_x_m=lane_end_x,
        width_m=lanes.width_m,
    )
    # The lanes share a wall, so their centrelines are one lane width plus one
    # wall thickness apart; the return lane is to the right of the outbound one.
    return_lane = Lane(
        name="return",
        center_y_m=-(lanes.width_m + lanes.wall_thickness_m),
        start_x_m=lane_start_x,
        end_x_m=lane_end_x,
        width_m=lanes.width_m,
    )

    walls = _lane_walls(params, forward_lane, return_lane)
    obstacles = _obstacles(params, return_lane, rng)

    takeoff_zone = GroundPatch(
        name="takeoff_zone",
        center_xy_m=(0.0, 0.0),
        size_xy_m=(params.takeoff_zone.size_x_m, params.takeoff_zone.size_y_m),
        texture=None,
        rgb=params.takeoff_zone.rgb,
        z_m=0.01,
    )

    zone_settings = params.delivery_zone
    zone_center_x = lane_end_x + zone_settings.gap_beyond_lane_m + zone_settings.size_x_m / 2.0
    zone_center_y = (forward_lane.center_y_m + return_lane.center_y_m) / 2.0
    delivery_zone_rect = Rect(
        name="delivery_zone",
        center_xy_m=(zone_center_x, zone_center_y),
        size_xy_m=(zone_settings.size_x_m, zone_settings.size_y_m),
    )
    delivery_zone = GroundPatch(
        name="delivery_zone",
        center_xy_m=delivery_zone_rect.center_xy_m,
        size_xy_m=delivery_zone_rect.size_xy_m,
        texture="textures/zone_ground.png",
        rgb=zone_settings.base_rgb,
        z_m=0.005,
    )

    delivery_qrs, target_payload = _place_delivery_qrs(params, delivery_zone_rect, rng)
    red_zones = _place_red_zones(params, delivery_zone_rect, delivery_qrs, rng)

    start_qr = QrMarker(
        name="start_qr",
        payload=target_payload,
        center_xy_m=(params.start_qr.offset_x_m, 0.0),
        size_m=params.start_qr.size_m,
        texture=f"textures/qr_{target_payload}.png",
        is_target=False,
    )

    return FieldLayout(
        seed=seed,
        forward_lane=forward_lane,
        return_lane=return_lane,
        walls=walls,
        obstacles=obstacles,
        banners=_banners(params, forward_lane, return_lane),
        banner_rgb=_banner_rgb(params, rng),
        takeoff_zone=takeoff_zone,
        start_qr=start_qr,
        delivery_zone=delivery_zone,
        delivery_zone_rect=delivery_zone_rect,
        delivery_qrs=delivery_qrs,
        red_zones=red_zones,
        geofence_xy_m=_geofence(params, takeoff_zone, delivery_zone_rect, walls),
    )

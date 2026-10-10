"""SDF builders for the generated world. Pure: elements in memory, no I/O.

Everything is assembled with `xml.etree.ElementTree` rather than string
templates so that malformed XML is impossible and the result can be asserted
on in unit tests.

Conventions:

* Static scenery only; every model gets `<static>true</static>`.
* Flat "paint" (zone outlines, QR codes, red zones) is a thin box rather than a
  plane, because a box can carry a PBR `albedo_map` and is easy to raise a few
  millimetres to avoid z-fighting with the ground.
* Texture paths are written relative to the world file; the gz server adds the
  world directory to `GZ_SIM_RESOURCE_PATH` so they resolve.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from xml.dom import minidom

from aero.sim.field import Banner, Box, FieldLayout, GroundPatch, QrMarker, Rect
from aero.sim.world_params import Rgb, WorldParams

SDF_VERSION = "1.9"
# Thickness of flat painted geometry; thin enough to be invisible from the air,
# thick enough that the renderer does not drop it.
PAINT_THICKNESS_M = 0.01
# Heights of coplanar paint layers, metres above the ground plane. Distinct
# values prevent z-fighting between overlapping patches.
GROUND_Z_M = 0.0
ZONE_PAINT_Z_M = 0.005
TAKEOFF_PAINT_Z_M = 0.01
RED_ZONE_Z_M = 0.015
QR_PAINT_Z_M = 0.02
# PX4's own worlds run physics at 250 Hz (4 ms); match it so controller timing
# and sim speed factors behave the same as in stock PX4 SITL.
PHYSICS_STEP_S = 0.004


def _text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    """Append `<tag>value</tag>` to `parent` and return the new element."""
    element = ET.SubElement(parent, tag)
    element.text = value
    return element


def _vector(values: tuple[float, ...]) -> str:
    """Format a vector the way SDF expects (space separated, trimmed).

    Nine significant digits: enough that a position keeps sub-micrometre
    fidelity and an angle stays within 1e-9 rad of the computed value, while
    still printing round decimals like `0.55` without a float tail.
    """
    return " ".join(f"{value:.9g}" for value in values)


def _pose(xyz: tuple[float, float, float], rpy_rad: tuple[float, float, float]) -> str:
    """Format an SDF pose from position and roll/pitch/yaw."""
    return _vector((*xyz, *rpy_rad))


def _material(
    parent: ET.Element, rgb: Rgb, texture: str | None = None, *, cast_shadows: bool = True
) -> None:
    """Attach a `<material>` using a flat colour or a PBR albedo map."""
    material = ET.SubElement(parent, "material")
    colour = _vector((*rgb, 1.0))
    # With an albedo map the base colour must stay white, otherwise it tints
    # the texture and QR contrast suffers.
    base = "1 1 1 1" if texture is not None else colour
    _text(material, "ambient", base)
    _text(material, "diffuse", base)
    _text(material, "specular", "0.1 0.1 0.1 1")
    if texture is not None:
        pbr = ET.SubElement(material, "pbr")
        metal = ET.SubElement(pbr, "metal")
        _text(metal, "albedo_map", texture)
        _text(metal, "metalness", "0")
        _text(metal, "roughness", "0.9")
    if not cast_shadows:
        _text(parent, "cast_shadows", "false")


def _box_geometry(parent: ET.Element, size_m: tuple[float, float, float]) -> ET.Element:
    """Append `<geometry><box><size>...` to `parent`."""
    geometry = ET.SubElement(parent, "geometry")
    box = ET.SubElement(geometry, "box")
    _text(box, "size", _vector(size_m))
    return geometry


def static_box_model(
    name: str,
    center_m: tuple[float, float, float],
    size_m: tuple[float, float, float],
    rgb: Rgb,
    *,
    yaw_rad: float = 0.0,
    texture: str | None = None,
    collide: bool = True,
) -> ET.Element:
    """Build one static box model.

    Args:
        name: Model name, unique in the world.
        center_m: Box centre (x, y, z) in ENU metres.
        size_m: Full extents (x, y, z) in metres.
        rgb: Flat colour, also the fallback when `texture` is set.
        yaw_rad: Rotation about z.
        texture: Relative albedo-map path, or None for a flat colour.
        collide: False for flat paint, which must not block the drone.

    Returns:
        The `<model>` element.
    """
    model = ET.Element("model", {"name": name})
    _text(model, "static", "true")
    _text(model, "pose", _pose(center_m, (0.0, 0.0, yaw_rad)))
    link = ET.SubElement(model, "link", {"name": "link"})
    if collide:
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        _box_geometry(collision, size_m)
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    _box_geometry(visual, size_m)
    _material(visual, rgb, texture, cast_shadows=collide)
    return model


def wall_model(box: Box) -> ET.Element:
    """Build a lane wall (collidable, flat grey)."""
    return static_box_model(box.name, box.center_m, box.size_m, box.rgb)


def obstacle_model(box: Box) -> ET.Element:
    """Build a return-lane obstacle (collidable, black)."""
    return static_box_model(box.name, box.center_m, box.size_m, box.rgb)


def ground_patch_model(patch: GroundPatch) -> ET.Element:
    """Build a flat painted patch (takeoff zone, delivery-zone ground)."""
    center_x, center_y = patch.center_xy_m
    size_x, size_y = patch.size_xy_m
    return static_box_model(
        patch.name,
        (center_x, center_y, patch.z_m),
        (size_x, size_y, PAINT_THICKNESS_M),
        patch.rgb,
        texture=patch.texture,
        collide=False,
    )


def qr_model(marker: QrMarker) -> ET.Element:
    """Build a QR code lying flat on the ground."""
    center_x, center_y = marker.center_xy_m
    return static_box_model(
        marker.name,
        (center_x, center_y, QR_PAINT_Z_M),
        (marker.size_m, marker.size_m, PAINT_THICKNESS_M),
        (1.0, 1.0, 1.0),
        texture=marker.texture,
        collide=False,
    )


def red_zone_model(rect: Rect, rgb: Rgb) -> ET.Element:
    """Build a red no-fly rectangle painted on the ground."""
    center_x, center_y = rect.center_xy_m
    size_x, size_y = rect.size_xy_m
    return static_box_model(
        rect.name,
        (center_x, center_y, RED_ZONE_Z_M),
        (size_x, size_y, PAINT_THICKNESS_M),
        rgb,
        collide=False,
    )


def banner_model(banner: Banner, rgb: Rgb) -> ET.Element:
    """Build a vertical banner board.

    The board is a thin box in the y-z plane, rotated by `banner.yaw_deg` so
    its printed face points at the approaching drone.
    """
    width_m, height_m = banner.size_m
    return static_box_model(
        banner.name,
        banner.center_m,
        (PAINT_THICKNESS_M, width_m, height_m),
        rgb,
        yaw_rad=math.radians(banner.yaw_deg),
        texture=banner.texture,
        collide=False,
    )


def ground_plane_model(params: WorldParams, texture: str) -> ET.Element:
    """Build the large textured ground plane with its collision plane."""
    size_m = params.ground.size_m
    model = ET.Element("model", {"name": "ground_plane"})
    _text(model, "static", "true")
    link = ET.SubElement(model, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    _text(plane, "normal", "0 0 1")
    _text(plane, "size", _vector((size_m, size_m)))
    surface = ET.SubElement(collision, "surface")
    friction = ET.SubElement(surface, "friction")
    ode = ET.SubElement(friction, "ode")
    _text(ode, "mu", "1")
    _text(ode, "mu2", "1")
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    _box_geometry(visual, (size_m, size_m, PAINT_THICKNESS_M))
    _text(visual, "pose", _pose((0.0, 0.0, GROUND_Z_M - PAINT_THICKNESS_M / 2), (0.0, 0.0, 0.0)))
    _material(visual, params.ground.base_rgb, texture)
    return model


def sun_light() -> ET.Element:
    """Build the directional sun, with shadows on (they stress the detectors)."""
    light = ET.Element("light", {"type": "directional", "name": "sun"})
    _text(light, "cast_shadows", "true")
    _text(light, "pose", "0 0 10 0 0 0")
    _text(light, "diffuse", "0.8 0.8 0.8 1")
    _text(light, "specular", "0.2 0.2 0.2 1")
    attenuation = ET.SubElement(light, "attenuation")
    _text(attenuation, "range", "1000")
    _text(attenuation, "constant", "0.9")
    _text(attenuation, "linear", "0.01")
    _text(attenuation, "quadratic", "0.001")
    _text(light, "direction", "-0.5 0.1 -0.9")
    return light


def world_element(name: str, params: WorldParams) -> ET.Element:
    """Build a `<world>` with physics, scene, lighting and GPS origin.

    System plugins are deliberately not written here: the gz server is started
    with PX4's `server.config`, which supplies physics, sensors, scene
    broadcaster, IMU, NavSat, magnetometer, air pressure and contact systems —
    the same set stock PX4 SITL uses.
    """
    world = ET.Element("world", {"name": name})
    physics = ET.SubElement(world, "physics", {"name": "default_physics", "type": "ode"})
    _text(physics, "max_step_size", f"{PHYSICS_STEP_S:g}")
    _text(physics, "real_time_factor", "1.0")
    _text(physics, "real_time_update_rate", f"{1.0 / PHYSICS_STEP_S:g}")
    scene = ET.SubElement(world, "scene")
    _text(scene, "ambient", "0.8 0.8 0.8 1")
    _text(scene, "background", "0.56 0.73 0.89 1")
    _text(scene, "shadows", "true")
    spherical = ET.SubElement(world, "spherical_coordinates")
    _text(spherical, "surface_model", "EARTH_WGS84")
    _text(spherical, "world_frame_orientation", "ENU")
    _text(spherical, "latitude_deg", f"{params.origin.lat_deg:.8f}")
    _text(spherical, "longitude_deg", f"{params.origin.lon_deg:.8f}")
    _text(spherical, "elevation", f"{params.elevation_m:g}")
    world.append(sun_light())
    return world


def build_world(layout: FieldLayout, params: WorldParams, ground_texture: str) -> ET.Element:
    """Assemble the complete `<sdf>` document for one field layout.

    Args:
        layout: Field geometry for this seed.
        params: Validated world parameters.
        ground_texture: Relative path of the generated ground texture.

    Returns:
        The root `<sdf>` element.
    """
    root = ET.Element("sdf", {"version": SDF_VERSION})
    world = world_element(f"skyscan_{layout.seed}", params)
    root.append(world)

    world.append(ground_plane_model(params, ground_texture))
    world.append(ground_patch_model(layout.delivery_zone))
    world.append(ground_patch_model(layout.takeoff_zone))
    world.append(qr_model(layout.start_qr))
    for marker in layout.delivery_qrs:
        world.append(qr_model(marker))
    for rect in layout.red_zones:
        world.append(red_zone_model(rect, params.red_zones.rgb))
    for wall in layout.walls:
        world.append(wall_model(wall))
    for obstacle in layout.obstacles:
        world.append(obstacle_model(obstacle))
    for banner in layout.banners:
        world.append(banner_model(banner, layout.banner_rgb))
    return root


def to_xml(root: ET.Element) -> str:
    """Serialise an element tree to indented XML with a declaration."""
    raw = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    # minidom emits its own declaration plus blank lines; normalise both.
    lines = [line for line in pretty.splitlines() if line.strip()]
    if lines and lines[0].startswith("<?xml"):
        lines = lines[1:]
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + "\n".join(lines) + "\n"

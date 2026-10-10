"""Unit tests for the SDF builders (structure, not rendering)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from aero.sim.field import Banner, Box, GroundPatch, QrMarker, Rect, build_field
from aero.sim.sdf import (
    PAINT_THICKNESS_M,
    SDF_VERSION,
    banner_model,
    build_world,
    ground_patch_model,
    qr_model,
    red_zone_model,
    static_box_model,
    to_xml,
    wall_model,
    world_element,
)
from aero.sim.world_params import WorldParams, load_world_params


@pytest.fixture(scope="module")
def params() -> WorldParams:
    """Checked-in world parameters."""
    return load_world_params()


def _find(element: ET.Element, path: str) -> ET.Element:
    """Find a required child, failing the test with the path if absent."""
    found = element.find(path)
    assert found is not None, f"missing {path}"
    return found


def test_static_box_has_collision_and_visual() -> None:
    model = static_box_model("w", (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (0.5, 0.5, 0.5))
    assert model.get("name") == "w"
    assert _find(model, "static").text == "true"
    assert _find(model, "pose").text == "1 2 3 0 0 0"
    assert _find(model, "link/collision/geometry/box/size").text == "4 5 6"
    assert _find(model, "link/visual/geometry/box/size").text == "4 5 6"


def test_paint_has_no_collision_so_it_cannot_block_the_drone() -> None:
    model = static_box_model(
        "p", (0.0, 0.0, 0.01), (1.0, 1.0, 0.01), (1.0, 0.0, 0.0), collide=False
    )
    assert model.find("link/collision") is None
    assert model.find("link/visual") is not None


def test_textured_material_keeps_a_white_base_colour() -> None:
    # A tinted base colour would multiply into the texture and ruin QR contrast.
    model = static_box_model(
        "q", (0.0, 0.0, 0.0), (1.0, 1.0, 0.01), (0.2, 0.9, 0.3), texture="textures/qr.png"
    )
    material = _find(model, "link/visual/material")
    assert _find(material, "diffuse").text == "1 1 1 1"
    assert _find(material, "pbr/metal/albedo_map").text == "textures/qr.png"


def test_flat_colour_material_has_no_texture() -> None:
    model = static_box_model("w", (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (0.2, 0.4, 0.6))
    material = _find(model, "link/visual/material")
    assert material.find("pbr") is None
    assert _find(material, "diffuse").text == "0.2 0.4 0.6 1"


def test_wall_model_uses_the_box_geometry() -> None:
    box = Box(name="wall_x", center_m=(5.0, 1.8, 1.5), size_m=(10.0, 0.1, 3.0), rgb=(0.5, 0.5, 0.5))
    model = wall_model(box)
    assert model.get("name") == "wall_x"
    assert _find(model, "link/collision/geometry/box/size").text == "10 0.1 3"


def test_qr_model_is_thin_square_paint() -> None:
    marker = QrMarker(
        name="start_qr",
        payload="DELIVERY_A",
        center_xy_m=(1.0, 0.0),
        size_m=1.0,
        texture="textures/qr_DELIVERY_A.png",
        is_target=False,
    )
    model = qr_model(marker)
    assert _find(model, "link/visual/geometry/box/size").text == f"1 1 {PAINT_THICKNESS_M:g}"
    assert model.find("link/collision") is None
    assert _find(model, "link/visual/material/pbr/metal/albedo_map").text == marker.texture


def test_ground_patch_uses_its_texture_or_colour() -> None:
    textured = ground_patch_model(
        GroundPatch("zone", (10.0, 0.0), (40.0, 30.0), "textures/zone.png", (0.4, 0.6, 0.3), 0.005)
    )
    assert _find(textured, "link/visual/material/pbr/metal/albedo_map").text == "textures/zone.png"
    plain = ground_patch_model(
        GroundPatch("takeoff", (0.0, 0.0), (3.0, 7.0), None, (0.15, 0.3, 0.75), 0.01)
    )
    assert plain.find("link/visual/material/pbr") is None


def test_red_zone_model_is_painted_in_the_configured_colour(params: WorldParams) -> None:
    model = red_zone_model(Rect("red_zone_0", (30.0, -2.0), (4.0, 4.0)), params.red_zones.rgb)
    assert model.find("link/collision") is None
    expected = " ".join(f"{channel:.6g}" for channel in params.red_zones.rgb) + " 1"
    assert _find(model, "link/visual/material/diffuse").text == expected


def test_banner_is_vertical_and_yawed_to_face_the_drone() -> None:
    banner = Banner(
        name="banner_forward",
        center_m=(3.0, 0.0, 3.7),
        size_m=(2.0, 1.0),
        yaw_deg=180.0,
        texture="textures/banner.png",
    )
    model = banner_model(banner, (0.2, 0.7, 0.3))
    # Thin in x, 2 m across y, 1 m tall in z.
    assert _find(model, "link/visual/geometry/box/size").text == f"{PAINT_THICKNESS_M:g} 2 1"
    pose = _find(model, "pose").text
    assert pose is not None
    yaw = float(pose.split()[-1])
    assert yaw == pytest.approx(math.pi, abs=1e-6)


def test_world_element_has_gps_origin_and_no_plugins(params: WorldParams) -> None:
    world = world_element("skyscan_1", params)
    assert world.get("name") == "skyscan_1"
    spherical = _find(world, "spherical_coordinates")
    assert _find(spherical, "world_frame_orientation").text == "ENU"
    assert float(str(_find(spherical, "latitude_deg").text)) == pytest.approx(params.origin.lat_deg)
    # System plugins come from PX4's server.config, not from the world file.
    assert world.find("plugin") is None
    assert world.find("physics") is not None
    assert world.find("light") is not None


def test_build_world_contains_every_feature(params: WorldParams) -> None:
    layout = build_field(params, 1)
    root = build_world(layout, params, "textures/ground.png")
    assert root.get("version") == SDF_VERSION
    world = _find(root, "world")
    names = {model.get("name") for model in world.findall("model")}
    assert "ground_plane" in names
    assert "takeoff_zone" in names
    assert "delivery_zone" in names
    assert layout.start_qr.name in names
    assert {marker.name for marker in layout.delivery_qrs} <= names
    assert {wall.name for wall in layout.walls} <= names
    assert {obstacle.name for obstacle in layout.obstacles} <= names
    assert {rect.name for rect in layout.red_zones} <= names
    assert {banner.name for banner in layout.banners} <= names


def test_coplanar_paint_layers_do_not_share_a_height(params: WorldParams) -> None:
    layout = build_field(params, 1)
    root = build_world(layout, params, "textures/ground.png")
    world = _find(root, "world")
    heights = {}
    for model in world.findall("model"):
        pose = model.find("pose")
        if pose is None or pose.text is None:
            continue
        heights[model.get("name")] = float(pose.text.split()[2])
    assert heights["delivery_zone"] < heights["takeoff_zone"] < heights[layout.start_qr.name]


def test_to_xml_is_parseable_and_declares_encoding(params: WorldParams) -> None:
    layout = build_field(params, 1)
    text = to_xml(build_world(layout, params, "textures/ground.png"))
    assert text.startswith('<?xml version="1.0" encoding="UTF-8"?>')
    assert ET.fromstring(text).tag == "sdf"
    assert "\n\n" not in text

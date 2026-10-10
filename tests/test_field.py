"""Unit tests for the seeded field layout.

The layout drives the SDF, the mission's field config and the judge's ground
truth, so the geometric guarantees the rulebook implies (lane clearance,
obstacle spacing, QR separation, red-zone clearance) are asserted here across
many seeds rather than eyeballed in one world.
"""

from __future__ import annotations

import colorsys
from itertools import pairwise

import pytest

from aero.core.geo import point_in_polygon
from aero.sim.field import build_field
from aero.sim.world_params import WorldParams, load_world_params

# Enough seeds to exercise every randomised branch (2-4 obstacles, 5-6 QRs,
# 1-2 red zones) without making the suite slow.
SEEDS = range(1, 41)


@pytest.fixture(scope="module")
def params() -> WorldParams:
    """Checked-in world parameters."""
    return load_world_params()


def test_same_seed_gives_the_same_layout(params: WorldParams) -> None:
    assert build_field(params, 7) == build_field(params, 7)


def test_layouts_vary_with_the_seed(params: WorldParams) -> None:
    layouts = [build_field(params, seed) for seed in SEEDS]
    signatures = {
        (
            round(layout.forward_lane.start_x_m, 3),
            len(layout.obstacles),
            len(layout.delivery_qrs),
            len(layout.red_zones),
            layout.target_payload,
            tuple(round(marker.center_xy_m[0], 3) for marker in layout.delivery_qrs),
        )
        for layout in layouts
    }
    # Distinct layouts for distinct seeds; a handful of collisions in the
    # coarse fields would still be fine, so require most to differ.
    assert len(signatures) >= len(layouts) - 2


def test_lanes_share_a_wall(params: WorldParams) -> None:
    layout = build_field(params, 1)
    separation = layout.forward_lane.center_y_m - layout.return_lane.center_y_m
    assert separation == pytest.approx(params.lanes.width_m + params.lanes.wall_thickness_m)
    assert len(layout.walls) == 3
    wall_ys = sorted(wall.center_m[1] for wall in layout.walls)
    gaps = [second - first for first, second in pairwise(wall_ys)]
    for gap in gaps:
        assert gap == pytest.approx(params.lanes.width_m + params.lanes.wall_thickness_m)


def test_walls_span_the_lane_and_reach_full_height(params: WorldParams) -> None:
    layout = build_field(params, 3)
    for wall in layout.walls:
        assert wall.size_m[0] == pytest.approx(params.lanes.length_m)
        assert wall.size_m[1] == pytest.approx(params.lanes.wall_thickness_m)
        assert wall.size_m[2] == pytest.approx(params.lanes.wall_height_m)
        assert wall.center_m[2] == pytest.approx(params.lanes.wall_height_m / 2)


def test_forward_lane_is_clear_of_obstacles(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        for obstacle in layout.obstacles:
            _, min_y, _, _ = obstacle.footprint
            forward_half = params.lanes.width_m / 2
            assert min_y < layout.forward_lane.center_y_m - forward_half, obstacle


@pytest.mark.parametrize("seed", SEEDS)
def test_obstacles_respect_spacing_and_free_gap(params: WorldParams, seed: int) -> None:
    layout = build_field(params, seed)
    settings = params.obstacles
    assert settings.count_range[0] <= len(layout.obstacles) <= settings.count_range[1]
    centers = sorted(obstacle.center_m[0] for obstacle in layout.obstacles)
    for first, second in pairwise(centers):
        assert second - first >= settings.min_spacing_x_m - 1e-6
    lane = layout.return_lane
    for obstacle in layout.obstacles:
        min_x, min_y, max_x, max_y = obstacle.footprint
        assert min_x >= lane.start_x_m
        assert max_x <= lane.end_x_m
        # Attached to one wall, so the free gap is the rest of the lane width.
        assert lane.width_m - obstacle.size_m[1] >= settings.min_free_gap_m - 1e-6
        assert min_y >= lane.center_y_m - lane.width_m / 2 - 1e-6
        assert max_y <= lane.center_y_m + lane.width_m / 2 + 1e-6


def test_obstacles_alternate_sides(params: WorldParams) -> None:
    layout = build_field(params, 5)
    sides = [obstacle.center_m[1] > layout.return_lane.center_y_m for obstacle in layout.obstacles]
    for first, second in pairwise(sides):
        assert first != second


@pytest.mark.parametrize("seed", SEEDS)
def test_delivery_qrs_are_separated_and_inside_the_zone(params: WorldParams, seed: int) -> None:
    layout = build_field(params, seed)
    settings = params.delivery_zone
    assert settings.qr_count_range[0] <= len(layout.delivery_qrs) <= settings.qr_count_range[1]
    zone = layout.delivery_zone_rect
    margin = settings.qr_margin_m + settings.qr_size_m / 2
    for marker in layout.delivery_qrs:
        x, y = marker.center_xy_m
        assert abs(x - zone.center_xy_m[0]) <= zone.size_xy_m[0] / 2 - margin + 1e-6
        assert abs(y - zone.center_xy_m[1]) <= zone.size_xy_m[1] / 2 - margin + 1e-6
    for index, first in enumerate(layout.delivery_qrs):
        for second in layout.delivery_qrs[index + 1 :]:
            distance = (
                (first.center_xy_m[0] - second.center_xy_m[0]) ** 2
                + (first.center_xy_m[1] - second.center_xy_m[1]) ** 2
            ) ** 0.5
            assert distance >= settings.qr_min_spacing_m - 1e-6


def test_payloads_are_unique_and_exactly_one_is_the_target(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        payloads = [marker.payload for marker in layout.delivery_qrs]
        assert len(set(payloads)) == len(payloads)
        targets = [marker for marker in layout.delivery_qrs if marker.is_target]
        assert len(targets) == 1
        assert layout.target_qr is targets[0]


def test_start_qr_payload_matches_the_target_delivery_qr(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        assert layout.start_qr.payload == layout.target_payload
        assert layout.start_qr.payload == layout.target_qr.payload
        assert layout.start_qr.is_target is False
        assert layout.start_qr.center_xy_m == (params.start_qr.offset_x_m, 0.0)
        assert len(layout.all_qrs) == len(layout.delivery_qrs) + 1


@pytest.mark.parametrize("seed", SEEDS)
def test_red_zones_keep_clear_of_qrs(params: WorldParams, seed: int) -> None:
    layout = build_field(params, seed)
    settings = params.red_zones
    assert settings.count_range[0] <= len(layout.red_zones) <= settings.count_range[1]
    clearance = settings.min_clearance_to_qr_m + params.delivery_zone.qr_size_m / 2
    for rect in layout.red_zones:
        half_x, half_y = rect.size_xy_m[0] / 2, rect.size_xy_m[1] / 2
        for marker in layout.delivery_qrs:
            delta_x = abs(rect.center_xy_m[0] - marker.center_xy_m[0])
            delta_y = abs(rect.center_xy_m[1] - marker.center_xy_m[1])
            assert delta_x >= half_x + clearance or delta_y >= half_y + clearance


def test_red_zones_do_not_overlap_each_other(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        for index, first in enumerate(layout.red_zones):
            for second in layout.red_zones[index + 1 :]:
                overlap_x = (
                    abs(first.center_xy_m[0] - second.center_xy_m[0])
                    < (first.size_xy_m[0] + second.size_xy_m[0]) / 2
                )
                overlap_y = (
                    abs(first.center_xy_m[1] - second.center_xy_m[1])
                    < (first.size_xy_m[1] + second.size_xy_m[1]) / 2
                )
                assert not (overlap_x and overlap_y)


def test_banners_sit_above_the_walls_and_face_the_drone(params: WorldParams) -> None:
    layout = build_field(params, 2)
    forward, return_banner = layout.banners
    assert forward.center_m[2] - params.banner.height_m / 2 >= params.lanes.wall_height_m
    assert forward.yaw_deg == 180.0
    assert forward.center_m[0] == pytest.approx(layout.forward_lane.start_x_m)
    assert return_banner.yaw_deg == 0.0
    assert return_banner.center_m[0] == pytest.approx(layout.return_lane.end_x_m)
    assert return_banner.center_m[1] == pytest.approx(layout.return_lane.center_y_m)


def test_banner_green_is_inside_the_configured_hsv_window(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        hue, saturation, value = colorsys.rgb_to_hsv(*layout.banner_rgb)
        assert params.banner.hue_deg_range[0] - 1 <= hue * 360 <= params.banner.hue_deg_range[1] + 1
        assert params.banner.saturation_range[0] - 1e-6 <= saturation
        assert saturation <= params.banner.saturation_range[1] + 1e-6
        assert params.banner.value_range[0] - 1e-6 <= value <= params.banner.value_range[1] + 1e-6


def test_geofence_contains_every_feature(params: WorldParams) -> None:
    for seed in SEEDS:
        layout = build_field(params, seed)
        fence = list(layout.geofence_xy_m)
        points = [
            layout.start_qr.center_xy_m,
            layout.forward_lane.entry_xy_m,
            layout.forward_lane.exit_xy_m,
            layout.return_lane.entry_xy_m,
            layout.return_lane.exit_xy_m,
            (0.0, 0.0),
            *(marker.center_xy_m for marker in layout.delivery_qrs),
            *(rect.center_xy_m for rect in layout.red_zones),
            *(corner for corner in layout.delivery_zone_rect.corners),
        ]
        for point in points:
            assert point_in_polygon(point, fence), (seed, point)


def test_delivery_zone_starts_beyond_the_lane_and_is_centred_on_them(
    params: WorldParams,
) -> None:
    layout = build_field(params, 4)
    zone = layout.delivery_zone_rect
    near_edge_x = zone.center_xy_m[0] - zone.size_xy_m[0] / 2
    assert near_edge_x == pytest.approx(
        layout.forward_lane.end_x_m + params.delivery_zone.gap_beyond_lane_m
    )
    expected_y = (layout.forward_lane.center_y_m + layout.return_lane.center_y_m) / 2
    assert zone.center_xy_m[1] == pytest.approx(expected_y)


def test_rect_corners_wind_consistently() -> None:
    params = load_world_params()
    layout = build_field(params, 1)
    corners = layout.delivery_zone_rect.corners
    assert len(corners) == 4
    assert point_in_polygon(layout.delivery_zone_rect.center_xy_m, list(corners))

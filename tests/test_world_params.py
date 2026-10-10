"""Unit tests for loading and validating `sim/world_params.yaml`."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import pytest
import yaml

from aero.core.errors import ConfigError
from aero.sim.world_params import DEFAULT_PARAMS_PATH, load_world_params, parse_world_params


@pytest.fixture(scope="module")
def raw_params() -> dict[str, Any]:
    """The checked-in parameter file, parsed but not validated."""
    data = yaml.safe_load(DEFAULT_PARAMS_PATH.read_text(encoding="utf-8"))
    assert isinstance(data, dict)
    return data


def test_checked_in_params_load() -> None:
    params = load_world_params()
    assert params.lanes.width_m == 3.5
    assert params.lanes.wall_height_m == pytest.approx(3.05)
    assert params.delivery_zone.size_x_m == 40.0
    assert params.camera_down.width_px == 1280
    assert params.range_down.max_m == 8.0


def test_camera_aspect_ratio() -> None:
    params = load_world_params()
    assert params.camera_down.aspect_ratio == pytest.approx(1280 / 720)


def test_missing_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="cannot read world params"):
        load_world_params(tmp_path / "nope.yaml")


def test_invalid_yaml_is_a_config_error(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("origin: [unclosed\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="not valid YAML"):
        load_world_params(bad)


def test_missing_section_names_the_path(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    del data["lanes"]
    with pytest.raises(ConfigError, match=r"world_params\.lanes: missing"):
        parse_world_params(data)


def test_missing_key_names_the_full_path(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    del data["cameras"]["down"]["hfov_deg"]
    with pytest.raises(ConfigError, match=r"world_params\.cameras\.down\.hfov_deg: missing"):
        parse_world_params(data)


def test_wrong_type_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["lanes"]["width_m"] = "wide"
    with pytest.raises(ConfigError, match="expected a number"):
        parse_world_params(data)


def test_boolean_is_not_accepted_as_a_number(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["lanes"]["width_m"] = True
    with pytest.raises(ConfigError, match="expected a number"):
        parse_world_params(data)


def test_below_minimum_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["cameras"]["down"]["width_px"] = 0
    with pytest.raises(ConfigError, match="below the minimum"):
        parse_world_params(data)


def test_rgb_outside_unit_range_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["ground"]["base_rgb"] = [1.5, 0.4, 0.2]
    with pytest.raises(ConfigError, match=r"outside \[0, 1\]"):
        parse_world_params(data)


def test_inverted_range_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["obstacles"]["width_range_m"] = [1.2, 0.6]
    with pytest.raises(ConfigError, match="exceeds high"):
        parse_world_params(data)


def test_range_sensor_needs_max_above_min(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["ranges"]["down"]["max_m"] = 0.1
    with pytest.raises(ConfigError, match="must exceed min_m"):
        parse_world_params(data)


def test_obstacle_wider_than_the_free_gap_allows_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["obstacles"]["width_range_m"] = [0.6, 2.0]
    with pytest.raises(ConfigError, match="free gap"):
        parse_world_params(data)


def test_banner_below_wall_height_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["banner"]["bottom_z_m"] = 1.0
    with pytest.raises(ConfigError, match="would block the lane"):
        parse_world_params(data)


def test_too_many_obstacles_for_the_lane_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["obstacles"]["count_range"] = [2, 10]
    with pytest.raises(ConfigError, match="of lane but the lane is"):
        parse_world_params(data)


def test_field_larger_than_the_ground_plane_is_rejected(raw_params: dict[str, Any]) -> None:
    data = copy.deepcopy(raw_params)
    data["ground"]["size_m"] = 40.0
    with pytest.raises(ConfigError, match="ground plane only extends"):
        parse_world_params(data)

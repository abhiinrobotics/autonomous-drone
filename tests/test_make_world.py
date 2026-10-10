"""Tests for the world generator: files, SDF validity, GPS, seed variation.

`gz sdf -k` is the real SDF parser, so it is used where available; the tests
skip rather than fail on a machine without Gazebo.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from aero.core.geo import Enu, LatLon, enu_from_gps, haversine_m
from aero.sim.field import build_field
from aero.sim.make_world import (
    FIELD_CONFIG_FILENAME,
    TRUTH_FILENAME,
    WORLD_FILENAME,
    GeneratedWorld,
    build_field_config,
    build_truth,
    generate_world,
)
from aero.sim.world_params import WorldParams, load_world_params

GZ_SDF_TIMEOUT_S = 30.0
# The generator is the slow part (texture rendering), so generate each seed
# once per module and share it.
SEEDS = (1, 2)


@pytest.fixture(scope="module")
def params() -> WorldParams:
    """Checked-in world parameters."""
    return load_world_params()


@pytest.fixture(scope="module")
def worlds(
    tmp_path_factory: pytest.TempPathFactory, params: WorldParams
) -> dict[int, GeneratedWorld]:
    """Generate the test seeds once into a temporary directory."""
    out_dir = tmp_path_factory.mktemp("worlds")
    return {seed: generate_world(seed, out_dir, params) for seed in SEEDS}


def test_generator_writes_every_artefact(worlds: dict[int, GeneratedWorld]) -> None:
    generated = worlds[1]
    assert generated.world_path.name == WORLD_FILENAME
    assert generated.field_config_path.name == FIELD_CONFIG_FILENAME
    assert generated.truth_path.name == TRUTH_FILENAME
    for path in (generated.world_path, generated.field_config_path, generated.truth_path):
        assert path.is_file()
    textures = sorted(path.name for path in (generated.directory / "textures").iterdir())
    assert "ground.png" in textures
    assert "zone_ground.png" in textures
    assert "banner.png" in textures
    assert any(name.startswith("qr_") for name in textures)


def test_every_texture_referenced_by_the_world_exists(worlds: dict[int, GeneratedWorld]) -> None:
    generated = worlds[1]
    root = ET.fromstring(generated.world_path.read_text(encoding="utf-8"))
    referenced = {element.text for element in root.iter("albedo_map") if element.text}
    assert referenced
    for relative in referenced:
        assert (generated.directory / relative).is_file(), relative


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz is not installed")
@pytest.mark.parametrize("seed", SEEDS)
def test_generated_world_passes_gz_sdf_check(worlds: dict[int, GeneratedWorld], seed: int) -> None:
    result = subprocess.run(
        ["gz", "sdf", "-k", str(worlds[seed].world_path)],
        capture_output=True,
        text=True,
        timeout=GZ_SDF_TIMEOUT_S,
        check=False,
    )
    output = result.stdout + result.stderr
    assert "Valid." in output, output
    # Warnings are errors here: an unresolved texture or a bad element would
    # otherwise only show up as a blank world.
    assert "Warning" not in output, output


def test_worlds_differ_between_seeds(worlds: dict[int, GeneratedWorld]) -> None:
    first = worlds[1].world_path.read_text(encoding="utf-8")
    second = worlds[2].world_path.read_text(encoding="utf-8")
    assert first != second
    assert worlds[1].world_name != worlds[2].world_name


def test_regenerating_a_seed_is_byte_identical(
    tmp_path: Path, params: WorldParams, worlds: dict[int, GeneratedWorld]
) -> None:
    again = generate_world(1, tmp_path, params)
    assert again.world_path.read_text(encoding="utf-8") == worlds[1].world_path.read_text(
        encoding="utf-8"
    )


def test_field_config_gps_round_trips_to_the_local_frame(
    worlds: dict[int, GeneratedWorld], params: WorldParams
) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    origin = LatLon(config["origin"]["lat_deg"], config["origin"]["lon_deg"])
    assert origin.lat_deg == pytest.approx(params.origin.lat_deg)
    for entry in [config["start_qr"], *config["delivery_qrs"]]:
        recovered = enu_from_gps(origin, LatLon(entry["gps"]["lat_deg"], entry["gps"]["lon_deg"]))
        assert recovered.east_m == pytest.approx(entry["enu"]["x_m"], abs=0.05)
        assert recovered.north_m == pytest.approx(entry["enu"]["y_m"], abs=0.05)


def test_field_config_home_is_the_origin(worlds: dict[int, GeneratedWorld]) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    assert config["home"]["enu"] == {"x_m": 0.0, "y_m": 0.0}
    home = LatLon(config["home"]["gps"]["lat_deg"], config["home"]["gps"]["lon_deg"])
    origin = LatLon(config["origin"]["lat_deg"], config["origin"]["lon_deg"])
    assert haversine_m(home, origin) == pytest.approx(0.0, abs=0.01)


def test_field_config_describes_both_lanes(
    worlds: dict[int, GeneratedWorld], params: WorldParams
) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    assert set(config["lanes"]) == {"forward", "return"}
    for lane in config["lanes"].values():
        assert lane["width_m"] == params.lanes.width_m
        assert lane["exit"]["enu"]["x_m"] > lane["entry"]["enu"]["x_m"]
        for point in (lane["entry"], lane["exit"]):
            assert "lat_deg" in point["gps"]


def test_field_config_target_matches_exactly_one_delivery_qr(
    worlds: dict[int, GeneratedWorld],
) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    targets = [entry for entry in config["delivery_qrs"] if entry["is_target"]]
    assert len(targets) == 1
    assert targets[0]["payload"] == config["target_payload"]
    assert config["start_qr"]["payload"] == config["target_payload"]


def test_field_config_red_zones_and_geofence_are_polygons(
    worlds: dict[int, GeneratedWorld],
) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    assert len(config["geofence"]) == 4
    for zone in config["red_zones"]:
        assert len(zone["polygon"]) == 4
        for corner in zone["polygon"]:
            assert {"enu", "gps"} <= set(corner)


def test_truth_has_what_the_judge_needs(
    worlds: dict[int, GeneratedWorld], params: WorldParams
) -> None:
    truth = json.loads(worlds[1].truth_path.read_text(encoding="utf-8"))
    layout = build_field(params, 1)
    assert truth["world_name"] == worlds[1].world_name
    assert truth["target_payload"] == layout.target_payload
    assert truth["drone_radius_m"] == params.vehicle.radius_m
    assert len(truth["walls"]) == len(layout.walls)
    assert len(truth["obstacles"]) == len(layout.obstacles)
    assert len(truth["delivery_qrs"]) == len(layout.delivery_qrs)
    assert set(truth["lanes"]) == {"forward", "return"}


def test_field_config_and_truth_agree_on_the_target(
    worlds: dict[int, GeneratedWorld],
) -> None:
    config = yaml.safe_load(worlds[1].field_config_path.read_text(encoding="utf-8"))
    truth = json.loads(worlds[1].truth_path.read_text(encoding="utf-8"))
    assert config["target_payload"] == truth["target_payload"]
    assert config["start_qr"]["enu"] != truth["target_qr"]["enu"]


def test_builders_are_pure_functions_of_the_layout(params: WorldParams) -> None:
    layout = build_field(params, 3)
    assert build_field_config(layout, params) == build_field_config(layout, params)
    assert build_truth(layout, params) == build_truth(layout, params)


def test_gps_offsets_match_the_local_offsets(params: WorldParams) -> None:
    layout = build_field(params, 1)
    config = build_field_config(layout, params)
    origin = LatLon(params.origin.lat_deg, params.origin.lon_deg)
    zone = config["delivery_zone"]
    expected = Enu(east_m=zone["enu"]["x_m"], north_m=zone["enu"]["y_m"], up_m=0.0)
    recovered = enu_from_gps(origin, LatLon(zone["gps"]["lat_deg"], zone["gps"]["lon_deg"]))
    assert recovered.east_m == pytest.approx(expected.east_m, abs=0.05)
    assert recovered.north_m == pytest.approx(expected.north_m, abs=0.05)

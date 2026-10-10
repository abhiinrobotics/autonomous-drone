"""Unit tests for the gz server configuration and service requests."""

from __future__ import annotations

import math
import os
from pathlib import Path

import pytest

from aero.core.errors import ConfigError
from aero.sim.gz_server import GzError, GzServerConfig, spawn_request, world_name_from_sdf

_WORLD_SDF = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="skyscan_7">
    <physics name="default_physics" type="ode"><max_step_size>0.004</max_step_size></physics>
  </world>
</sdf>
"""


def _world_file(tmp_path: Path, text: str = _WORLD_SDF) -> Path:
    """Write a world SDF into `tmp_path`."""
    path = tmp_path / "world.sdf"
    path.write_text(text, encoding="utf-8")
    return path


def test_world_name_is_read_from_the_sdf(tmp_path: Path) -> None:
    assert world_name_from_sdf(_world_file(tmp_path)) == "skyscan_7"


def test_missing_world_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="cannot read world"):
        world_name_from_sdf(tmp_path / "absent.sdf")


def test_malformed_world_file_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not valid XML"):
        world_name_from_sdf(_world_file(tmp_path, "<sdf><world>"))


def test_world_without_a_name_is_a_config_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="no <world name"):
        world_name_from_sdf(_world_file(tmp_path, "<sdf version='1.9'><world/></sdf>"))


def test_argv_runs_a_headless_rendering_server(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=_world_file(tmp_path))
    argv = config.argv()
    assert argv[:4] == ["gz", "sim", "-s", "-r"]
    # Without this flag the camera sensors never render on a headless server.
    assert "--headless-rendering" in argv
    assert argv[-1] == str(config.world_path)


def test_argv_can_run_with_a_gui_render_path(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=_world_file(tmp_path), headless_rendering=False)
    assert "--headless-rendering" not in config.argv()


def test_env_puts_the_world_directory_first_on_the_resource_path(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=_world_file(tmp_path))
    env = config.env({"PATH": "/usr/bin"})
    paths = env["GZ_SIM_RESOURCE_PATH"].split(os.pathsep)
    # The world's own directory must come first so its relative texture paths
    # resolve before anything else with the same name.
    assert paths[0] == str(tmp_path)
    assert str(config.px4_models_dir) in paths
    assert env["GZ_SIM_SERVER_CONFIG_PATH"] == str(config.px4_server_config)
    assert env["PATH"] == "/usr/bin"


def test_env_keeps_an_existing_resource_path(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=_world_file(tmp_path))
    env = config.env({"GZ_SIM_RESOURCE_PATH": "/opt/models"})
    assert "/opt/models" in env["GZ_SIM_RESOURCE_PATH"].split(os.pathsep)


def test_extra_resource_paths_are_included(tmp_path: Path) -> None:
    extra = tmp_path / "models"
    config = GzServerConfig(world_path=_world_file(tmp_path), extra_resource_paths=(extra,))
    assert str(extra) in config.env({})["GZ_SIM_RESOURCE_PATH"].split(os.pathsep)


def test_validate_rejects_a_missing_world(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=tmp_path / "nope.sdf")
    with pytest.raises(GzError, match="world not found"):
        config.validate()


def test_validate_rejects_a_missing_px4_checkout(tmp_path: Path) -> None:
    config = GzServerConfig(world_path=_world_file(tmp_path), px4_dir=tmp_path / "no_px4")
    with pytest.raises(GzError, match="PX4 gz models not found"):
        config.validate()


def test_spawn_request_carries_pose_and_model_path(tmp_path: Path) -> None:
    request = spawn_request("aero_x500_0", tmp_path / "model.sdf", (1.0, -2.0, 0.25), 0.0)
    assert 'name: "aero_x500_0"' in request
    assert "allow_renaming: false" in request
    assert f'sdf_filename: "{tmp_path / "model.sdf"}"' in request
    assert "x: 1, y: -2, z: 0.25" in request
    assert "z: 0, w: 1" in request


def test_spawn_request_encodes_yaw_as_a_quaternion(tmp_path: Path) -> None:
    request = spawn_request("m", tmp_path / "model.sdf", (0.0, 0.0, 0.0), math.pi / 2)
    expected_z = math.sin(math.pi / 4)
    assert f"z: {expected_z:.9g}" in request

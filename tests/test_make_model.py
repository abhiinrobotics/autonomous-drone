"""Unit tests for the generated vehicle model."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from aero.sim.make_model import (
    MODEL_CONFIG_FILENAME,
    MODEL_FILENAME,
    build_model,
    build_model_config,
    generate_model,
    topics,
)
from aero.sim.world_params import WorldParams, load_world_params


@pytest.fixture(scope="module")
def params() -> WorldParams:
    """Checked-in world parameters."""
    return load_world_params()


@pytest.fixture(scope="module")
def model(params: WorldParams) -> ET.Element:
    """The generated model document."""
    return build_model(params)


def _sensor(model: ET.Element, name: str) -> ET.Element:
    """Find a sensor by name."""
    found = model.find(f".//sensor[@name='{name}']")
    assert found is not None, f"missing sensor {name}"
    return found


def test_topics_are_prefixed_with_the_model_name() -> None:
    sensor_topics = topics("aero_x500")
    assert sensor_topics.down_image == "/aero_x500/down_cam/image"
    assert sensor_topics.front_image == "/aero_x500/front_cam/image"
    assert sensor_topics.front_scan == "/aero_x500/front_range/scan"
    assert sensor_topics.down_scan == "/aero_x500/down_range/scan"


def test_model_merges_the_px4_x500(model: ET.Element, params: WorldParams) -> None:
    root = model.find("model")
    assert root is not None
    assert root.get("name") == params.vehicle.model_name
    include = root.find("include")
    assert include is not None
    # merge keeps x500's link, joint and motor-plugin names, which is what the
    # PX4 gz_bridge actuator mapping relies on.
    assert include.get("merge") == "true"
    assert include.findtext("uri") == "model://x500"


def test_every_sensor_link_is_fixed_to_base_link(model: ET.Element) -> None:
    joints = {joint.get("name"): joint for joint in model.findall(".//joint")}
    for name in ("down_cam", "front_cam", "front_range", "down_range"):
        joint = joints[f"{name}_joint"]
        assert joint.get("type") == "fixed"
        assert joint.findtext("parent") == "base_link"
        assert joint.findtext("child") == f"{name}_link"


def test_down_camera_points_at_the_ground(model: ET.Element) -> None:
    link = model.find(".//link[@name='down_cam_link']")
    assert link is not None
    pose = link.findtext("pose")
    assert pose is not None
    values = [float(value) for value in pose.split()]
    # gz pitch +90 deg looks down, matching PX4's own x500_mono_cam_down.
    assert values[4] == pytest.approx(math.pi / 2, abs=1e-3)
    # Mounted below base_link: FRD +z down becomes gz -z.
    assert values[2] < 0.0


def test_front_camera_looks_forward(model: ET.Element) -> None:
    link = model.find(".//link[@name='front_cam_link']")
    assert link is not None
    values = [float(value) for value in str(link.findtext("pose")).split()]
    assert values[3:] == pytest.approx([0.0, 0.0, 0.0])
    assert values[0] > 0.0


def test_camera_sensor_uses_configured_resolution_and_fov(
    model: ET.Element, params: WorldParams
) -> None:
    sensor = _sensor(model, "down_cam")
    assert sensor.get("type") == "camera"
    assert sensor.findtext("topic") == topics(params.vehicle.model_name).down_image
    assert float(str(sensor.findtext("update_rate"))) == params.camera_down.update_rate_hz
    camera = sensor.find("camera")
    assert camera is not None
    assert float(str(camera.findtext("horizontal_fov"))) == pytest.approx(
        math.radians(params.camera_down.hfov_deg), abs=1e-5
    )
    image = camera.find("image")
    assert image is not None
    assert int(str(image.findtext("width"))) == params.camera_down.width_px
    assert int(str(image.findtext("height"))) == params.camera_down.height_px
    assert image.findtext("format") == "R8G8B8"


def test_cameras_have_noise_so_tests_are_not_unrealistically_clean(
    model: ET.Element, params: WorldParams
) -> None:
    noise = _sensor(model, "front_cam").find("camera/noise")
    assert noise is not None
    assert float(str(noise.findtext("stddev"))) == params.camera_front.noise_stddev


def test_range_sensors_emulate_a_narrow_single_beam(model: ET.Element, params: WorldParams) -> None:
    sensor = _sensor(model, "front_range")
    assert sensor.get("type") == "gpu_lidar"
    lidar = sensor.find("lidar")
    assert lidar is not None
    half_beam = math.radians(params.range_front.beam_deg) / 2
    for axis in ("horizontal", "vertical"):
        block = lidar.find(f"scan/{axis}")
        assert block is not None
        assert int(str(block.findtext("samples"))) == params.range_front.rays
        assert float(str(block.findtext("min_angle"))) == pytest.approx(-half_beam, abs=1e-5)
        assert float(str(block.findtext("max_angle"))) == pytest.approx(half_beam, abs=1e-5)
    range_block = lidar.find("range")
    assert range_block is not None
    assert float(str(range_block.findtext("min"))) == params.range_front.min_m
    assert float(str(range_block.findtext("max"))) == params.range_front.max_m


def test_down_range_is_clipped_at_the_tf_luna_limit(model: ET.Element, params: WorldParams) -> None:
    # Deliberate: the real TF-Luna cannot see past ~8 m, so the mission must
    # not wait on it at survey altitude.
    maximum = _sensor(model, "down_range").find("lidar/range")
    assert maximum is not None
    assert float(str(maximum.findtext("max"))) == 8.0
    assert params.range_down.max_m == 8.0


def test_model_config_points_at_the_sdf(params: WorldParams) -> None:
    text = build_model_config(params.vehicle.model_name)
    root = ET.fromstring(text)
    assert root.findtext("name") == params.vehicle.model_name
    sdf = root.find("sdf")
    assert sdf is not None
    assert sdf.text == MODEL_FILENAME


def test_generate_model_writes_both_files(tmp_path: Path, params: WorldParams) -> None:
    path = generate_model(tmp_path, params)
    assert path == tmp_path / params.vehicle.model_name / MODEL_FILENAME
    assert path.is_file()
    assert (path.parent / MODEL_CONFIG_FILENAME).is_file()
    assert ET.fromstring(path.read_text(encoding="utf-8")).tag == "sdf"

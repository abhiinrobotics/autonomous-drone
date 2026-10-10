"""Generate the `aero_x500` vehicle model: PX4's x500 plus our four sensors.

The model wraps PX4's stock `x500` with `<include merge="true">`, which flattens
its links, joints and motor plugins into our model so the gz_bridge actuator
mapping keeps working (the same trick PX4's own `x500_mono_cam_down` uses).
On top of that we add, as links with fixed joints to `base_link`:

* `down_cam`   — ground-facing camera (QR, banner-free navigation, delivery)
* `front_cam`  — forward camera (banner, corridor, obstacles)
* `front_range`— narrow gpu_lidar emulating a TFmini Plus (3.6 deg beam, 12 m)
* `down_range` — narrow gpu_lidar emulating a TF-Luna (clipped at 8 m)

Sensor topics are fixed and documented (see `topics()`), so the HAL does not
have to reconstruct gz's auto-generated topic names.

The file is written to `sim/models/aero_x500/`, which is on the gz resource
path. `gz sdf -k` cannot validate it (it refuses to resolve the `model://x500`
include), so the model is checked by spawning it and asserting that every
sensor topic publishes — see `tests/test_part1_sitl.py`.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from aero.core.errors import ConfigError
from aero.core.geo import frd_to_gz_pose
from aero.sim.sdf import SDF_VERSION, to_xml
from aero.sim.world_params import CameraParams, RangeParams, WorldParams, load_world_params

logger = logging.getLogger(__name__)

MODEL_FILENAME = "model.sdf"
MODEL_CONFIG_FILENAME = "model.config"
DEFAULT_MODELS_DIR = Path(__file__).resolve().parents[2] / "sim" / "models"
# Link mass/inertia for the add-on sensors, matching PX4's own `mono_cam` and
# `LW20` models so the x500's flight behaviour is not noticeably changed.
_CAMERA_MASS_KG = 0.05
_CAMERA_INERTIA = 4e-5
_RANGE_MASS_KG = 0.02
_RANGE_INERTIA = 4e-6
# Camera clip planes, metres. Far must exceed the 10 m survey altitude plus the
# field diagonal so the whole delivery zone renders.
_CAMERA_NEAR_M = 0.1
_CAMERA_FAR_M = 500.0
# gz lidar needs an angular resolution value; 1 means "one ray per sample".
_LIDAR_RESOLUTION = 1


@dataclass(frozen=True, slots=True)
class SensorTopics:
    """gz topic names published by the generated model.

    Args:
        down_image: Down camera image topic.
        front_image: Front camera image topic.
        front_scan: Front rangefinder LaserScan topic.
        down_scan: Down rangefinder LaserScan topic.
    """

    down_image: str
    front_image: str
    front_scan: str
    down_scan: str


def topics(model_name: str) -> SensorTopics:
    """Topic names for a model generated under `model_name`."""
    return SensorTopics(
        down_image=f"/{model_name}/down_cam/image",
        front_image=f"/{model_name}/front_cam/image",
        front_scan=f"/{model_name}/front_range/scan",
        down_scan=f"/{model_name}/down_range/scan",
    )


def _text(parent: ET.Element, tag: str, value: str) -> ET.Element:
    """Append `<tag>value</tag>` and return it."""
    element = ET.SubElement(parent, tag)
    element.text = value
    return element


def _pose_text(
    xyz_frd_m: tuple[float, float, float], rpy_frd_deg: tuple[float, float, float]
) -> str:
    """Format a body-FRD mount as an SDF pose string in the gz model frame."""
    rpy_rad = tuple(math.radians(angle) for angle in rpy_frd_deg)
    xyz, rpy = frd_to_gz_pose(xyz_frd_m, (rpy_rad[0], rpy_rad[1], rpy_rad[2]))
    return " ".join(f"{value:.9g}" for value in (*xyz, *rpy))


def _inertial(link: ET.Element, mass_kg: float, inertia: float) -> None:
    """Attach a small diagonal inertial block to a sensor link."""
    inertial = ET.SubElement(link, "inertial")
    _text(inertial, "mass", f"{mass_kg:g}")
    tensor = ET.SubElement(inertial, "inertia")
    for axis, value in (("ixx", inertia), ("iyy", inertia), ("izz", inertia)):
        _text(tensor, axis, f"{value:g}")
    for axis in ("ixy", "ixz", "iyz"):
        _text(tensor, axis, "0")


def _sensor_link(name: str, pose: str, mass_kg: float, inertia: float) -> ET.Element:
    """Build a sensor link with its pose relative to `base_link`."""
    link = ET.Element("link", {"name": f"{name}_link"})
    _text(link, "pose", pose)
    _inertial(link, mass_kg, inertia)
    return link


def _fixed_joint(name: str) -> ET.Element:
    """Build a fixed joint tying a sensor link to `base_link`."""
    joint = ET.Element("joint", {"name": f"{name}_joint", "type": "fixed"})
    _text(joint, "parent", "base_link")
    _text(joint, "child", f"{name}_link")
    return joint


def _camera_sensor(name: str, params: CameraParams, image_topic: str) -> ET.Element:
    """Build a `<sensor type="camera">` element from camera parameters."""
    sensor = ET.Element("sensor", {"name": name, "type": "camera"})
    _text(sensor, "gz_frame_id", f"{name}_link")
    _text(sensor, "topic", image_topic)
    _text(sensor, "update_rate", f"{params.update_rate_hz:g}")
    _text(sensor, "always_on", "1")
    _text(sensor, "visualize", "false")
    camera = ET.SubElement(sensor, "camera")
    # The camera's own pose is identity: the mount is on the link, so there is
    # exactly one place where the mount geometry lives.
    _text(camera, "camera_info_topic", f"{image_topic}/camera_info")
    _text(camera, "horizontal_fov", f"{math.radians(params.hfov_deg):.9g}")
    image = ET.SubElement(camera, "image")
    _text(image, "width", str(params.width_px))
    _text(image, "height", str(params.height_px))
    _text(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _text(clip, "near", f"{_CAMERA_NEAR_M:g}")
    _text(clip, "far", f"{_CAMERA_FAR_M:g}")
    if params.noise_stddev > 0.0:
        noise = ET.SubElement(camera, "noise")
        _text(noise, "type", "gaussian")
        _text(noise, "mean", "0")
        _text(noise, "stddev", f"{params.noise_stddev:g}")
    return sensor


def _range_sensor(name: str, params: RangeParams, scan_topic: str) -> ET.Element:
    """Build a narrow `gpu_lidar` that emulates a single-beam rangefinder.

    A real TFmini/TF-Luna reports one distance over a small cone. gz has no
    such sensor, so we sample a small `rays x rays` grid across the beam and
    let the HAL reduce it (minimum) to one range.
    """
    half_beam_rad = math.radians(params.beam_deg) / 2.0
    sensor = ET.Element("sensor", {"name": name, "type": "gpu_lidar"})
    _text(sensor, "gz_frame_id", f"{name}_link")
    _text(sensor, "topic", scan_topic)
    _text(sensor, "update_rate", f"{params.update_rate_hz:g}")
    _text(sensor, "always_on", "1")
    _text(sensor, "visualize", "false")
    lidar = ET.SubElement(sensor, "lidar")
    scan = ET.SubElement(lidar, "scan")
    for axis in ("horizontal", "vertical"):
        block = ET.SubElement(scan, axis)
        _text(block, "samples", str(params.rays))
        _text(block, "resolution", str(_LIDAR_RESOLUTION))
        _text(block, "min_angle", f"{-half_beam_rad:.9g}")
        _text(block, "max_angle", f"{half_beam_rad:.9g}")
    range_block = ET.SubElement(lidar, "range")
    _text(range_block, "min", f"{params.min_m:g}")
    _text(range_block, "max", f"{params.max_m:g}")
    _text(range_block, "resolution", "0.01")
    if params.noise_stddev_m > 0.0:
        noise = ET.SubElement(lidar, "noise")
        _text(noise, "type", "gaussian")
        _text(noise, "mean", "0")
        _text(noise, "stddev", f"{params.noise_stddev_m:g}")
    return sensor


def build_model(params: WorldParams) -> ET.Element:
    """Assemble the `<sdf>` document for the vehicle model.

    Args:
        params: Validated world parameters (camera and range mounts).

    Returns:
        Root `<sdf>` element.
    """
    model_name = params.vehicle.model_name
    sensor_topics = topics(model_name)
    root = ET.Element("sdf", {"version": SDF_VERSION})
    model = ET.SubElement(root, "model", {"name": model_name})
    _text(model, "self_collide", "false")
    include = ET.SubElement(model, "include", {"merge": "true"})
    _text(include, "uri", "model://x500")

    cameras = (("down_cam", params.camera_down), ("front_cam", params.camera_front))
    for name, camera_params in cameras:
        pose = _pose_text(camera_params.mount_xyz_m, camera_params.mount_rpy_deg)
        link = _sensor_link(name, pose, _CAMERA_MASS_KG, _CAMERA_INERTIA)
        image_topic = sensor_topics.down_image if name == "down_cam" else sensor_topics.front_image
        link.append(_camera_sensor(name, camera_params, image_topic))
        model.append(link)
        model.append(_fixed_joint(name))

    ranges = (("front_range", params.range_front), ("down_range", params.range_down))
    for name, range_params in ranges:
        pose = _pose_text(range_params.mount_xyz_m, range_params.mount_rpy_deg)
        link = _sensor_link(name, pose, _RANGE_MASS_KG, _RANGE_INERTIA)
        scan_topic = sensor_topics.front_scan if name == "front_range" else sensor_topics.down_scan
        link.append(_range_sensor(name, range_params, scan_topic))
        model.append(link)
        model.append(_fixed_joint(name))

    return root


def build_model_config(model_name: str) -> str:
    """Build the `model.config` manifest gz uses to find the model."""
    config = ET.Element("model")
    _text(config, "name", model_name)
    _text(config, "version", "1.0")
    sdf = ET.SubElement(config, "sdf", {"version": SDF_VERSION})
    sdf.text = MODEL_FILENAME
    _text(config, "description", "x500 with down/front cameras and down/front rangefinders")
    return to_xml(config)


def generate_model(out_dir: Path | None = None, params: WorldParams | None = None) -> Path:
    """Write the vehicle model to `<out_dir>/<model_name>/`.

    Args:
        out_dir: Models directory; defaults to `sim/models`.
        params: Pre-loaded parameters; loaded from the default path if None.

    Returns:
        Path of the written `model.sdf`.

    Raises:
        ConfigError: The model directory could not be written.
    """
    world_params = load_world_params() if params is None else params
    model_name = world_params.vehicle.model_name
    directory = (DEFAULT_MODELS_DIR if out_dir is None else out_dir) / model_name
    try:
        directory.mkdir(parents=True, exist_ok=True)
        model_path = directory / MODEL_FILENAME
        model_path.write_text(to_xml(build_model(world_params)), encoding="utf-8")
        (directory / MODEL_CONFIG_FILENAME).write_text(
            build_model_config(model_name), encoding="utf-8"
        )
    except OSError as error:
        raise ConfigError(f"cannot write model to {directory}: {error}") from error
    logger.info("model_generated name=%s path=%s", model_name, model_path)
    return model_path

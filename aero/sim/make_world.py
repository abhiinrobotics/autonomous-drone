"""Generate a seeded SkyScan world: SDF, textures, field GPS config, ground truth.

Library half of the generator; the command line lives in `scripts/make_world.py`:

    python scripts/make_world.py --seed 1
    python scripts/make_world.py --seed 1 --out-dir sim/worlds

Outputs under `<out-dir>/<seed>/`:
    world.sdf                  static field geometry (no vehicle, no plugins)
    textures/*.png             ground, delivery-zone ground, banner, QR codes
    mission_config_field.yaml  GPS/ENU of every feature, geofence, red zones
    truth.json                 ground truth for the judge

The vehicle is not part of the world file: it is spawned into the running
simulation (see `aero.sim.gz_server`), which keeps `world.sdf` free of
`<include>` elements that `gz sdf -k` cannot resolve.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from aero.core.geo import Enu, LatLon, gps_from_enu
from aero.sim.field import FieldLayout, QrMarker, build_field
from aero.sim.sdf import build_world, to_xml
from aero.sim.textures import make_banner_png, make_ground_png, make_qr_png, write_png
from aero.sim.world_params import WorldParams, load_world_params

logger = logging.getLogger(__name__)

WORLD_FILENAME = "world.sdf"
FIELD_CONFIG_FILENAME = "mission_config_field.yaml"
TRUTH_FILENAME = "truth.json"
TEXTURE_DIRNAME = "textures"
GROUND_TEXTURE = f"{TEXTURE_DIRNAME}/ground.png"
ZONE_TEXTURE = f"{TEXTURE_DIRNAME}/zone_ground.png"
BANNER_TEXTURE = f"{TEXTURE_DIRNAME}/banner.png"
DEFAULT_WORLDS_DIR = Path(__file__).resolve().parents[2] / "sim" / "worlds"


@dataclass(frozen=True, slots=True)
class GeneratedWorld:
    """Paths produced for one seed.

    Args:
        seed: Layout seed.
        world_name: `<world name=...>` inside the SDF, e.g. `skyscan_1`.
        directory: Output directory for this seed.
        world_path: The generated `world.sdf`.
        field_config_path: The generated `mission_config_field.yaml`.
        truth_path: The generated `truth.json`.
    """

    seed: int
    world_name: str
    directory: Path
    world_path: Path
    field_config_path: Path
    truth_path: Path


def _latlon(origin: LatLon, xy_m: tuple[float, float]) -> dict[str, float]:
    """Convert a field point to a lat/lon mapping for YAML."""
    point = gps_from_enu(origin, Enu(east_m=xy_m[0], north_m=xy_m[1], up_m=0.0))
    return {"lat_deg": round(point.lat_deg, 9), "lon_deg": round(point.lon_deg, 9)}


def _xy(xy_m: tuple[float, float]) -> dict[str, float]:
    """Round a local ENU point for YAML/JSON output."""
    return {"x_m": round(xy_m[0], 4), "y_m": round(xy_m[1], 4)}


def _qr_entry(marker: QrMarker, origin: LatLon) -> dict[str, Any]:
    """Describe one QR marker for the field config."""
    return {
        "name": marker.name,
        "payload": marker.payload,
        "is_target": marker.is_target,
        "size_m": marker.size_m,
        "enu": _xy(marker.center_xy_m),
        "gps": _latlon(origin, marker.center_xy_m),
    }


def build_field_config(layout: FieldLayout, params: WorldParams) -> dict[str, Any]:
    """Build the mission field configuration (GPS plus local ENU of everything).

    Mission code consumes this file; it must therefore contain every position
    the mission needs and no simulator-only detail.
    """
    origin = params.origin
    lanes = {}
    for lane in (layout.forward_lane, layout.return_lane):
        lanes[lane.name] = {
            "center_y_m": round(lane.center_y_m, 4),
            "width_m": lane.width_m,
            "wall_height_m": params.lanes.wall_height_m,
            "entry": {"enu": _xy(lane.entry_xy_m), "gps": _latlon(origin, lane.entry_xy_m)},
            "exit": {"enu": _xy(lane.exit_xy_m), "gps": _latlon(origin, lane.exit_xy_m)},
        }
    return {
        "seed": layout.seed,
        "origin": {
            "lat_deg": origin.lat_deg,
            "lon_deg": origin.lon_deg,
            "elevation_m": params.elevation_m,
        },
        "home": {"enu": _xy((0.0, 0.0)), "gps": _latlon(origin, (0.0, 0.0))},
        "takeoff_zone": {
            "size_x_m": layout.takeoff_zone.size_xy_m[0],
            "size_y_m": layout.takeoff_zone.size_xy_m[1],
            "enu": _xy(layout.takeoff_zone.center_xy_m),
            "gps": _latlon(origin, layout.takeoff_zone.center_xy_m),
        },
        "lanes": lanes,
        "start_qr": _qr_entry(layout.start_qr, origin),
        "target_payload": layout.target_payload,
        "delivery_zone": {
            "size_x_m": layout.delivery_zone_rect.size_xy_m[0],
            "size_y_m": layout.delivery_zone_rect.size_xy_m[1],
            "enu": _xy(layout.delivery_zone_rect.center_xy_m),
            "gps": _latlon(origin, layout.delivery_zone_rect.center_xy_m),
            "corners": [
                {"enu": _xy(corner), "gps": _latlon(origin, corner)}
                for corner in layout.delivery_zone_rect.corners
            ],
        },
        "delivery_qrs": [_qr_entry(marker, origin) for marker in layout.delivery_qrs],
        "red_zones": [
            {
                "name": rect.name,
                "polygon": [
                    {"enu": _xy(corner), "gps": _latlon(origin, corner)} for corner in rect.corners
                ],
            }
            for rect in layout.red_zones
        ],
        "geofence": [
            {"enu": _xy(corner), "gps": _latlon(origin, corner)} for corner in layout.geofence_xy_m
        ],
    }


def build_truth(layout: FieldLayout, params: WorldParams) -> dict[str, Any]:
    """Build the judge's ground truth: exact geometry the drone must not know."""
    return {
        "seed": layout.seed,
        "world_name": f"skyscan_{layout.seed}",
        "target_payload": layout.target_payload,
        "target_qr": {
            "payload": layout.target_qr.payload,
            "enu": _xy(layout.target_qr.center_xy_m),
        },
        "drone_radius_m": params.vehicle.radius_m,
        "walls": [
            {"name": box.name, "center_m": list(box.center_m), "size_m": list(box.size_m)}
            for box in layout.walls
        ],
        "obstacles": [
            {"name": box.name, "center_m": list(box.center_m), "size_m": list(box.size_m)}
            for box in layout.obstacles
        ],
        "red_zones": [
            {"name": rect.name, "polygon": [list(corner) for corner in rect.corners]}
            for rect in layout.red_zones
        ],
        "delivery_qrs": [
            {
                "name": marker.name,
                "payload": marker.payload,
                "is_target": marker.is_target,
                "enu": _xy(marker.center_xy_m),
            }
            for marker in layout.delivery_qrs
        ],
        "lanes": {
            lane.name: {
                "center_y_m": lane.center_y_m,
                "start_x_m": lane.start_x_m,
                "end_x_m": lane.end_x_m,
                "width_m": lane.width_m,
            }
            for lane in (layout.forward_lane, layout.return_lane)
        },
        "geofence": [list(corner) for corner in layout.geofence_xy_m],
    }


def _write_textures(layout: FieldLayout, params: WorldParams, directory: Path) -> None:
    """Render every texture the world references."""
    ground = params.ground
    write_png(
        directory / GROUND_TEXTURE,
        make_ground_png(
            ground.texture_px,
            ground.texture_px,
            ground.base_rgb,
            ground.noise_strength,
            layout.seed,
        ),
    )
    zone = params.delivery_zone
    zone_height_px = max(1, round(zone.texture_px * zone.size_y_m / zone.size_x_m))
    write_png(
        directory / ZONE_TEXTURE,
        make_ground_png(
            zone.texture_px, zone_height_px, zone.base_rgb, zone.noise_strength, layout.seed + 1
        ),
    )
    banner = params.banner
    banner_height_px = max(1, round(banner.texture_px * banner.height_m / banner.width_m))
    write_png(
        directory / BANNER_TEXTURE,
        make_banner_png(banner.text, banner.texture_px, banner_height_px, layout.banner_rgb),
    )
    qr_settings = params.start_qr
    for marker in layout.all_qrs:
        write_png(
            directory / marker.texture,
            make_qr_png(marker.payload, qr_settings.texture_px, qr_settings.quiet_zone_modules),
        )


def generate_world(
    seed: int, out_dir: Path | None = None, params: WorldParams | None = None
) -> GeneratedWorld:
    """Generate every artefact for one seed.

    Args:
        seed: Layout seed.
        out_dir: Parent directory for `<seed>/`; defaults to `sim/worlds`.
        params: Pre-loaded parameters; loaded from the default path if None.

    Returns:
        Paths of the generated files.

    Raises:
        ConfigError: Parameters are invalid or a file could not be written.
    """
    world_params = load_world_params() if params is None else params
    layout = build_field(world_params, seed)
    directory = (DEFAULT_WORLDS_DIR if out_dir is None else out_dir) / str(seed)
    directory.mkdir(parents=True, exist_ok=True)

    _write_textures(layout, world_params, directory)

    world_path = directory / WORLD_FILENAME
    world_path.write_text(to_xml(build_world(layout, world_params, GROUND_TEXTURE)), "utf-8")

    field_config_path = directory / FIELD_CONFIG_FILENAME
    field_config_path.write_text(
        yaml.safe_dump(build_field_config(layout, world_params), sort_keys=False), "utf-8"
    )

    truth_path = directory / TRUTH_FILENAME
    truth_path.write_text(json.dumps(build_truth(layout, world_params), indent=2), "utf-8")

    logger.info(
        "world_generated seed=%d dir=%s qrs=%d obstacles=%d red_zones=%d target=%s",
        seed,
        directory,
        len(layout.delivery_qrs),
        len(layout.obstacles),
        len(layout.red_zones),
        layout.target_payload,
    )
    return GeneratedWorld(
        seed=seed,
        world_name=f"skyscan_{seed}",
        directory=directory,
        world_path=world_path,
        field_config_path=field_config_path,
        truth_path=truth_path,
    )

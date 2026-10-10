#!/usr/bin/env python3
"""Part 1 acceptance capture: generate a world, spawn the vehicle, save sensor data.

Starts the gz server on a generated world, spawns `aero_x500`, teleports it to
the start-QR observation altitude (autonomous flight needs offboard control,
which arrives in Part 2), then saves one frame from each camera and one reading
from each rangefinder, and decodes the start QR.

Usage:
    python scripts/capture_part1.py --seed 1
    python scripts/capture_part1.py --seed 2 --alt-m 5 --out-dir artifacts/part1

Exit code 0 only if both cameras produced a frame, both rangefinders produced a
reading, and the decoded start QR matches the seed's target payload.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from aero.core.errors import AeroError  # noqa: E402 - needs REPO_ROOT on sys.path
from aero.sim.capture import SensorRig, decode_qr_payloads, observe_pose, save_frame  # noqa: E402
from aero.sim.gz_server import GzServer, GzServerConfig  # noqa: E402
from aero.sim.gz_truth import ModelPoseWatcher  # noqa: E402
from aero.sim.make_model import DEFAULT_MODELS_DIR, generate_model, topics  # noqa: E402
from aero.sim.make_world import generate_world  # noqa: E402
from aero.sim.world_params import load_world_params  # noqa: E402

logger = logging.getLogger("capture_part1")

DEFAULT_OUT_DIR = REPO_ROOT / "artifacts" / "part1"
DEFAULT_LOG_DIR = REPO_ROOT / "logs"
# Observation altitude for the start-QR capture: the mission scans the QR from
# 5 m, which is also where the 66 deg down camera covers ~6.5 m of ground.
DEFAULT_CAPTURE_ALT_M = 5.0
SENSOR_STREAM_TIMEOUT_S = 90.0


def capture(args: argparse.Namespace) -> dict[str, Any]:
    """Run the capture and return a summary dictionary.

    Args:
        args: Parsed command line.

    Returns:
        Summary of what was captured, written to `summary.json`.

    Raises:
        AeroError: World generation, the simulator, or a sensor failed.
    """
    params = load_world_params(args.params) if args.params else load_world_params()
    world = generate_world(args.seed, args.worlds_dir, params)
    model_path = generate_model(DEFAULT_MODELS_DIR, params)
    sensor_topics = topics(params.vehicle.model_name)
    model_instance = f"{params.vehicle.model_name}_0"

    run_stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    log_dir = args.log_dir / f"{run_stamp}_part1_seed{args.seed}"
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    server_config = GzServerConfig(
        world_path=world.world_path, extra_resource_paths=(DEFAULT_MODELS_DIR,)
    )
    summary: dict[str, Any] = {
        "seed": args.seed,
        "world": str(world.world_path),
        "world_name": world.world_name,
        "model": str(model_path),
        "target_payload": None,
        "capture_pose_enu_m": None,
        "files": {},
    }

    with GzServer(server_config, log_dir / "gz.log") as server:
        server.spawn_model(
            model_instance,
            model_path,
            (0.0, 0.0, params.vehicle.spawn_z_m),
            math.radians(params.vehicle.spawn_yaw_deg),
        )
        rig = SensorRig(sensor_topics)
        pose_watcher = ModelPoseWatcher(server.world_name, model_instance)
        try:
            # Hover over the start QR, which sits `offset_x_m` ahead of home.
            capture_xyz = (params.start_qr.offset_x_m, 0.0, args.alt_m)
            snapshot = observe_pose(
                server,
                model_instance,
                capture_xyz,
                yaw_rad=math.radians(params.vehicle.spawn_yaw_deg),
                rig=rig,
                pose_watcher=pose_watcher,
                timeout_s=SENSOR_STREAM_TIMEOUT_S,
            )
            payloads = decode_qr_payloads(snapshot.down_frame)
            down_path = out_dir / f"seed{args.seed}_down.png"
            front_path = out_dir / f"seed{args.seed}_front.png"
            save_frame(down_path, snapshot.down_frame)
            save_frame(front_path, snapshot.front_frame)
            summary.update(
                {
                    "capture_pose_enu_m": list(capture_xyz),
                    "down_frame": {
                        "width_px": snapshot.down_frame.width_px,
                        "height_px": snapshot.down_frame.height_px,
                        "qr_payloads": payloads,
                    },
                    "front_frame": {
                        "width_px": snapshot.front_frame.width_px,
                        "height_px": snapshot.front_frame.height_px,
                    },
                    "ranges": {
                        "front": {
                            "distance_m": round(snapshot.front_range.distance_m, 3),
                            "valid": snapshot.front_range.valid,
                        },
                        "down": {
                            "distance_m": round(snapshot.down_range.distance_m, 3),
                            "valid": snapshot.down_range.valid,
                        },
                    },
                    "files": {
                        "down_frame": str(down_path),
                        "front_frame": str(front_path),
                        "gz_log": str(server.log_path),
                    },
                }
            )
        finally:
            rig.close()
            pose_watcher.close()

    field = json.loads((world.directory / "truth.json").read_text(encoding="utf-8"))
    summary["target_payload"] = field["target_payload"]
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=1, help="world seed")
    parser.add_argument(
        "--alt-m", type=float, default=DEFAULT_CAPTURE_ALT_M, help="capture altitude"
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="artifact directory")
    parser.add_argument(
        "--worlds-dir",
        type=Path,
        default=None,
        help="where to generate <seed>/ (default sim/worlds)",
    )
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR, help="run log directory")
    parser.add_argument("--params", type=Path, default=None, help="world params YAML")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Capture Part 1 sensor data and report whether it is usable."""
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s %(message)s", stream=sys.stderr
    )
    try:
        summary = capture(args)
    except AeroError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    summary_path = args.out_dir / f"seed{args.seed}_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"summary: {summary_path}")

    payloads = summary["down_frame"]["qr_payloads"]
    target = summary["target_payload"]
    if target not in payloads:
        print(
            f"FAILED: start QR {target!r} not decoded from the down frame (got {payloads})",
            file=sys.stderr,
        )
        return 1
    if not summary["ranges"]["down"]["valid"]:
        print("FAILED: down rangefinder reported no return", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

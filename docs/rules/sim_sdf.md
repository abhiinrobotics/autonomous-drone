# Simulation rules: Gazebo (gz sim) world, SDF, vehicle, judge

Geometry below is **inferred from rulebook Fig. 3 / §4.2.4** (layout "subject to change on event day"). Every number is a parameter in `sim/world_params.yaml`; the generator randomises within ranges per seed. Record deviations in STATUS.md.

## Principles
- SDF is **generated** by `sim/make_world.py --seed N` (Python, `xml.etree` or templates), never hand-edited. Output: `sim/worlds/<seed>/world.sdf`, `textures/*.png`, `mission_config_field.yaml` (GPS/NED of every feature, geofence, red polygons, target ID), `truth.json` (for judge).
- Validate every output with `gz sdf -k world.sdf` (generator test + CI step). Fix warnings, not just errors.
- World plugin block (physics, user-commands, scene-broadcaster, contact, imu, air-pressure, navsat, magnetometer, sensors with `ogre2`) is **copied from the installed PX4** `Tools/simulation/gz/worlds/default.sdf`, not hand-typed. Keep `<spherical_coordinates>` (set a real lat/lon, elevation 0) so GPS works.
- Static geometry: `<static>true</static>`, each wall = link with `<collision>` + `<visual>` boxes. Textures via PBR `albedo_map` (relative `textures/..png`) on thin boxes (1x1x0.01 m). Generate QR PNGs with a >=4-module quiet zone, nearest-neighbour upscale, 1024 px, no anti-aliasing blur.
- Prefer not to fork the PX4 tree. Put models under `sim/models` (`GZ_SIM_RESOURCE_PATH`). If a PX4 airframe entry is unavoidable, install it via `scripts/install_airframe.sh` and document the PX4 version tested. Consult the installed PX4 docs (use Context7 if available) for "custom vehicle / standalone gz" for that exact version.

## Frames (do not assume — test it)
gz world is ENU (x east, y north, z up); PX4 local is NED (x north, y east, z down). Mission "forward" = +x_enu. Part 1 must include a test that spawns the drone, reads gz pose and PX4 `LOCAL_POSITION_NED`/yaw, and asserts the mapping; encode it once in `core/geo.py` (`enu_to_ned`, `gps_from_enu`). Corridor axis, spawn yaw, and camera mounts must be consistent with that test.

## Field layout (defaults, all in world_params.yaml)
- Origin (0,0) = takeoff point **home** (blue takeoff zone 3 x 7 m, centred on home, x in [-1.5, 1.5]).
- Start QR: 1x1 m on ground at x = +1.0 m from home (drone climbs to 5 m then moves ~1 m forward, so it is overhead). Payload = target ID (one of the delivery QRs).
- Two straight open-top lanes along +x, each **3.5 m wide x 10 m long**, walls **3.05 m (10 ft)** high, 0.1 m thick, neutral grey. Forward lane centred on y=0 (straight ahead of home), starts at x = 3.0. Return lane directly beside it (shared wall, centre y = -(3.5+0.1)), same x extent. Outer walls at both sides. Parameters: `lane_width_m, lane_length_m, wall_h_m, lane_gap_m, lane_start_x_m`.
- Green banner at each lane entrance (forward: facing -x; return: facing +x on the zone side): board **2.0 x 1.0 m** (param) with green background and "AeroTHON 2026" text, bottom edge just above wall height (3.2 m) so it never blocks the lane; green hue/saturation jittered per seed within the detector’s HSV range +/- margin to avoid overfitting.
- **Return lane only** has obstacles: 2-4 black boxes, full wall height, width 0.6-1.2 m, alternating sides, >=2.5 m longitudinal spacing, always leaving >=2.0 m free lateral gap. Forward lane is clear.
- Delivery zone: **40 x 30 m** ground rectangle starting 2 m beyond the lane end, centred on the lane axes, light-green textured ground (so a naive green detector fails on grass). 5-6 unique 1x1 m QR targets (payloads `DELIVERY_A`..), spacing >=5 m, one equals the start QR. 1-2 red rectangles (default 4x4 m) not overlapping QRs (>=3 m clearance). All inside the geofence with >=3 m margin.
- Ground plane 200x200 m, green-ish noise texture. Sun with shadows on (shadows/glare stress CLAHE).
- Geofence polygon = bounding region of takeoff zone + lanes + zone + 5 m margin (written to field yaml; used by FlightLink software fence).

## Vehicle `aero_x500`
- Based on PX4 `x500` (`<include>` it; add sensor links with fixed joints). Mounts (config): down camera pointing down, image-up = body forward; front camera level forward.
- Down camera 1280x720 @ 15-30 Hz, HFOV default 66 deg (placeholder, real FOV unknown — config). Front camera 1280x800 @ 30 Hz, HFOV default 90 deg (placeholder). Verify actual FOV from SDF in tests; `camera_model.py` reads the same value from config.
- Front range: gz lidar emulating a TFmini Plus: ~3.6 deg beam (e.g. 5x5 rays over +/-1.8 deg, take min), 0.1-12 m, 100 Hz. Down range: TF-Luna-like, 0.2-8 m, 100 Hz (clipped at 8 m on purpose). Optional optical flow is NOT required; EKF uses GPS in SITL.
- Add small Gaussian noise to ranges and cameras (config) so tests are not unrealistically clean.
- Sensor data reaches mission code via `hal/sitl/*` (gz-transport python bindings), mirroring how the Pi reads sensors on hardware. Payload/hook is **logical** (SimWinch), no hook physics.
- Headless run for CI/batch: `gz sim -s -r --headless-rendering <world>` (cameras need the rendering server). GUI only for debugging.

## Judge (`sim/judge.py`)
- Reads gz ground-truth poses (drone, via gz topic) + `truth.json`. No dependence on the drone’s own estimate.
- Collision = drone body (radius `drone_radius_m`, default 0.35) intersects wall/obstacle AABBs or ground impact above speed threshold. Red-zone violation = drone ground projection inside a red polygon (count entries, -5 each). Delivery error = horizontal distance between release point (payload ground contact) and target QR centre. Timing from first arm/takeoff. Scores follow rulebook §5.6 weights; partial credit formulas documented in code and in STATUS.md.

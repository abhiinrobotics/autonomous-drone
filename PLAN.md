# AeroTHON 2026 — Mission 2 (SkyScan) Autonomous Stack — SITL-first Plan

> Read `CLAUDE.md` first. Rules: `docs/rules/*`. Specs: `docs/spec/*`. World/SDF geometry: `docs/rules/sim_sdf.md`.

Execute **one part at a time**. Do not start part N+1 until part N's acceptance test passes. After each part: commit, update `docs/STATUS.md` (what works, known issues, how to run).

## 0. Context

- Drone: quad, Pixhawk 6C running **PX4**, Raspberry Pi 4B companion. Pi↔FC via MAVLink (pymavlink; **no MAVSDK, no ROS**).
- Goal now: **algorithms only, pure SITL** (PX4 SITL + Gazebo). Real hardware drivers are stubs for later.
- Same mission code must run on SITL and hardware. Only the hardware profile changes (`AERO_PROFILE=sitl|hw`).
- Reference: `docs/spec/rulebook_mission2.md` and `docs/spec/design_report_summary.md` (PDFs in `docs/reference/` — do not open).

## 1. Mission spec (from rulebook; all values in `mission_config.yaml`)

| Step | Requirement |
|---|---|
| Takeoff | Autonomous, from takeoff zone, to ~5 m |
| Start QR | Move forward ~1 m, scan QR (1 m x 1 m, **configurable**) with down cam, decode target ID |
| Corridor entry | Detect green banner ("AeroTHON 2026"), align, descend to 3 m (10 ft) |
| Corridor | 3.5 m wide, ~10 m long, 3 m high walls. Forward lane: clear. Avoid walls. |
| Exit | Climb to ~10 m |
| Delivery zone | ~40 m x 30 m, multiple QRs. At ~10 m find the QR matching the start QR. Avoid red zones (GPS polygons given in config; also detect visually) |
| Delivery | Descend to 5 m over target, lower payload by winch/pulley, release only after ground contact |
| Return | Climb to 10 m, fly back to **return lane** entry, banner, descend to 3 m, navigate lane **with static obstacles**, exit |
| Landing | Land at takeoff point (use home position; no marker allowed) |
| Limits | 15 min total. Geofence from config. Record flight data (CSV + video) |

Scoring (§5.6, judge must implement): takeoff+QR 10, corridor 15, target QR ID 10, red-zone 10 (−5/violation), delivery accuracy 15, return nav 20, landing 5, time 15.

## 2. Architecture rules (enforce in every part)

1. **Layers**: `hal/` (interfaces + drivers) → `core/` (flight link, state, math, logging) → `vision/` (pure functions, no I/O) → `phases/` → `mission/`.
2. **Mission code never touches hardware/sim directly.** It only uses interfaces: `FlightLink`, `Camera`, `RangeSensor`, `Winch`.
3. **One MAVLink reader thread** with a thread-safe latest-message cache. No other code calls `recv_match`.
4. **PX4 offboard needs a setpoint stream ≥ 20 Hz**, started before switching to offboard and never interrupted. A `SetpointStreamer` thread holds the current target; phases only update the target (`set_velocity_body`, `set_position_local`, `hold`). **No `time.sleep` control loops; no blocking vision inside the streamer.**
5. Phases are classes with `tick(ctx) -> PhaseStatus(RUNNING|DONE|FAILED)` driven by a fixed-rate loop. Timeouts per phase from config.
6. Altitude source: EKF local-position z above ~6 m; down rangefinder only below ~6 m (TF-Luna max ~8 m). Never wait on rangefinder ≥ 9.5 m.
7. Every tunable (gains, thresholds, altitudes, speeds, QR size, camera FOV/mount, winch offset px) lives in `mission_config.yaml` or `config/hardware.py`. No magic numbers in phase code.
8. Failsafe: any phase FAILED or exception → PX4 RTL (AUTO.RTL via DO_SET_MODE with PX4 custom mode), stop streamer cleanly. Never exit mid-air silently.
9. Vision functions are pure (`frame, params -> result`) and unit-tested on saved frames.
10. Sim camera is cleaner than real: sim `Camera` wrapper supports optional augmentation (blur, noise, gamma, brightness drift) from config, on by default in batch tests.
11. Python 3.10+, type hints, `pytest`. Integration tests marked `@pytest.mark.sitl`.

## 3. Repo layout

```
aero/
  config/
    hardware.py          # PROFILES = {"sitl": {...}, "hw": {...}}  (device paths, drivers, camera res, FC link)
    mission_config.yaml  # mission params + field geometry (GPS)
  hal/
    interfaces.py        # ABCs: FlightLink-agnostic sensors: Camera, RangeSensor, Winch
    sitl/                # gz camera, gz range, sim winch
    hw/                  # picamera2, usb cam, tfmini, tfluna, pigpio winch  (STUBS: raise NotImplementedError)
    factory.py           # build_hal(profile)
  core/
    flight_link.py       # PX4 pymavlink: reader thread, cache, streamer, arm/offboard/takeoff/land/rtl
    geo.py               # NED<->GPS, haversine, point-in-polygon, segment intersect w/ buffer
    camera_model.py      # intrinsics from FOV/res, pixel->ground projection (alt, attitude, mount)
    logging_.py          # CSV telemetry + JSON events + video writer
    loop.py              # fixed-rate loop helper
  vision/
    qr.py banner.py corridor.py obstacles.py redzone.py
  phases/
    p01_takeoff_qr.py p02_corridor_entry.py p03_corridor_nav.py p04_exit_climb.py
    p05_zone_search.py p06_precision_lock.py p07_winch_release.py
    p08_return_transit.py p09_return_entry.py p10_return_nav.py p11_landing.py
  mission/
    runner.py            # FSM, timers (15 min), failsafe
  sim/
    models/aero_x500/    # PX4 gz model + front cam, down cam, front range, down range
    worlds/              # generated
    make_world.py        # --seed N -> world.sdf + mission_config_field.yaml (GPS coords of everything)
    judge.py             # ground-truth scoring per §5.6
    run_sitl.sh          # starts PX4 SITL + gz with the world
  tests/
  docs/STATUS.md
```

## 4. Parts

### Part 0 — Environment bring-up
- Detect/install: PX4-Autopilot (SITL), Gazebo (gz sim), `pymavlink opencv-python pyzbar numpy pyyaml qrcode pytest`, `libzbar0`, gz python bindings.
- `scripts/check_env.py` prints versions and runs a smoke test: start PX4 SITL (`gz_x500`), connect with pymavlink, receive heartbeat.
- **Accept**: heartbeat received, versions logged to `docs/STATUS.md`. **Ask the user** if PX4/gz versions are unexpected.

### Part 1 — Sim vehicle, sensors, world
- `aero_x500` model: down camera (config FOV/res), front camera, front 1-beam range (≈3.6° beam), down 1-beam range (max 8 m), optional flow. Sensor topics documented.
- Follow `docs/rules/sim_sdf.md` exactly (geometry, plugins, validation with `gz sdf -k`). `make_world.py --seed N`: ground plane (grass-like texture), takeoff zone + start QR (1 m), two side-by-side 3.5 m x 10 m lanes with 3 m walls, green banner at each lane entrance, **return lane (second lane) with 2–4 black obstacle blocks** at seeded positions, delivery zone 40 x 30 m with 5–6 QR textures (generated, 1 m) incl. target, 1–2 red zone rectangles, home at takeoff zone. Writes `mission_config_field.yaml` with GPS of lane entries/exits, zone corners, red polygons, geofence. Layout must vary with seed.
- `hal/sitl`: `Camera.read()`, `RangeSensor.read()` from gz topics.
- **Accept**: script saves one frame from each camera and ranges to `artifacts/part1/`; QR in start frame is decodable with pyzbar; world differs between seeds 1 and 2.

### Part 2 — Config + FlightLink
- `config/hardware.py` + `factory.py` as in layout. FC link string (`udp:127.0.0.1:14540` for sitl; serial path for hw).
- `FlightLink`: reader thread + cache (`get(msg_type)`, `age()`), `SetpointStreamer`, `arm`, `offboard`, `takeoff(alt)`, `set_position_local`, `set_velocity_body(vx,vy,vz,yaw_rate)`, `hold()`, `climb_to(alt)` (position-z with xy hold), `land()`, `rtl()`, `pos_local()`, `pos_gps()`, `alt_agl()` (altitude policy §2.6), `yaw()`, `set_geofence`. Request message rates at connect.
- **Accept** (`@sitl`): takeoff to 5 m, move 1 m forward, hold 5 s, climb to 10 m, descend to 3 m with xy drift < 0.3 m, goto GPS point within 1 m, land and disarm. A deliberate 3 s vision stall must NOT trigger offboard loss.

### Part 3 — Vision library on captured data
- `scripts/capture_dataset.py` flies/places the drone, saves frames from both cameras at varied altitudes/seeds into `tests/data/`.
- Implement + unit test: `qr.decode_all(frame)` (CLAHE+sharpen, returns data+polygon, multi-QR), `banner.detect(frame)` (HSV green + shape/aspect/size gating, ROI above horizon; must reject grass), `corridor.estimate(frame)` (returns lateral error px and heading error; use wall x-intercepts at fixed image rows / vanishing point, **not** slope-sign classification; single-wall fallback; returns confidence), `obstacles.detect(frame)` (black blocks in lane: bearing + rough range), `redzone.detect(frame)` (red hue wraps 0/180; returns pixel polygons).
- `camera_model.py`: pixel→ground NED/GPS using altitude, roll/pitch/yaw, mount rotation, FOV.
- **Accept**: pytest passes on dataset: QR decode ≥ 95 % at 10 m / 1 m QR, banner no false positives on grass frames, corridor error sign correct in left/right-offset frames, projection of a known QR within 1 m of ground truth.

### Part 4 — Takeoff, start QR, corridor entry, forward corridor
- p01: takeoff 5 m, forward ~1 m, 3-frame QR confirmation, store target (config QR size used for sanity check).
- p02: search/yaw-align banner, closed-loop approach, descend to 3 m.
- p03: forward cruise 0.5 m/s with PID on corridor error (lateral + yaw trim), end condition = distance travelled along lane ≥ length + margin (EKF local pos), not a timer.
- **Accept** (`@sitl`, seeds 1–10): drone exits forward lane without touching walls (judge), ≥ 9/10 seeds.

### Part 5 — Exit, zone search, QR match, red zones
- p04: brake, pure climb to 10 m.
- p05: lawnmower coverage of zone computed from camera footprint at 10 m (width from FOV) with overlap; **continuous scan while moving at ≤2 m/s** (exposure short); decode all QRs, geotag each via `camera_model`, dedupe, match target. Red zones: config polygons + visual detection added to a runtime no-fly set with buffer; path planner skips/detours segments crossing them (segment-polygon test with buffer). Early exit when target found and confirmed twice.
- **Accept** (seeds 1–10): target found ≥ 9/10, 0 red-zone entries, geotag error < 1.5 m, time budget logged.

### Part 6 — Precision lock + winch (sim)
- p06: fly to target geotag, descend to 5 m while visually servoing; error converted to metres (alt-scaled), target pixel = winch-line pixel from config (`winch_offset_px`), deadband + hold time, abort if QR lost > N s (re-climb and re-acquire).
- p07: `SimWinch` with simulated Hall pulses; lower to `ground_dist + slack`; release only on ground contact (range-based); jam detection (no pulses); retract; confirm released flag.
- **Accept**: final horizontal error < 0.5 m (judge) in ≥ 9/10 seeds; no release before ground contact; log shows pulse-based length.

### Part 7 — Return leg
- p08: climb to 10 m, fly (GPS, red-zone-safe path) to a staging point 6–8 m in front of return lane entry (from config).
- p09: find banner, align, descend to 3 m.
- p10: lane nav (same corridor estimator) + obstacle avoidance: front range + camera obstacle detector; lateral bypass within lane (3.5 m wide, keep wall margin ≥ 0.6 m), then re-centre; end by distance travelled.
- **Accept** (seeds 1–10): clears 2–4 obstacles and exits with no collision ≥ 9/10.

### Part 8 — Landing, runner, failsafes, logging
- p11: fly to home (GPS), descend, PX4 land; confirm disarm; landing distance from home logged.
- `runner.py`: FSM over phases, 15-min clock, per-phase timeouts, geofence, failsafe-to-RTL, structured logs (CSV telemetry, JSON events per phase, MP4 of both cameras).
- **Accept**: full mission end-to-end on seeds 1–5 under 15 min, logs written, forced failure in each phase triggers RTL cleanly.

### Part 9 — Judge + batch tuning
- `judge.py` reads gz ground truth and scores per §5.6 weights.
- `scripts/batch.py --seeds 1-30 --augment` → table of score per seed, failure causes. Tune gains in config, not code.
- **Accept**: mean score and failure histogram committed in `docs/STATUS.md`.

## 5. Hardware handoff (later, not now)
Fill `hal/hw/*` drivers (picamera2 down cam, USB front cam by `/dev/v4l/by-id`, TFmini/TF-Luna serial or via FC `DISTANCE_SENSOR`, pigpio winch + Hall GPIO, Matek MSP→FC bridge). Add udev rules. Re-tune camera mount, FOV, exposure/focus, gains. No mission code changes expected.

## 6. Open items for the user (Claude Code: ask when reached)
- Exact camera FOV and mount orientation (Part 1/3 use config placeholders).
- Final field geometry/GPS (Part 5/7 read from config at the venue).
- Winch offset relative to down-cam centre.
- PX4 version on the 6C (only matters for hardware handoff).

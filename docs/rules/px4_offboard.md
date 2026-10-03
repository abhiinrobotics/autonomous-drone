# PX4 + pymavlink rules (verify against the installed PX4 version; decide via heartbeat/params, record in STATUS.md)

## Connections
- SITL: companion/offboard MAVLink port `udpin:0.0.0.0:14540`. GCS (QGC) is 14550.
- Hardware: Pixhawk 6C over USB -> `/dev/ttyACM0` (use a udev symlink `/dev/fc`). Path/baud come from `config/hardware.py`.

## Architecture
- `FlightLink` owns the `mavutil` connection. **Reader thread** is the only caller of `recv_match`; it stores the latest message per type with receive timestamp. Public API: `get(msg_name) -> (msg, age_s)`; typed snapshot helpers (`pos_local()`, `pos_gps()`, `yaw_rad()`, `armed()`, `landed_state()`, `battery()`).
- **SetpointStreamer** thread publishes the current target at >=20 Hz continuously (also while holding). Phases call `set_velocity_ned/ set_position_local / hold()` which only swap the target atomically. Streaming must start (and run ~1 s) BEFORE requesting OFFBOARD and never stop until land/RTL is engaged. If no target: hold current position.
- Request rates after connect with `MAV_CMD_SET_MESSAGE_INTERVAL`: LOCAL_POSITION_NED(32), GLOBAL_POSITION_INT(33), ATTITUDE(30), EXTENDED_SYS_STATE(245), DISTANCE_SENSOR(132), BATTERY_STATUS(147), HOME_POSITION(242). STATUSTEXT is always collected into a ring buffer for logs.

## Modes (PX4 custom modes via MAV_CMD_DO_SET_MODE: param1=1, param2=main, param3=sub)
- OFFBOARD: main=6, sub=0. AUTO.RTL: main=4, sub=5. AUTO.LAND: main=4, sub=6. AUTO.LOITER: main=4, sub=3. Confirm by decoding `HEARTBEAT.custom_mode` (main = bits 16-23, sub = bits 24-31). Do not use ArduPilot/ArduCopter mode numbers (GUIDED=4, LAND=9 ...); they are wrong for PX4.
- Closed loop: after every mode/arm command, wait for the heartbeat/ACK with timeout and raise on failure.
- Sequence: stream hold setpoint -> arm (verify `armed`) -> request OFFBOARD (verify) -> climb via position setpoint.

## Setpoints (SET_POSITION_TARGET_LOCAL_NED, frame MAV_FRAME_LOCAL_NED)
- `type_mask`: a SET bit means IGNORE that field. Define named flags once in `core/mavlink_masks.py` and compose; never hand-write binary literals in phases.
  - velocity + yaw_rate: ignore pos(0-2), accel(6-8), yaw(10) -> 0x05C7. position + yaw: ignore vel(3-5), accel(6-8), yaw_rate(11) -> 0x09F8. position only: add yaw(10) -> 0x0DF8.
- Prefer LOCAL_NED everywhere. For body-frame commands compute the rotation from current yaw in `core/geo.py` (avoid relying on BODY_NED semantics).
- Hold altitude while moving laterally: try a mixed mask (z position + xy velocity + yaw rate); PX4 maps ignored axes to NaN and normally supports this — verify in SITL in Part 2 and record the result. Fallback: velocity-only with a clamped P-loop on z (vz = Kp*(z_target - z)). Climb/descend with position setpoints holding current x,y,yaw.
- Clamp all outputs (max speed, max vertical speed, max yaw rate) from config.

## Altitude policy
- `z_ned` from LOCAL_POSITION_NED is the primary altitude source (AGL = -z, origin at arming spot, flat field assumed). Down rangefinder is valid only below its max range (TF-Luna ~8 m) and only for fine tracking (<~6 m: takeoff, 3 m corridor, 5 m delivery, landing). Never wait on a rangefinder reading above its range.

## Landing / failsafe
- Land: AUTO.LAND, confirm disarm via heartbeat. RTL on any failure. Implement an authoritative **software geofence** in FlightLink (polygon from config) that refuses/clamps targets and triggers RTL on breach; optionally mirror to PX4 geofence params.
- Link-loss: if reader sees no heartbeat > N s -> mission aborts (RTL is handled by PX4 failsafe params `NAV_DLL_ACT`, `COM_OF_LOSS_T`; set sane values in SITL param file).

## Do not
Send one-shot setpoints with sleeps; arm without a live setpoint stream; assume mode changes succeeded; use `time.time()` for timeouts; call `recv_match` outside the reader.

# Design pitfalls to avoid (checked against this project's hardware and PX4)

1. **MAVLink reading**: filtering `recv_match(type=...)` on a shared connection discards other messages; multiple readers race. One reader thread + cache only. Request message rates explicitly.
2. **PX4 offboard**: one-shot setpoints followed by sleeps/blocking vision drop offboard mode. Stream continuously (>=20 Hz) in its own thread.
3. **Autopilot dialects**: ArduPilot mode numbers/messages do not apply. Use PX4 custom modes; decode heartbeat to verify.
4. **Rangefinder range**: TF-Luna maxes at ~8 m. Never wait for rangefinder values at 10 m; use EKF altitude above ~6 m.
5. **Optical flow**: autopilots do not emit `OPTICAL_FLOW_RAD`; the EKF already fuses flow. Do not add manual drift compensation on top of velocity setpoints.
6. **Corridor wall lines**: each wall shows two edges with opposite image slope, so slope-sign classification is wrong. Group by position and intersect with a fixed image row; the mean of segment midpoints is perspective-biased.
7. **Single-beam front range (3.6 deg)** only sees dead ahead; it cannot find side obstacles. Camera gives direction, range gives distance; sides rely on the camera.
8. **Down-camera control**: convert pixel error to metres using altitude and intrinsics (never fixed px->velocity gains); centre on the winch-line pixel, not the frame centre; the camera-to-body mapping is config, not an assumption.
9. **Delivery rule**: descend to 5 m before lowering the payload; release only after ground contact is confirmed.
10. **Winch**: hobby servo duty cycles differ (7.5 % is neutral for continuous-rotation servos); calibrate and keep them in config. Confirm payload release (do not assume a timer = released). Use a GPIO abstraction (pigpio/gpiozero), not raw RPi.GPIO calls scattered in phases.
11. **Mission glue that is easy to forget**: 1 m forward move after takeoff; flying from the delivery zone back to the return-lane entrance; corridor end by distance travelled (not a timer); red-zone handling both from given GPS polygons and visual detection; flight-data logging for the jury.
12. **Camera resolution**: default 640x480 is far too low to decode QR codes at altitude; set resolution explicitly, lock focus (IMX708 autofocus), short exposure. USB cameras: bind by `/dev/v4l/by-id`, never by index.
13. **Green detection** fails on grass unless gated by shape, size and an ROI above the horizon.
14. **Time budget**: estimated endurance (~8 min) is shorter than the 15-min window; every phase needs a time budget and the mission clock must be logged.

# Design report summary (team Vayuratha, drone "Dhruv") — facts relevant to software

## Platform
True-X quad, ~400 mm wheelbase, 7x4x3 tri-blade props, EMAX ECO II 2207 1700KV, 4-in-1 ESC (DShot), 4S 5200 mAh 25C LiPo. Estimated AUW ~1.94 kg (report), CG X-Y centred, payload underslung. Estimated endurance ~8.3 min (80 % usable) -> **mission time budget matters** (15-min window > battery!). Low-battery: warn 14.0 V, RTL 13.6 V (4S).
Design cruise in corridor 0.5 m/s. Planned mission 11 waypoints, takeoff alt 10 m AGL in report (rulebook says 5 m scan first).

## Avionics (CURRENT truth, differs from report where noted)
- FC: **Pixhawk 6C, PX4** (report says Pixhawk 2.4.8 — changed). Version unknown yet.
- Companion: **Raspberry Pi 4B 8 GB** (report inconsistently says Pi 5). Pi<->FC: USB (`/dev/ttyACM0`).
- Cameras: **Arducam IMX708 12MP (CSI) facing down**; **OV9782 global-shutter colour USB camera (front)**. FOV unknown (config).
- Rangefinders: Benewake **TF-Luna down** (~0.2-8 m), **TFmini Plus front** (3.6 deg beam, 12 m) — via USB-UART or I2C on the Pi, may later move to FC.
- Optical flow: Matek 3901-L0X (MSP over UART) on Pi UART. GPS: Holybro M10. Telemetry: SiK 433 MHz. RC: RadioMaster TX12 + ELRS RP1.
- Winch: MG90 servo (rotation handled), spool circumference ~0.08 m, **Hall sensor on Pi GPIO**, ~4 pulses/rev (~20 mm/pulse, verify). Payload hook: torsion-spring/counterweight gravity hook that opens when line tension drops on ground contact.
- Note for later hardware handoff: all sensors on the Pi means FC/EKF will not see them unless bridged; plan to move range/flow to FC if PX4 drivers allow.

## Software facts from report
- Mission states: init -> takeoff -> QR -> corridor entry (HSV banner) -> corridor (Canny+Hough) -> exit/climb -> zone search -> target match -> precision lock (15 px) -> winch (Hall pulse count, anti-jam 2 s) -> release -> return -> land.
- QR: pyzbar, 3-frame confirmation, dict {qr: (lat, lon, t)} inventory with dedupe, CSV log.
- Dual computing: Pixhawk = stabilisation/safety, Pi = vision + mission logic.
- Safety: RTL on low battery/datalink loss, geofence (3.5 x 10 m corridor box in report; real geofence given at Phase 2), arming checks, emergency landing, mission-resume idea.

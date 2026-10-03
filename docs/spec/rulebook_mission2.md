# AeroTHON 2026 Track 1 — Condensed rules (Rev 2, March 2026). Source of truth: docs/reference PDF (do not open).

## UAS constraints
Multirotor only. Micro UAS, MTOW 2 kg (take-off < 2 kg), electric, payload 100 g, comms range >= 1 km. Payload box 10 x 5 x 5 cm with a top eyelet (hook attach). Innovation expected in payload drop mechanism.

## Phase 2 inspection (50 pts) — relevant to us
Dimensions vs submitted 2D drawing (10); same components as report: motor/ESC/prop, battery, **flight controller / radio TX / RX** (6); take-off weight within 50 g of report = 10 (50-100 g: 8, 100-200: 5, >200: 0); structural integrity (10); static checks (8); fail-safes configured (4): RTL on low battery, RTL on datalink loss, geofence (2); aesthetics (2). Any deviation from report needs a written Change Request (Appendix B), one change per CR, penalty possible. **FC was changed (Pixhawk 2.4.8 -> 6C/PX4): file a CR.**

## Mission 2 — SkyScan (fully autonomous, 100 pts)
Sequence:
1. Autonomous takeoff from ground start point. Ascend to **5 m**, move forward ~**1 m**, scan the start **QR** (delivery location info). [10 pts]
2. Identify the **AeroTHON 2026 banner with green background** at the corridor entrance, align, descend to **10 ft (~3 m)**, navigate the **3.5 m wide corridor** autonomously avoiding walls/obstacles. [15]
3. Upon reaching the delivery zone ascend to **~10 m**, identify the QR matching the start QR among multiple QRs in the zone. [10]
4. Avoid **red/restricted zones** in the delivery area. Each violation **-5**. [10]
5. Descend to **5 m**, lower the payload with a **controlled pulley/winch** and release **only after reaching the ground** at the correct QR. [15]
6. Ascend to **10 m**, return through the corridor (detect banner again for return lap), avoid obstacles. [20]
7. Land safely at the take-off point **without manual intervention**. [5]
8. Complete within the **15-minute** window; marks proportional to how quickly. [15]
Notes: Delivery-zone **geofence coordinates are given in Phase 2** and must be programmed into ground-station software. UAS must **record flight data** and share it with the jury afterwards. Layout/field "subject to change on event day". Zone about 40 x 30 m; corridor ~10 m long, 3.5 m wide, 10 ft high (Fig. 3). Return lap lane has static obstacles (Fig. 3). 5-minute takeoff-start penalty; 15-min slot starts at throttle-up; relaunch allowed only if first flight < 2 min with takeoff failure/immediate landing/crash.
Scoring is "preliminary; final criteria shared after Phase 1".

## Mission 1 (manual, 100 pts) — not our current scope
Obstacle course, red zones (-10), observation zones with verbal report, payload placement, return. Manual control only.

## Phase 1 scoring (done, selected for Phase 2)
Design report 50 + presentation 50.

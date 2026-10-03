# AeroTHON 2026 — Mission 2 "SkyScan" autonomous stack

Quadcopter (Pixhawk 6C, **PX4**) + Raspberry Pi 4B companion. Python 3.10+, pymavlink (no MAVSDK, no ROS), OpenCV, pyzbar.
**Current focus: algorithms in pure SITL (PX4 + Gazebo gz).** Hardware drivers are stubs until later.

## Session start (always)
1. Read `docs/STATUS.md` (state + next step). 2. Read ONLY the current Part in `PLAN.md`.
Do not pre-read anything else. Never open PDFs (`docs/reference/`) — use `docs/spec/*.md`.

## Load on demand
| Working on | Read first |
|---|---|
| any code | `docs/rules/engineering_standards.md` |
| FlightLink, offboard, modes, setpoints | `docs/rules/px4_offboard.md` |
| camera / QR / banner / corridor / red zone | `docs/rules/vision.md` |
| Gazebo world, SDF, model, sensors, judge | `docs/rules/sim_sdf.md` |
| tests, SITL runs, batch | `docs/rules/testing.md` |
| mission requirements, scoring | `docs/spec/rulebook_mission2.md` |
| drone hardware facts, sensors, winch | `docs/spec/design_report_summary.md` |
| design choices that commonly go wrong | `docs/rules/pitfalls.md` |

## Non-negotiables
- Same mission code runs on SITL and hardware. Only `AERO_PROFILE=sitl|hw` (config/hardware.py) changes. Mission/phase code talks to interfaces, never to sim/hardware directly.
- One MAVLink reader thread + state cache. One setpoint streamer (>=20 Hz). No `recv_match` elsewhere. No sleep-based control loops.
- No magic numbers: every tunable in `mission_config.yaml` / `config/hardware.py`. QR size (1.0 m), camera FOV/mount, winch offset are config.
- Units in names (`_m`, `_mps`, `_rad`, `_px`). Frames documented. NED local, body FRD.
- Any failure -> PX4 RTL, never silent exit. No bare `except`. `logging`, not `print`.
- Vision = pure functions, unit-tested on saved frames.
- Everything is built from scratch in this repo. Do not touch `docs/reference/`.

## Workflow
- One Part at a time (`/part N`). Stop when its acceptance test passes. Then `/handoff`.
- Small commits (Conventional Commits). `make lint typecheck test` must pass before commit.
- Ask the user only for: FOV/mount values, field data, hardware facts not in docs. Otherwise decide, record the decision in `docs/STATUS.md`.

## Token discipline
- Use Grep/Glob and line ranges; don’t read whole big files or logs. Summarise logs via the `sitl-debugger` agent.
- Run tests quietly (`pytest -q -x --tb=short`); never paste full tracebacks/logs back, quote the relevant lines.
- Prefer targeted edits over rewriting files. Keep answers short; code and results over prose.

## Record keeping (mandatory)
- `docs/DECISIONS.md`: log every non-obvious decision/assumption as it is made (template inside). Never rewrite history; supersede.
- `docs/IMPLEMENTATION.md`: per Part, record files built, run/verify commands, and a status table. Mark VERIFIED only after running the command; paste the command + result summary. Any later change to a built Part -> add a line under "Changes after initial implementation" and re-verify or mark UNVERIFIED.
- `docs/STATUS.md` stays a short pointer (current Part, next command); details live in the two logs above.

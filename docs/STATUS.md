# STATUS (update at the end of every Part; keep < 80 lines)

## Current
- Part: 0 (DONE, acceptance test passing)
- Next command: `/part 1`

## Environment (measured 2026-10-10, `make check-env`)
- OS / Python: Ubuntu 22.04.5 (Linux 6.8.0-106-generic) / Python 3.10.12 in `.venv`
- PX4: `v1.18.0-alpha1-96-g1512a552f9` (commit `1512a552f9`), `~/PX4-Autopilot`, `px4_sitl_default` built
- Gazebo: gz-sim **8.15.0** (Harmonic) selected; 7.9.0 (Garden) also installed
- Packages: pymavlink 2.4.50, numpy 2.2.6, opencv-python 5.0.0.93, pyzbar 0.1.9 (libzbar OK), PyYAML 6.0.3, qrcode 8.2, pytest 9.1.1
- Hardware FC will run PX4 1.17.0 (planned) — see D-001, re-verify offboard on hardware
- Camera FOV (down/front), mount: still placeholders from `docs/rules/sim_sdf.md` (set in Part 1)

## Done
- Part 0 — environment bring-up (commit `fc3dbe2`): `scripts/check_env.py` (version report + SITL heartbeat smoke test), `aero/sim/px4_sitl.py` (PX4+gz process-group launcher), `aero/sim/mavlink_probe.py`, `aero/sim/env_report.py`, function-scoped pytest `sitl` fixture. Details in `docs/IMPLEMENTATION.md`.

## Decisions
See `docs/DECISIONS.md` D-001..D-011: PX4 1.18.0-alpha1 for SITL (hardware 1.17.0), gz Harmonic 8.15.0, Python SITL launcher instead of `run_sitl.sh`, `px4 -d` mandatory, make targets clear `PYTHONPATH`, refuse start on a stale gz world (and on an unreachable gz), numeric-prefix version comparison, heartbeat-based readiness, process group recorded at spawn, function-scoped `sitl` fixture.

## Known issues
- Running `pytest` directly in a ROS-sourced shell fails (ROS Humble pytest plugins + `PYTHONPATH`). Use `make test` / `make test-sitl` (D-005).
- A SITL run hard-killed with SIGKILL leaves PX4 + gz alive; `make sitl-clean` clears them (pgid is also saved as `px4.pgid` in the run dir).
- A stale gz world makes the next start refuse with a clear message, by design (D-006).
- SITL (1.18.0-alpha1) and planned hardware firmware (1.17.0) differ by about one release (D-001).

## Measured results
| Part | Test | Seeds | Result |
|---|---|---|---|
| 0 | `make test` (unit) | n/a | 54 passed, 2026-10-10 |
| 0 | `make test-sitl` (`tests/test_sitl_smoke.py`) | 0 | 1 passed in 13.8 s, heartbeat sysid=1, no leftover processes, 2026-10-10 |
| 0 | `make check-env` | n/a | all checks yes, heartbeat in 1.0 s after ready, 2026-10-10 |

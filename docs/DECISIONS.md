# DECISIONS (append-only log; never delete entries, supersede them)

Rule: log every non-obvious choice (architecture, parameters, tradeoffs, assumptions, deviations from PLAN.md/rulebook). Newest at the bottom. If a decision changes, add a new entry and mark the old one `Superseded by D-xxx`.

Entry template:

## D-001 — <short title>
- Date / Part: YYYY-MM-DD / Part N
- Status: Active | Superseded by D-xxx | Reverted
- Context: why this came up (1-3 lines)
- Decision: what we chose
- Alternatives: what was rejected and why
- Consequences: impact, risks, what must be re-checked (e.g. on hardware)
- Evidence: test/commit/measurement that supports it (or "assumption — unverified")

---

## D-001 — SITL runs PX4 v1.18.0-alpha1; hardware will run PX4 1.17.0
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: The PX4 checkout on the dev machine is `v1.18.0-alpha1-96-g1512a552f9` (commit `1512a552f9`, pre-release main branch), already built for SITL. The flight controller (Pixhawk 6C) is planned to run a 1.17.0 release.
- Decision: Keep the existing 1.18.0-alpha1 build for all SITL work rather than checking out and rebuilding a stable tag. The exact commit is recorded here and in `docs/STATUS.md` so runs stay reproducible.
- Alternatives: Pin a stable release (1.16.x or 1.15.x) and rebuild — rejected because it costs a full rebuild now and the offboard/MAVLink surface we depend on (position/velocity setpoints, `DO_SET_MODE` with PX4 custom modes, `AUTO.RTL`) is stable across these versions.
- Consequences: SITL and hardware firmware will differ by roughly one release. Anything version-sensitive — offboard mode entry, failsafe behaviour, parameter names — must be re-verified on hardware before flight. If 1.18 changes behaviour under us, re-pin to the recorded commit instead of tracking `main`.
- Evidence: `scripts/check_env.py` reports the version and the SITL smoke test passes against it (`artifacts/part0/env_report.md`, 2026-10-10). The hardware version is the user's stated plan — assumption, unverified.

## D-002 — Gazebo Harmonic (gz-sim 8.15.0) is the simulator
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: Both Gazebo Harmonic (8.15.0) and Garden (7.9.0) are installed. `gz sim --versions` prints `8.15.0` first and PX4's `px4-rc.gzsim` takes `head -n 1` and enforces a minimum of 8.0.0.
- Decision: Target Harmonic 8.15.0. The environment check records every installed version and requires the selected one to be >= 8.0.0, so a future change in which version sorts first is caught immediately.
- Alternatives: Uninstall Garden to remove the ambiguity — rejected as out of scope and possibly needed by other projects on this machine.
- Consequences: World and model SDF written in Part 1 must target gz-sim 8 plugin names and topic layouts.
- Evidence: `artifacts/part0/env_report.md` row "gz sim": found 8.15.0, note "installed: 8.15.0, 7.9.0".

## D-003 — PX4 SITL is launched from Python (`aero/sim/px4_sitl.py`), not a shell script
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: `PLAN.md` section 3 sketches `sim/run_sitl.sh`. PX4's `rcS` starts the `gz sim` server as a sibling background process, so killing the `px4` process alone leaves Gazebo running; a leftover Gazebo world makes the next run silently attach to it (see D-006). The testing rules require a pytest fixture that starts and stops the simulator with guaranteed cleanup.
- Decision: Implement the launcher as a Python context manager `Px4Sitl` with a frozen `SitlConfig`. The child is started with `start_new_session=True` and torn down with `os.killpg` (SIGINT, then SIGKILL after a grace period), which reaps PX4 and Gazebo together. `scripts/check_env.py` and the pytest `sitl` fixture both use it.
- Alternatives: A shell script plus `trap` — rejected because the pytest fixture would have to re-implement readiness detection and teardown, and shell traps do not fire reliably when pytest is interrupted.
- Consequences: `run_sitl.sh` from the planned layout is not created; if a human-facing one-liner is wanted later it should be a thin wrapper over this module. Mission code must never import `aero.sim`.
- Evidence: `make test-sitl` passes (1 passed, 12.87 s) and no `px4` or `gz` process survives the run (checked with `ps -eo comm`), 2026-10-10.

## D-004 — PX4 SITL always runs in daemon mode (`px4 -d`)
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: Launching `px4` with a non-tty stdin (`/dev/null`, or a pipe under pytest) makes the interactive `pxh>` shell spin on EOF. A first manual run produced a 290 MB console log in 25 seconds.
- Decision: Always pass `-d` (daemon mode, no `pxh` shell). `SitlConfig.argv()` hard-codes it and a unit test asserts it is present.
- Alternatives: Keep the shell and attach a pseudo-terminal — rejected as needless complexity; we never type into the PX4 shell from automation.
- Consequences: PX4 console commands are not available interactively during automated runs; use MAVLink or `px4-<module>` client binaries instead.
- Evidence: `tests/test_px4_sitl.py::test_argv_uses_daemon_mode_and_instance`; post-change log for a full smoke run is 4 KB.

## D-005 — All make targets clear `PYTHONPATH` and use the repo venv interpreter
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: ROS 2 Humble is installed system-wide on the dev machine and exports `PYTHONPATH=/opt/ros/humble/...`. Its packages register pytest entry-point plugins, so a plain `pytest` run aborted with `ModuleNotFoundError: No module named 'lark'` while loading ROS launch plugins. A plain `pytest` on `PATH` also resolved outside the venv, which made `aero` unimportable.
- Decision: Every `Makefile` target runs `env -u PYTHONPATH $(VENV)/bin/python -m <tool>` with `VENV ?= .venv`. Results no longer depend on whether the venv is activated or on the ROS environment.
- Alternatives: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` — rejected because it would also disable `pytest-timeout`, which the testing rules rely on.
- Consequences: Running `pytest` by hand in a ROS-sourced shell can still fail; use `make test` / `make test-sitl`. This project uses no ROS packages (CLAUDE.md), so nothing is lost by hiding them.
- Evidence: `make lint typecheck test` -> 39 passed, mypy clean, ruff clean, 2026-10-10.

## D-006 — Refuse to start SITL while another Gazebo world is running
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: PX4's `px4-rc.gzsim` looks for an existing `/world/<name>/clock` topic and, if it finds one, attaches to that world instead of starting its own. After a crashed or killed run, the next run would then fly in a stale world with the wrong geometry — a silent, hard-to-debug failure and a classic source of flaky SITL tests.
- Decision: `Px4Sitl.start()` calls `running_gz_worlds()` and raises `SitlError` naming the leftover world unless `SitlConfig.allow_existing_world=True`. `scripts/check_env.py` reports the same thing as a "gz world idle" check before it tries to launch.
- Alternatives: Kill leftover `gz sim` processes automatically — rejected for now: killing processes we did not start could destroy a Gazebo session the user is deliberately running. The multi-vehicle / standalone-Gazebo workflow can opt in with the flag.
- Consequences: A crashed run requires killing `gz sim` by hand before the next start; the error message says so.
- Evidence: Guard code reviewed; teardown verified to leave no `gz` process, so the guard should not trigger in normal operation (2026-10-10).

## D-007 — Version checks compare the numeric prefix only
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: Real version strings on this machine carry suffixes: PX4 `v1.18.0-alpha1-96-g1512a552f9`, libzbar `0.23.92-4build2`. A strict semver comparison would reject PX4 1.18.0-alpha1 against a 1.15.0 floor, or refuse to parse at all.
- Decision: `aero/sim/env_report.parse_version` takes the leading dotted-integer prefix and ignores every suffix, so a pre-release of X.Y.Z satisfies a minimum of X.Y.Z. The full original string (with suffix and commit) is always shown in the report.
- Alternatives: Use `packaging.version` — rejected to avoid another dependency for a report-only comparison, and its pre-release ordering (1.18.0a1 < 1.18.0) is the opposite of what we want here.
- Consequences: The check cannot distinguish a pre-release from its release. Acceptable: it is a floor check for "is the toolchain new enough", and the exact commit is recorded for reproducibility.
- Evidence: `tests/test_env_report.py::test_parse_version` / `::test_meets_minimum` cover these exact strings.

## D-008 — SITL readiness is a MAVLink heartbeat, not a console log marker
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: The first implementation decided "PX4 is ready" by scanning the console log for `Startup script returned successfully`. PX4's stdout is redirected to a plain file, so libc block-buffers it: the marker can sit unflushed while the vehicle is already alive, and can appear late enough to time out a healthy run. Review also pointed out that re-reading the whole log every 250 ms is O(n^2) in log size, and that PX4's init scripts print `ERROR [init]` for conditions they then continue past.
- Decision: `Px4Sitl.wait_until_ready()` polls for a MAVLink heartbeat on the instance's offboard UDP port and returns the decoded heartbeat. The log is read only to explain a failure (exit code, last `ERROR [init]` line), and an init error is logged at WARNING instead of aborting. Fatal conditions are: the PX4 process exits, or the heartbeat budget expires.
- Alternatives: Launch through `stdbuf -oL` or a pty to force line buffering — rejected as a workaround that still trusts text parsing for a condition MAVLink reports directly.
- Consequences: `aero/sim/px4_sitl.py` depends on `aero/sim/mavlink_probe.py` (and therefore pymavlink). Readiness now means "MAVLink is up", which is exactly what every later Part needs. Non-fatal init errors no longer fail a run; a genuinely broken init still fails, via the process exit or the timeout.
- Evidence: `make test-sitl` 1 passed in 13.8 s; `make check-env` heartbeat in 1.0 s after ready; `tests/test_px4_sitl.py::test_wait_until_ready_reports_early_exit` and `::test_wait_until_ready_times_out_without_heartbeat`.

## D-009 — The SITL process group is recorded at spawn, never derived from the pid
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: Teardown originally computed the group with `os.getpgid(process.pid)` at `stop()` time. Review reproduced a real failure: `Popen.poll()` reaps the child, so once PX4 has exited (the exact case teardown exists for) its pid is gone, `getpgid` raised `ProcessLookupError`, and `stop()` returned having signalled nothing — leaving the orphaned `gz sim` running, which then tripped the stale-world guard and kept the whole SITL suite red. The same lookup could in principle hit a recycled pid and signal an unrelated process group.
- Decision: `start()` stores the process-group id (equal to the child pid, because `start_new_session=True` makes the child a session and group leader) and writes it to `px4.pgid` in the run directory. `stop()` signals that recorded group — SIGINT, grace period, then SIGKILL from a `finally` block so a `KeyboardInterrupt` cannot abort teardown half-way — and never calls `os.getpgid`.
- Alternatives: Avoid `poll()` so the child stays unreaped — rejected: detecting an early exit is required, and relying on "not yet reaped" is fragile.
- Consequences: Teardown works whether or not PX4 is still alive. The `px4.pgid` file lets a human clean up a run killed with SIGKILL; `make sitl-clean` does the same by pattern.
- Evidence: `tests/test_px4_sitl.py::test_stop_kills_the_sibling_after_px4_already_exited` (the regression), `::test_stop_kills_the_sibling_process`, `::test_stop_escalates_to_sigkill`. After every SITL run, `ps -eo comm` shows no `px4` or `gz` process.

## D-010 — The pytest `sitl` fixture is function-scoped
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: The fixture started as session-scoped to amortise the ~13 s start-up. One shared instance would carry arm state, flight mode, parameters and vehicle position between tests, making results order-dependent, and the per-run `logs/<timestamp>_<seed>/` directory would stop being per-run.
- Decision: `sitl` and `sitl_run_dir` are function-scoped: every SITL test gets a fresh simulator and its own run directory (`logs/<timestamp>_<seed>_<pid>/`). Only the immutable inputs (`px4_dir`, `sim_seed`, `sim_speed_factor`) are session-scoped. The `sitl` marker also gets its timeout centrally in `conftest.pytest_collection_modifyitems`, because the global `timeout = 120` in `pyproject.toml` is otherwise shorter than the SITL start-up budget.
- Alternatives: Session scope with an explicit reset (disarm, mode, reposition) between tests — rejected for now: resetting PX4 reliably is harder than restarting it, and `PX4_SIM_SPEED_FACTOR` keeps the restart cost acceptable.
- Consequences: SITL suites cost ~15 s per test. If that becomes the bottleneck in Part 9 batch runs, revisit with a measured number, not a guess.
- Evidence: `make test-sitl` 1 passed in 13.8 s, 2026-10-10.

## D-011 — "Cannot ask Gazebo" is an error, not "nothing is running"
- Date / Part: 2026-10-10 / Part 0
- Status: Active
- Context: The stale-world guard (D-006) probes `gz topic -l`. The first version returned an empty list when the probe itself failed, so an unreachable or hung gz transport was indistinguishable from an idle machine — the silent fallback that `docs/rules/engineering_standards.md` rejects.
- Decision: `probe_gz_worlds()` returns `GzProbe(worlds, probe_ok)`. `Px4Sitl.start()` refuses to launch when `probe_ok` is False (message says to pass `allow_existing_world=True` to override), and `check_env.py` reports "cannot tell" as a failed check rather than a pass. The topic parsing is a separate pure function, `parse_world_names`, with unit tests.
- Alternatives: Raise from the probe itself — rejected: the environment report wants to show the condition as a row, not abort.
- Consequences: On a machine without gz on `PATH`, SITL refuses to start with an explanatory message instead of failing later and more obscurely.
- Evidence: `tests/test_px4_sitl.py::test_probe_failure_is_not_reported_as_idle`, `::test_start_refuses_when_gz_probe_fails`, `::test_parse_world_names_finds_clock_topics`.

## D-012 — The vehicle is spawned into our own gz server, not by PX4
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: Part 1 needs a generated world, camera rendering on a headless server, and a vehicle model with our own sensors. If PX4 starts Gazebo (the Part 0 path) it chooses the world and spawns a stock airframe, and `PX4_SIM_MODEL=aero_x500` would additionally require a new airframe file inside PX4's ROMFS — forking the PX4 tree, which `docs/rules/sim_sdf.md` asks us to avoid.
- Decision: `aero/sim/gz_server.py` starts `gz sim -s -r --headless-rendering <generated world>` with PX4's own `server.config` for the system plugins, spawns `sim/models/aero_x500` through gz's `create` service, and PX4 then only attaches to it (`PX4_GZ_STANDALONE=1`, `PX4_GZ_MODEL_NAME=aero_x500_0`). `PX4_SIM_MODEL` stays `gz_x500`, which selects PX4's stock *airframe* (mixer, parameters) while the simulated body is ours.
- Alternatives: Install an airframe file into PX4 via a script (rejected: forks the PX4 tree and must be redone on every PX4 update); let PX4 spawn the model from `PX4_GZ_MODELS` (same problem, plus no control of the gz command line).
- Consequences: Two processes to manage instead of one, both handled by `aero/sim/process.py`. `--headless-rendering` is ours to set, which is what makes the cameras publish. Any PX4 parameter that depends on the airframe entry is the stock x500's.
- Evidence: `tests/test_part1_sitl.py` (6 passed): PX4 logs `PX4_GZ_MODEL_NAME set, PX4 will attach to existing model` and `gz_bridge world: skyscan_1, model: aero_x500_0`, then arms, takes off and repositions under its own control.

## D-013 — The world file holds static geometry only; the vehicle is spawned separately
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: `docs/rules/sim_sdf.md` requires every generated world to be validated with `gz sdf -k`. That tool refuses to resolve `<include>` URIs at all ("Tried to use callback in sdf::findFile()"), in every form tried (`model://`, absolute path, `file://`), so a world containing the vehicle cannot be validated.
- Decision: `world.sdf` contains ground, lanes, walls, obstacles, banners, zone, QRs and red zones — no includes — and is validated with `gz sdf -k`, treating warnings as errors. The vehicle is a separate model spawned into the running world (D-012). The vehicle model itself is validated by loading it: the SITL test asserts that all four sensor topics publish.
- Alternatives: Skip validation for worlds with a vehicle (rejected: loses the check on exactly the generated part); inline the whole x500 into the world (rejected: duplicates PX4's model and breaks on PX4 updates).
- Consequences: Nothing in the pipeline can regress the generated SDF unnoticed, but a broken vehicle model only shows up in a SITL run.
- Evidence: `tests/test_make_world.py::test_generated_world_passes_gz_sdf_check` for seeds 1 and 2: "Valid.", no warnings.

## D-014 — System plugins come from PX4's `server.config`, not from the world file
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: `docs/rules/sim_sdf.md` says to copy the world plugin block from the installed PX4's `default.sdf`. In PX4 v1.18.0-alpha1 that file has no plugin block at all: `gz_env.sh` points `GZ_SIM_SERVER_CONFIG_PATH` at `src/modules/simulation/gz_bridge/server.config`, which lists physics, sensors, scene broadcaster, IMU, NavSat, magnetometer, air pressure, contact, user commands and PX4's optical-flow plugin.
- Decision: Generated worlds carry physics, scene, lighting and `<spherical_coordinates>` but no `<plugin>` elements, and `GzServerConfig` points the server at PX4's `server.config`. This is the documented rule's intent (use exactly the plugin set stock PX4 SITL uses) implemented against the installed version.
- Alternatives: Hand-copy the plugin list into every generated world (rejected: duplicates a PX4-owned list that changes between versions).
- Consequences: A generated world started without that server config has no physics or sensors. `GzServer` always sets it; anyone running `gz sim` on the file by hand must too.
- Evidence: `tests/test_sdf.py::test_world_element_has_gps_origin_and_no_plugins`, plus the SITL runs where IMU, NavSat and the cameras all work.

## D-015 — Mounts are configured in body FRD and converted once for Gazebo
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: Camera and rangefinder mounts have to be stated somewhere. Gazebo model frames are FLU (x forward, y left, z up) while this project's convention — and what `core/camera_model.py` will need in Part 3 — is aerospace body FRD (x forward, y right, z down, positive pitch nose-up).
- Decision: `sim/world_params.yaml` states every mount in body FRD, and `aero/core/geo.frd_to_gz_pose` performs the single conversion (mirror y and z, flip the pitch and yaw signs). A down-facing sensor is therefore pitch -90 deg in config and pitch +90 deg in the generated SDF, which is what PX4's own `x500_mono_cam_down` uses.
- Alternatives: Write gz-frame poses straight into the YAML (rejected: the mission layer would then need a second, separately maintained description of the same mount).
- Consequences: Reading the generated SDF alone is misleading unless the conversion is known; the function's docstring and tests carry it.
- Evidence: `tests/test_geo.py::test_frd_down_facing_sensor_becomes_gz_pitch_plus_ninety`, `tests/test_make_model.py::test_down_camera_points_at_the_ground`, and the captured down frames, which show the ground with image-up pointing forward.

## D-016 — Sensor captures pause the simulation and verify the pose before reading
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: Part 1 must photograph the start QR from 5 m, but autonomous flight needs offboard control, which is Part 2. The first implementation teleported the vehicle with gz's `set_pose` while physics ran: the model keeps whatever velocity gravity has given it, so it tumbled and the "down" frame showed sky. gz also applies a `set_pose` on a later simulation update than the one in which the service replies, so a frame taken straight afterwards can predate the move.
- Decision: `aero.sim.capture.observe_pose` pauses the world, teleports, steps 10 iterations, checks the model's ground-truth pose against the request (0.4 m tolerance), and only then steps 25 more iterations and reads the sensors. Because a paused world publishes only while stepping, capture rigs also use a long freshness limit and rely on message counters advancing instead.
- Alternatives: Re-send the pose at 10 Hz with physics running (rejected: it worked only intermittently and the drone still drifted); fly the vehicle (not possible before Part 2).
- Consequences: Captures are deterministic and repeatable. This path must never be used with PX4 attached, because PX4 runs in lockstep with the simulation clock; the SITL fixtures keep the two apart.
- Evidence: `tests/test_part1_sitl.py` sensor tests pass repeatedly; `scripts/capture_part1.py --seed 1|2` decodes the expected payload and reports a down range within 0.15 m of the commanded altitude.

## D-017 — Gazebo Python bindings are linked into the venv and pinned to protobuf 3.20.3
- Date / Part: 2026-10-10 / Part 1
- Status: Active
- Context: Reading camera and lidar topics needs `gz.transport13` / `gz.msgs10`, which Ubuntu installs into `/usr/lib/python3/dist-packages`, outside our venv. They are generated against the pre-4.x protobuf API, and the system's pip-installed protobuf 6.33.5 refuses to import them ("Descriptors cannot not be created directly") — so they are broken even for the system interpreter.
- Decision: Symlink only the `gz` package into the venv's `site-packages` and pin `protobuf==3.20.3` as a project dependency. Linking the single package keeps the rest of the system's dist-packages (including ROS) out of the venv, which `make`'s `PYTHONPATH` clearing exists to prevent (D-005).
- Alternatives: Recreate the venv with `--system-site-packages` (rejected: drags in everything and re-opens D-005); read images through the `gz topic -e` CLI (rejected: text-encoded binary, far too slow for a camera).
- Consequences: A fresh clone needs the symlink step, so it is in `make install` and `docs/SETUP.md`. protobuf is held at 3.20.3 until the distribution ships regenerated bindings.
- Evidence: `make install` on this machine, then the SITL tests, which decode 1280x720 images and LaserScans over gz transport.

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

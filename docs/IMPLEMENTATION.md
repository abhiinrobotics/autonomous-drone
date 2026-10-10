# IMPLEMENTATION LOG (append-only per Part; update status cells in place)

Purpose: exact record of what was built, how to run it, and whether it actually works.
Rule: a row may be marked VERIFIED only after the command was actually run; record the command and the result. Otherwise UNVERIFIED. If it fails: BROKEN + one-line cause.

Entry template:

## Part N — <title>  (commit <hash>, YYYY-MM-DD)
### Built
- `path/to/file.py` — what it does (1 line each)
### How to run / verify
- `command` -> expected result
### Status
| Item | Status (VERIFIED / UNVERIFIED / BROKEN) | Evidence (command + result, seeds, date) |
|---|---|---|
### Deviations from PLAN.md
- (none) | what changed and the matching D-xxx
### Known gaps / TODO
- ...
### Changes after initial implementation
- YYYY-MM-DD <hash>: what changed, why (D-xxx), re-verified? yes/no

---

## Part 0 — Environment bring-up  (commit fc3dbe2, 2026-10-10)
### Built
- `aero/sim/__init__.py` — simulation-tooling package marker; states that `phases`/`mission` must not import it.
- `aero/sim/env_report.py` — pure helpers: `parse_version`, `meets_minimum`, `make_check`, `parse_gz_versions`, `select_gz_version`, `failed`, `render_report`, `render_document`.
- `aero/sim/mavlink_probe.py` — one-shot heartbeat probe (`wait_for_heartbeat`), `offboard_udp_port(instance)`, `connection_string` (`udpin:0.0.0.0:<port>`), `Heartbeat` dataclass. Not the mission MAVLink path.
- `aero/sim/px4_sitl.py` — `SitlConfig` (frozen, validated), `Px4Sitl` context manager (start / wait_until_ready / stop), `parse_world_names`, `probe_gz_worlds`. Process group recorded at spawn and killed with SIGINT then SIGKILL; pgid written to `px4.pgid` in the run directory.
- `scripts/check_env.py` — version/table checks (OS, Python, 7 packages, QR encode+decode roundtrip, gz sim, PX4 checkout+build, gz world idle) plus the SITL heartbeat smoke test; writes `artifacts/part0/env_report.md`; exit 1 on any failure.
- `tests/conftest.py` — function-scoped `sitl` fixture (run dir `logs/<timestamp>_<seed>_<pid>/`, `meta.json`, guaranteed teardown, skip when PX4 absent); `AERO_PX4_DIR` / `AERO_SEED` / `AERO_SIM_SPEED` overrides; central timeout for `sitl`-marked tests.
- `tests/test_env_report.py` — 22 unit tests for the pure helpers (incl. real-world version strings).
- `tests/test_px4_sitl.py` — 24 unit tests: config/env/argv, gz probe parsing and refusal paths, and process teardown driven by a fake `px4` that spawns a sibling in its group.
- `tests/test_sitl_smoke.py` — Part 0 acceptance test (`@pytest.mark.sitl`).
- `Makefile` — `venv`, `install`, `lint`, `format`, `typecheck`, `test`, `test-sitl`, `check-env`, `sitl-clean`; all tool invocations via `env -u PYTHONPATH .venv/bin/python -m ...`.

### How to run / verify
- `make check-env` -> version table, all rows `yes`, `SITL smoke test: PASS`, report at `artifacts/part0/env_report.md`, exit 0.
- `make lint typecheck test` -> ruff clean, mypy clean, unit tests pass.
- `make test-sitl` -> Part 0 acceptance test (starts PX4 SITL headless, asserts a PX4 heartbeat).
- `make sitl-clean` -> only needed after a run was hard-killed.

### Status
| Item | Status (VERIFIED / UNVERIFIED / BROKEN) | Evidence (command + result, seeds, date) |
|---|---|---|
| Toolchain versions reported | VERIFIED | `make check-env` 2026-10-10: Ubuntu 22.04.5 / Python 3.10.12, pymavlink 2.4.50, numpy 2.2.6, opencv-python 5.0.0.93, pyzbar 0.1.9, PyYAML 6.0.3, qrcode 8.2, pytest 9.1.1, gz sim 8.15.0 (7.9.0 also installed), PX4 `v1.18.0-alpha1-96-g1512a552f9` built — every row `yes` |
| libzbar usable (QR encode -> decode) | VERIFIED | same run, row "qr encode/decode" = ok |
| **Acceptance: heartbeat from PX4 SITL** | VERIFIED | `make check-env` 2026-10-10: `heartbeat from sysid=1 compid=0 type=2 autopilot=12 (px4=True) in 1.0 s`; `make test-sitl`: 1 passed in 13.8 s, seed 0 |
| Acceptance: versions logged to STATUS.md | VERIFIED | `docs/STATUS.md` "Environment" section, 2026-10-10 |
| PX4 + gz teardown leaves no process | VERIFIED | `ps -eo comm \| grep -cE '^(px4\|gz)$'` = 0 after each of 4 SITL runs, 2026-10-10 |
| Teardown after PX4 already exited | VERIFIED | `pytest tests/test_px4_sitl.py` 24 passed, incl. `test_stop_kills_the_sibling_after_px4_already_exited`, 2026-10-10 |
| Unit tests + types + lint | VERIFIED | `make lint typecheck test` 2026-10-10: ruff clean, mypy clean (10 files), 54 passed / 1 deselected |

### Deviations from PLAN.md
- `sim/run_sitl.sh` from the planned layout was not created; the launcher is Python (`aero/sim/px4_sitl.py`) so the pytest fixture can share readiness detection and guaranteed process-group teardown — D-003.
- PX4 version was unexpected (pre-release `v1.18.0-alpha1`). Asked the user as PLAN.md requires; decision to keep it is D-001 (hardware will run 1.17.0).
- `aero/config/`, `aero/hal/`, `aero/core/` are not created yet; Part 0 only needed `aero/sim/`.

### Known gaps / TODO
- A run hard-killed with SIGKILL (OOM, CI kill, `kill -9` on pytest) still leaks PX4 + gz, because the child is deliberately detached from the terminal's signal group. Mitigated, not solved: the pgid is written to `px4.pgid` and `make sitl-clean` kills leftovers. A `PR_SET_PDEATHSIG` on the child would close most of the gap (would still leak the gz sibling).
- The gz stale-world guard refuses to start but never cleans up for you — by design, so we cannot kill a Gazebo session the user started deliberately (D-006).
- `SitlConfig.world` defaults to PX4's own `default` world; the project world arrives in Part 1.
- `aero/sim/__init__.py` states the import boundary (no `aero.sim` in `phases`/`mission`) as a comment; add an import-boundary test once those packages exist.
- Camera FOV / mount values are still the placeholders in `docs/rules/sim_sdf.md` — to be fixed in Part 1.

### Changes after initial implementation
- (none yet)

# Engineering standards (apply to all code)

## Design
- Layers: `hal` -> `core` -> `vision` -> `phases` -> `mission`. Dependencies point one way. `vision/` and `core/geo.py` are pure (no I/O, no globals).
- Depend on interfaces (`hal/interfaces.py` ABCs / `typing.Protocol`), inject via constructors. No module-level singletons, no mutable globals, no import-time side effects.
- Config: frozen `@dataclass(frozen=True, slots=True)` loaded once from YAML/`hardware.py`, validated at load (fail fast with a clear message). Passed explicitly.
- Phases: `class Phase: def tick(self, ctx) -> PhaseStatus` (RUNNING/DONE/FAILED + reason). No blocking inside `tick`. Per-phase timeout from config. Phases communicate via `MissionContext` (typed), not module state.
- Prefer small functions (<~40 lines), single responsibility, early returns. No premature abstraction: add an interface only where sim/hw genuinely differ.

## Python style
- 3.10+, full type hints, `from __future__ import annotations`. `mypy` clean. `ruff` clean (config in pyproject).
- Names carry units: `alt_m`, `speed_mps`, `yaw_rate_radps`, `err_px`, `dt_s`. Angles in radians internally; degrees only at I/O edges.
- Frames: local NED (x north, y east, z down); body FRD (x fwd, y right, z down); image (u right, v down). Every function that takes/returns a vector states its frame in the docstring. Conversions only in `core/geo.py` / `core/camera_model.py`.
- Docstrings: one-line summary + Args/Returns with units for public functions. Comments explain *why*, not *what*.
- Use `enum`, `dataclass`, `pathlib`, `time.monotonic()` for durations (never `time.time()` for intervals).
- No bare `except:`; catch specific exceptions; never swallow. Raise domain errors (`LinkLostError`, `SensorTimeout`, `ConfigError`). Use `logging` (module logger), structured key=value messages; no `print` outside scripts.
- No magic numbers or inline gains/thresholds -> config. Constants that are physics/protocol facts go in a named module constant with a comment.

## Concurrency & timing
- Threads: reader, setpoint streamer, camera grabber (latest-frame, drop old), logger. Shared state only through thread-safe containers (locks or queues); expose immutable snapshots.
- Every thread: daemon=False, stoppable via `threading.Event`, joined on shutdown (`close()` / context manager).
- Control loops use a fixed-rate helper (`core/loop.py`) with deadline accounting and overrun warnings; never `time.sleep(const)` as pacing in logic.
- Every blocking wait has a timeout and returns a result, never loops forever.

## Safety
- The offboard setpoint stream must never starve (see px4_offboard.md). Vision/IO exceptions must not kill the streamer.
- Command only through `FlightLink`; clamp velocities/accelerations/altitude per config; software geofence check before every position/velocity target.
- On any unhandled exception or phase FAILED: log, RTL, stop threads in order, exit non-zero.

## Testing & Definition of Done
- Pure logic: pytest unit tests incl. edge cases and sign conventions. SITL: `@pytest.mark.sitl` tests defined in PLAN.md.
- A change is done when: types+lint+tests pass; no new magic numbers; logs have enough info to debug the phase; `docs/STATUS.md` updated; commit message is Conventional Commits (`feat(phases): ...`).

## Anti-patterns (reject in review)
`time.sleep` control loops - `recv_match` outside the reader - hardware/sim imports in phases - hard-coded device paths/ports/QR size - copy-pasted helpers (one shared implementation each) - prints as logging - silent fallbacks (e.g. "assume clear if no data") - mutable default args - global state.

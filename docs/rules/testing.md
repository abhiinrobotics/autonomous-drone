# Testing rules

- `make test` = fast unit tests only (no sim). `make test-sitl` = `pytest -m sitl` (needs PX4+gz). Batch: `python scripts/batch.py --seeds 1-10 [--augment]`.
- SITL tests start/stop the simulator via a fixture (`tests/conftest.py`): headless gz (`HEADLESS=1`), unique run dir under `logs/<timestamp>_<seed>/`, guaranteed cleanup of PX4/gz processes (even on failure), per-test timeout.
- Deterministic: world seed + vision augmentation seed + PX4 `PX4_SIM_SPEED_FACTOR` recorded in the run dir `meta.json`.
- Assertions use the ground-truth **judge** (gz poses), not the drone’s own estimate.
- A flaky test is a bug: find the cause (timing, process cleanup) — don’t retry-loop it.
- Each Part ships: unit tests, its SITL acceptance test, and an entry in `docs/STATUS.md` with the measured result (e.g. 9/10 seeds).
- Logs per run: `telemetry.csv` (10-20 Hz: pos, vel, att, setpoint, ranges, phase), `events.jsonl` (phase transitions, decisions, vision results), `video_down.mp4`/`video_front.mp4` (optional flag), `meta.json`.

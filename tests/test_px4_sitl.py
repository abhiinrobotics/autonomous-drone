"""Unit tests for the PX4 SITL launcher.

The process-lifecycle tests replace the `px4` binary with a small shell script
that spawns a *sibling* in the same process group, which is how PX4's `rcS`
starts `gz sim`. They need no PX4 build and no Gazebo, so they run in
`make test`.
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import pytest

from aero.sim.px4_sitl import (
    PGID_FILENAME,
    GzProbe,
    Px4Sitl,
    SitlConfig,
    SitlError,
    parse_world_names,
    probe_gz_worlds,
)

# Deadlines for the fake-process tests. Small on purpose: these are local shell
# scripts, not a simulator.
_SPAWN_TIMEOUT_S = 10.0
_DEATH_TIMEOUT_S = 10.0
_FAST_GRACE_S = 0.5
_POLL_S = 0.05
_SIBLING_LIFETIME_S = 600


def _fake_px4_tree(root: Path, script: str, model: str = "gz_x500") -> Path:
    """Create a fake PX4 build tree whose `px4` binary is `script`."""
    build = root / "build" / "px4_sitl_default"
    (build / "bin").mkdir(parents=True)
    binary = build / "bin" / "px4"
    binary.write_text(script)
    binary.chmod(0o755)
    airframes = build / "rootfs" / "etc" / "init.d-posix" / "airframes"
    airframes.mkdir(parents=True)
    (airframes / f"4001_{model}").write_text("")
    return root


def _stub_px4(root: Path, model: str = "gz_x500") -> Path:
    """Fake tree with a `px4` that does nothing (for config-only tests)."""
    return _fake_px4_tree(root, "#!/bin/sh\n", model=model)


def _sibling_script(tail: str, sibling_file: Path) -> str:
    """Shell script that spawns a long-lived sibling, then runs `tail`."""
    return f'#!/bin/sh\nsleep {_SIBLING_LIFETIME_S} &\necho $! > "{sibling_file}"\n{tail}\n'


def _runner(root: Path, log_name: str = "px4.log", **overrides: object) -> Px4Sitl:
    """Build a runner against a fake tree, skipping the real gz probe."""
    config = SitlConfig(px4_dir=root, allow_existing_world=True, **overrides)  # type: ignore[arg-type]
    return Px4Sitl(config, root / "run" / log_name)


def _wait_for_file(path: Path, timeout_s: float = _SPAWN_TIMEOUT_S) -> str:
    """Wait for `path` to have content and return it stripped."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            text = path.read_text().strip()
            if text:
                return text
        time.sleep(_POLL_S)
    raise AssertionError(f"{path} never got content within {timeout_s} s")


def _is_alive(pid: int) -> bool:
    """Whether `pid` still exists (zombies count as gone for our purposes)."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _assert_dead(pid: int, what: str, timeout_s: float = _DEATH_TIMEOUT_S) -> None:
    """Assert `pid` disappears within `timeout_s`."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _is_alive(pid):
            return
        time.sleep(_POLL_S)
    raise AssertionError(f"{what} (pid {pid}) still alive after {timeout_s} s")


# --- configuration -----------------------------------------------------------


def test_paths_follow_px4_build_layout(tmp_path: Path) -> None:
    config = SitlConfig(px4_dir=tmp_path)
    assert config.px4_binary == tmp_path / "build" / "px4_sitl_default" / "bin" / "px4"
    assert config.rootfs_dir == tmp_path / "build" / "px4_sitl_default" / "rootfs"


def test_validate_accepts_built_tree(tmp_path: Path) -> None:
    SitlConfig(px4_dir=_stub_px4(tmp_path)).validate()  # must not raise


def test_validate_reports_missing_binary(tmp_path: Path) -> None:
    with pytest.raises(SitlError, match="px4 SITL binary not found"):
        SitlConfig(px4_dir=tmp_path).validate()


def test_validate_reports_missing_rootfs(tmp_path: Path) -> None:
    binary = tmp_path / "build" / "px4_sitl_default" / "bin" / "px4"
    binary.parent.mkdir(parents=True)
    binary.write_text("#!/bin/sh\n")
    with pytest.raises(SitlError, match="rootfs not found"):
        SitlConfig(px4_dir=tmp_path).validate()


def test_validate_reports_unknown_model(tmp_path: Path) -> None:
    config = SitlConfig(px4_dir=_stub_px4(tmp_path), model="gz_nonexistent")
    with pytest.raises(SitlError, match="unknown PX4_SIM_MODEL"):
        config.validate()


def test_offboard_port_follows_instance() -> None:
    # PX4 offsets the onboard/offboard MAVLink port per instance; binding 14540
    # for instance 2 would wait forever.
    assert SitlConfig(px4_dir=Path("/px4")).offboard_udp_port == 14540
    assert SitlConfig(px4_dir=Path("/px4"), instance=2).offboard_udp_port == 14542


def test_env_headless_sets_px4_variables() -> None:
    config = SitlConfig(px4_dir=Path("/px4"), model="gz_x500", world="skyscan", headless=True)
    env = config.env({"PATH": "/usr/bin"})
    assert env["PATH"] == "/usr/bin"
    assert env["PX4_SIM_MODEL"] == "gz_x500"
    assert env["PX4_GZ_WORLD"] == "skyscan"
    assert env["GZ_IP"] == "127.0.0.1"
    assert env["HEADLESS"] == "1"
    assert env["PX4_GZ_NO_FOLLOW"] == "1"
    assert "PX4_SIM_SPEED_FACTOR" not in env


def test_env_gui_run_omits_headless() -> None:
    env = SitlConfig(px4_dir=Path("/px4"), headless=False).env({})
    assert "HEADLESS" not in env
    assert "PX4_GZ_NO_FOLLOW" not in env


def test_env_speed_factor_and_extra_env() -> None:
    config = SitlConfig(
        px4_dir=Path("/px4"),
        speed_factor=4.0,
        extra_env=(("PX4_HOME_LAT", "12.9"),),
    )
    env = config.env({})
    assert env["PX4_SIM_SPEED_FACTOR"] == "4.0"
    assert env["PX4_HOME_LAT"] == "12.9"


def test_env_does_not_mutate_base_environment() -> None:
    base = {"PATH": "/usr/bin"}
    SitlConfig(px4_dir=Path("/px4")).env(base)
    assert base == {"PATH": "/usr/bin"}


def test_argv_uses_daemon_mode_and_instance() -> None:
    # -d is required: with a non-tty stdin the pxh shell spins on EOF.
    config = SitlConfig(px4_dir=Path("/px4"), instance=2)
    assert config.argv() == ["/px4/build/px4_sitl_default/bin/px4", "-d", "-i", "2"]


# --- gz world probe ----------------------------------------------------------


def test_parse_world_names_finds_clock_topics() -> None:
    topics = "\n".join(
        [
            "/clock",
            "/world/default/clock",
            "/world/skyscan_1/clock",
            "/world/default/stats",
            "/world/default/scene/info",
            "",
        ]
    )
    assert parse_world_names(topics) == ["default", "skyscan_1"]


def test_parse_world_names_empty_without_worlds() -> None:
    assert parse_world_names("/clock\n/stats\n") == []
    assert parse_world_names("") == []


def test_probe_failure_is_not_reported_as_idle(monkeypatch: pytest.MonkeyPatch) -> None:
    # "cannot ask gz" must never look like "nothing is running".
    monkeypatch.setattr("aero.sim.px4_sitl.subprocess.run", _raise_oserror)
    probe = probe_gz_worlds()
    assert probe == GzProbe(worlds=(), probe_ok=False)


def _raise_oserror(*args: object, **kwargs: object) -> None:
    raise OSError("gz not installed")


def test_start_refuses_when_gz_probe_fails(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("aero.sim.px4_sitl.subprocess.run", _raise_oserror)
    config = SitlConfig(px4_dir=_stub_px4(tmp_path))
    runner = Px4Sitl(config, tmp_path / "run" / "px4.log")
    with pytest.raises(SitlError, match="could not ask gz"):
        runner.start()


def test_start_refuses_when_a_world_is_running(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "aero.sim.px4_sitl.probe_gz_worlds",
        lambda *args, **kwargs: GzProbe(worlds=("stale",), probe_ok=True),
    )
    runner = Px4Sitl(SitlConfig(px4_dir=_stub_px4(tmp_path)), tmp_path / "run" / "px4.log")
    with pytest.raises(SitlError, match="already running"):
        runner.start()


# --- process lifecycle -------------------------------------------------------


def test_stop_is_safe_before_start(tmp_path: Path) -> None:
    runner = _runner(_stub_px4(tmp_path))
    runner.stop()
    runner.stop()
    assert runner.is_running is False


def test_wait_until_ready_without_start_raises(tmp_path: Path) -> None:
    with pytest.raises(SitlError, match="not started"):
        _runner(_stub_px4(tmp_path)).wait_until_ready()


def test_start_records_process_group_id(tmp_path: Path) -> None:
    sibling_file = tmp_path / "sibling.pid"
    root = _fake_px4_tree(tmp_path, _sibling_script("wait", sibling_file))
    runner = _runner(root)
    runner.start()
    try:
        pgid_text = _wait_for_file(runner.log_path.parent / PGID_FILENAME)
        assert int(pgid_text) > 0
    finally:
        runner.stop()


def test_stop_kills_the_sibling_process(tmp_path: Path) -> None:
    sibling_file = tmp_path / "sibling.pid"
    root = _fake_px4_tree(tmp_path, _sibling_script("wait", sibling_file))
    runner = _runner(root, stop_grace_s=_FAST_GRACE_S)
    runner.start()
    sibling_pid = int(_wait_for_file(sibling_file))
    assert _is_alive(sibling_pid) is True
    runner.stop()
    _assert_dead(sibling_pid, "sibling (stands in for gz sim)")
    assert runner.is_running is False


def test_stop_kills_the_sibling_after_px4_already_exited(tmp_path: Path) -> None:
    # Regression: the process group must not be looked up from the child's pid.
    # Once PX4 has been reaped (poll()), that pid is gone, and deriving the
    # group from it would either fail silently or hit a recycled pid.
    sibling_file = tmp_path / "sibling.pid"
    root = _fake_px4_tree(tmp_path, _sibling_script("exit 7", sibling_file))
    runner = _runner(root, stop_grace_s=_FAST_GRACE_S)
    runner.start()
    sibling_pid = int(_wait_for_file(sibling_file))
    deadline = time.monotonic() + _DEATH_TIMEOUT_S
    while runner.is_running and time.monotonic() < deadline:
        time.sleep(_POLL_S)
    assert runner.is_running is False, "fake px4 should have exited"
    assert _is_alive(sibling_pid) is True, "sibling should outlive px4"
    runner.stop()
    _assert_dead(sibling_pid, "orphaned sibling")


def test_stop_escalates_to_sigkill(tmp_path: Path) -> None:
    sibling_file = tmp_path / "sibling.pid"
    root = _fake_px4_tree(tmp_path, _sibling_script("trap '' INT\nwait", sibling_file))
    runner = _runner(root, stop_grace_s=_FAST_GRACE_S)
    runner.start()
    sibling_pid = int(_wait_for_file(sibling_file))
    runner.stop()
    _assert_dead(sibling_pid, "sibling of a SIGINT-ignoring px4")


def test_wait_until_ready_reports_early_exit(tmp_path: Path) -> None:
    root = _fake_px4_tree(tmp_path, "#!/bin/sh\necho 'ERROR [init] boom'\nexit 3\n")
    runner = _runner(root, stop_grace_s=_FAST_GRACE_S, instance=50)
    runner.start()
    try:
        with pytest.raises(SitlError, match="exited with code 3"):
            runner.wait_until_ready(timeout_s=_DEATH_TIMEOUT_S)
    finally:
        runner.stop()


def test_wait_until_ready_times_out_without_heartbeat(tmp_path: Path) -> None:
    sibling_file = tmp_path / "sibling.pid"
    root = _fake_px4_tree(tmp_path, _sibling_script("wait", sibling_file))
    runner = _runner(
        root,
        stop_grace_s=_FAST_GRACE_S,
        heartbeat_attempt_s=0.2,
        ready_timeout_s=0.6,
        instance=51,
    )
    runner.start()
    try:
        with pytest.raises(SitlError, match="no MAVLink heartbeat"):
            runner.wait_until_ready()
    finally:
        runner.stop()

"""Part 0 SITL acceptance test: PX4 SITL comes up and sends a heartbeat."""

from __future__ import annotations

import pytest

from aero.sim.mavlink_probe import wait_for_heartbeat
from aero.sim.px4_sitl import Px4Sitl

HEARTBEAT_TIMEOUT_S = 30.0


@pytest.mark.sitl
def test_heartbeat_received(sitl: Px4Sitl) -> None:
    assert sitl.is_running is True
    # The fixture already waited for one heartbeat; connect again to prove the
    # link is still live and reports a PX4 vehicle.
    heartbeat = wait_for_heartbeat(sitl.config.offboard_udp_port, HEARTBEAT_TIMEOUT_S)
    assert heartbeat is not None, f"no heartbeat within {HEARTBEAT_TIMEOUT_S} s"
    assert heartbeat.system_id > 0
    assert heartbeat.is_px4 is True

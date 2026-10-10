"""Minimal MAVLink heartbeat probe for simulator bring-up and smoke tests.

This is *not* the mission MAVLink path: `core/flight_link.py` (Part 2) owns the
single reader thread for that. This module exists so that simulator tooling can
answer one question — "is a PX4 instance talking on this port?" — without
pulling in flight-control machinery. It opens a connection, waits for one
heartbeat, and closes again.
"""

from __future__ import annotations

from dataclasses import dataclass

from pymavlink import mavutil

# MAVLink system id used by ground stations and tooling; PX4 vehicles use 1.
GCS_SYSTEM_ID = 255
# PX4 SITL serves the onboard/offboard MAVLink stream on this remote UDP port
# for instance 0 and increments it per instance (see px4-rc.mavlink).
OFFBOARD_UDP_PORT_BASE = 14540


class ProbeError(RuntimeError):
    """The MAVLink endpoint could not be opened (e.g. the port is in use)."""


@dataclass(frozen=True, slots=True)
class Heartbeat:
    """One decoded HEARTBEAT message.

    Args:
        system_id: MAVLink system id of the sender (PX4 vehicle is 1).
        component_id: MAVLink component id of the sender.
        mav_type: `MAV_TYPE` value (13 = hexarotor, 2 = quadrotor).
        autopilot: `MAV_AUTOPILOT` value (12 = PX4).
    """

    system_id: int
    component_id: int
    mav_type: int
    autopilot: int

    @property
    def is_px4(self) -> bool:
        """True if the heartbeat came from a PX4 autopilot."""
        return self.autopilot == int(mavutil.mavlink.MAV_AUTOPILOT_PX4)


def offboard_udp_port(instance: int = 0) -> int:
    """UDP port PX4 streams the onboard/offboard link to for `instance`."""
    return OFFBOARD_UDP_PORT_BASE + instance


def connection_string(port: int) -> str:
    """pymavlink connection string for listening on `port` (see px4_offboard.md)."""
    return f"udpin:0.0.0.0:{port}"


def wait_for_heartbeat(port: int, timeout_s: float) -> Heartbeat | None:
    """Listen on `port` for a single heartbeat.

    Args:
        port: UDP port to bind.
        timeout_s: How long to wait for the first heartbeat.

    Returns:
        The decoded heartbeat, or None if none arrived before the timeout.

    Raises:
        ProbeError: The port could not be bound.
    """
    try:
        connection = mavutil.mavlink_connection(
            connection_string(port), source_system=GCS_SYSTEM_ID
        )
    except OSError as error:
        raise ProbeError(f"cannot listen on {connection_string(port)}: {error}") from error
    try:
        message = connection.wait_heartbeat(timeout=timeout_s)
        if message is None:
            return None
        return Heartbeat(
            system_id=connection.target_system,
            component_id=connection.target_component,
            mav_type=int(message.type),
            autopilot=int(message.autopilot),
        )
    finally:
        connection.close()

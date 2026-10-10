"""Latest-message gz-transport subscriber shared by the SITL sensor drivers.

Every gz sensor behaves the same way from the mission's point of view: a topic
delivers messages faster than the mission consumes them, and only the newest
one matters. This module owns that pattern once — a gz node, a callback that
replaces the cached message, and a lock — so the camera and rangefinder
drivers only have to decode.

The gz Python bindings deliver callbacks on their own thread, so the cache is
guarded by a lock and handed out as an immutable snapshot.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Generic, TypeVar

from gz.transport13 import Node

from aero.core.errors import SensorTimeout

logger = logging.getLogger(__name__)

MessageT = TypeVar("MessageT")


class GzSubscriber(Generic[MessageT]):
    """Keeps the most recent message of one gz topic."""

    def __init__(self, topic: str, message_type: type[MessageT], max_age_s: float) -> None:
        """Subscribe to `topic`.

        Args:
            topic: Full gz topic name.
            message_type: Generated protobuf class of the topic's messages.
            max_age_s: Oldest message `latest()` will still return.

        Raises:
            SensorTimeout: gz refused the subscription (wrong type or no
                transport), which is a setup error, not a transient one.
        """
        self._topic = topic
        self._max_age_s = max_age_s
        self._lock = threading.Lock()
        self._message: MessageT | None = None
        self._received_s = 0.0
        self._sequence = 0
        # The node must stay referenced: dropping it cancels the subscription.
        self._node: Node | None = Node()
        if not self._node.subscribe(message_type, topic, self._on_message):
            raise SensorTimeout(f"could not subscribe to gz topic {topic!r}")
        logger.info("gz_subscribed topic=%s type=%s", topic, message_type.__name__)

    @property
    def topic(self) -> str:
        """Topic this subscriber listens to."""
        return self._topic

    @property
    def sequence(self) -> int:
        """Number of messages received so far."""
        with self._lock:
            return self._sequence

    def _on_message(self, message: Any) -> None:
        """Replace the cached message (called on a gz transport thread)."""
        with self._lock:
            self._message = message
            self._received_s = time.monotonic()
            self._sequence += 1

    def latest(self) -> tuple[MessageT, float, int]:
        """Return the newest message with its arrival time and sequence number.

        Returns:
            (message, monotonic receive time, sequence number).

        Raises:
            SensorTimeout: Nothing received yet, or the newest message is older
                than `max_age_s`.
        """
        with self._lock:
            message = self._message
            received_s = self._received_s
            sequence = self._sequence
        if message is None:
            raise SensorTimeout(f"no message yet on gz topic {self._topic!r}")
        age_s = time.monotonic() - received_s
        if age_s > self._max_age_s:
            raise SensorTimeout(
                f"gz topic {self._topic!r} is stale: {age_s:.2f} s old "
                f"(limit {self._max_age_s:.2f} s)"
            )
        return message, received_s, sequence

    def wait_for_message(self, timeout_s: float, poll_interval_s: float = 0.05) -> None:
        """Block until the first message arrives.

        Args:
            timeout_s: How long to wait.
            poll_interval_s: Poll period.

        Raises:
            SensorTimeout: Nothing arrived within `timeout_s`.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._message is not None:
                    return
            time.sleep(poll_interval_s)
        raise SensorTimeout(f"no message on gz topic {self._topic!r} within {timeout_s:.1f} s")

    def close(self) -> None:
        """Drop the subscription. Safe to call more than once."""
        with self._lock:
            self._message = None
        self._node = None

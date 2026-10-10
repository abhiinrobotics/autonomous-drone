"""Ground-truth model poses straight from Gazebo.

The judge (Part 9) and the frame-mapping test must not trust the drone's own
estimate, so they read the simulator's pose topic instead. This module is the
one place that knows how gz publishes poses.

`aero.sim` may use `aero.hal.sitl` helpers; the dependency never points the
other way, and mission code uses neither.
"""

from __future__ import annotations

import logging
import math

from gz.msgs10.pose_v_pb2 import Pose_V

from aero.core.errors import SensorTimeout
from aero.core.geo import Enu
from aero.hal.sitl.gz_subscriber import GzSubscriber

logger = logging.getLogger(__name__)

# Pose topics publish at the simulation rate; anything older than this is not
# the current pose.
DEFAULT_MAX_AGE_S = 2.0


def pose_topic(world_name: str) -> str:
    """Topic carrying the poses of every dynamic model in `world_name`."""
    return f"/world/{world_name}/dynamic_pose/info"


class ModelPoseWatcher:
    """Latest ground-truth pose of one model."""

    def __init__(self, world_name: str, model_name: str, max_age_s: float = DEFAULT_MAX_AGE_S):
        """Subscribe to the world's pose topic.

        Args:
            world_name: Name of the running world.
            model_name: Model instance whose pose is wanted.
            max_age_s: Oldest pose message `read()` will still use.

        Raises:
            SensorTimeout: The subscription was refused.
        """
        self._model_name = model_name
        self._subscriber: GzSubscriber[Pose_V] | None = GzSubscriber(
            pose_topic(world_name), Pose_V, max_age_s
        )

    @property
    def model_name(self) -> str:
        """Model this watcher follows."""
        return self._model_name

    def wait_for_pose(self, timeout_s: float) -> None:
        """Block until a pose message has arrived.

        Args:
            timeout_s: How long to wait.

        Raises:
            SensorTimeout: Nothing arrived in time, or the watcher is closed.
        """
        self._require_open().wait_for_message(timeout_s)

    def read(self) -> tuple[Enu, float]:
        """Return the model's position and heading.

        Returns:
            (position in world ENU metres, yaw about +z in radians).

        Raises:
            SensorTimeout: No fresh pose, the model is absent from the latest
                message, or the watcher is closed.
        """
        message, _, _ = self._require_open().latest()
        for pose in message.pose:
            if pose.name != self._model_name:
                continue
            position = Enu(
                east_m=float(pose.position.x),
                north_m=float(pose.position.y),
                up_m=float(pose.position.z),
            )
            quaternion = pose.orientation
            yaw_rad = math.atan2(
                2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
            )
            return position, yaw_rad
        raise SensorTimeout(f"model {self._model_name!r} is not in the latest pose message")

    def close(self) -> None:
        """Drop the subscription. Safe to call more than once."""
        if self._subscriber is not None:
            self._subscriber.close()
            self._subscriber = None

    def _require_open(self) -> GzSubscriber[Pose_V]:
        """Return the subscriber or explain that the watcher is closed.

        Raises:
            SensorTimeout: The watcher has been closed.
        """
        if self._subscriber is None:
            raise SensorTimeout(f"pose watcher for {self._model_name!r} is closed")
        return self._subscriber

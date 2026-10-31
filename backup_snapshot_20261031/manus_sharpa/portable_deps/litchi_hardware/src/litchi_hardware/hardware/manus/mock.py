"""Deterministic in-process fake Manus tracker."""

from __future__ import annotations

import math
import time

from litchi_hardware.core.errors import ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import HandSide, HandTrackingFrame, Pose

from .base import ManusTrackerBase
from .config import ManusConfig
from .schema import MANUS_SHARPA_25_SCHEMA

_FINGER_LENGTHS = {
    "thumb": (0.035, 0.028, 0.024, 0.020),
    "index": (0.050, 0.036, 0.025, 0.020),
    "middle": (0.055, 0.040, 0.027, 0.021),
    "ring": (0.052, 0.037, 0.025, 0.020),
    "pinky": (0.044, 0.030, 0.021, 0.018),
}


class MockManusTracker(ManusTrackerBase):
    """Produces a smooth cycle in the 25-keypoint teleoperation order."""

    def __init__(self, config: ManusConfig) -> None:
        super().__init__(config)
        self._started_at = 0.0
        self._sequence = 0

    @property
    def transport_name(self) -> str:
        return "fake"

    @property
    def available_sides(self) -> frozenset[HandSide]:
        return frozenset(HandSide)

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        self._started_at = time.monotonic()
        self._sequence = 0
        self._lifecycle.connected()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._lifecycle.disconnected()

    def read_frame(
        self, side: HandSide, timeout_s: float | None = None
    ) -> HandTrackingFrame:
        del timeout_s
        if not self.is_connected:
            raise ConnectionError("fake Manus tracker is not connected")
        elapsed = time.monotonic() - self._started_at
        closure = 0.5 - 0.5 * math.cos(elapsed * 1.2)
        frame = HandTrackingFrame(
            side=side,
            keypoints=self._make_keypoints(side, closure),
            timestamp_s=time.time(),
            schema=MANUS_SHARPA_25_SCHEMA,
            frame_id=self.config.frame_id,
            sequence=self._sequence,
        )
        self._sequence += 1
        return frame

    @staticmethod
    def _make_keypoints(side: HandSide, closure: float) -> tuple[Pose, ...]:
        points = [Pose.identity()]
        lateral_sign = 1.0 if side is HandSide.LEFT else -1.0

        # Teleoperation order: wrist, 4 thumb nodes, then 5 nodes per finger.
        thumb_base = (0.026 * lateral_sign, 0.0, 0.018)
        thumb_angle = closure * 0.9
        x, y, z = thumb_base
        for index, length in enumerate(_FINGER_LENGTHS["thumb"]):
            x += lateral_sign * length * math.cos(0.7 + thumb_angle + index * 0.12)
            y += length * math.sin(thumb_angle + index * 0.12)
            points.append(Pose((x, y, z), (0.0, 0.0, 0.0, 1.0)))

        bases = (0.030, 0.010, -0.010, -0.027)
        for finger_name, base_x in zip(
            ("index", "middle", "ring", "pinky"), bases, strict=True
        ):
            x = lateral_sign * base_x
            y = 0.0
            z = 0.025
            points.append(Pose((x, y, z), (0.0, 0.0, 0.0, 1.0)))
            for segment_index, length in enumerate(_FINGER_LENGTHS[finger_name]):
                angle = closure * (0.35 + 0.35 * segment_index)
                y += length * math.sin(angle)
                z += length * math.cos(angle)
                points.append(Pose((x, y, z), (0.0, 0.0, 0.0, 1.0)))

        return tuple(points)

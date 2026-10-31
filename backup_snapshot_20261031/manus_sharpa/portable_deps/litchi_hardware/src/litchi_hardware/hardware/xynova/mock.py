"""In-process fake Xynova hand implementing the core interface."""

from __future__ import annotations

import time

from litchi_hardware.core.errors import ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, TactileFrame
from litchi_hardware.hardware.xynova._ffi import TACTILE_NUM

from .base import XynovaHandBase
from .config import XynovaConfig
from .schema import XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA


class MockXynovaHand(XynovaHandBase):
    def __init__(self, config: XynovaConfig) -> None:
        super().__init__(config)
        zero = (0.0,) * XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.size
        self._position = zero
        self._last_position = zero
        self._last_update = time.monotonic()
        self._stopped = False
        self._serial = f"MOCK-{config.device_id:02X}"

    @property
    def transport_name(self) -> str:
        return "fake"

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        self._stopped = False
        self._last_update = time.monotonic()
        self._lifecycle.connected()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._stopped = True
        self._lifecycle.disconnected()

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        if self._stopped:
            raise ConnectionError("fake Xynova hand is stopped")
        self._last_position = self._position
        self._position = positions
        self._last_update = time.monotonic()

    def read_joint_state(self) -> JointState:
        if not self.is_connected:
            raise ConnectionError("fake Xynova hand is not connected")
        elapsed = max(time.monotonic() - self._last_update, 1e-6)
        velocity = tuple(
            (current - previous) / elapsed
            for current, previous in zip(self._position, self._last_position, strict=True)
        )
        return JointState(
            XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.names,
            self._position,
            velocity,
            (),
            timestamp_s=time.time(),
            schema=XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA,
        )

    def read_tactile(self, timeout_s: float | None = None) -> TactileFrame:
        del timeout_s
        if not self.is_connected:
            raise ConnectionError("fake Xynova hand is not connected")
        if not self.config.enable_tactile:
            raise ConnectionError("fake Xynova tactile feedback is disabled")
        return TactileFrame(
            values=(0.0,) * (TACTILE_NUM * 6),
            shape=(TACTILE_NUM, 6),
            timestamp_s=time.time(),
            frame_id="xynova_tactile_wrench",
        )

    def stop(self) -> None:
        self._stopped = True

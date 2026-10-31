"""In-process fake Linker Hand actuator."""

from __future__ import annotations

import time

from litchi_hardware.core.errors import ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, TactileFrame

from .base import LinkerHandBase
from .config import LinkerConfig


class MockLinkerHand(LinkerHandBase):
    def __init__(self, config: LinkerConfig) -> None:
        super().__init__(config)
        self._position = config.open_position
        self._previous = config.open_position
        self._updated_at = time.monotonic()

    @property
    def transport_name(self) -> str:
        return "fake"

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        self._position = self.config.open_position
        self._previous = self._position
        self._updated_at = time.monotonic()
        self._stopped = False
        self._serial = f"FAKE-{self.config.model.value}-{self.config.side.value}"
        self._firmware_version = "fake"
        self._lifecycle.connected()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._stopped = True
        self._lifecycle.disconnected()

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        self._previous = self._position
        self._position = positions
        self._updated_at = time.monotonic()

    def read_joint_state(self) -> JointState:
        if not self.is_connected:
            raise ConnectionError("fake Linker Hand is not connected")
        elapsed = max(time.monotonic() - self._updated_at, 1e-6)
        velocity = tuple(
            (current - previous) / elapsed
            for current, previous in zip(self._position, self._previous, strict=True)
        )
        return JointState(
            self.joint_schema.names,
            self._position,
            velocity,
            timestamp_s=time.time(),
            schema=self.joint_schema,
        )

    def read_tactile(self, finger: str = "thumb", timeout_s: float | None = 0.0) -> TactileFrame:
        del timeout_s
        if not self.is_connected:
            raise ConnectionError("fake Linker Hand is not connected")
        if not self.config.enable_tactile:
            raise ConnectionError("fake Linker Hand tactile is disabled")
        return TactileFrame(
            (0.0, 0.25, 0.5, 0.75),
            (2, 2),
            time.time(),
            f"linker/{self.config.side.value}/{finger}",
        )

    def stop(self) -> None:
        self._stopped = True

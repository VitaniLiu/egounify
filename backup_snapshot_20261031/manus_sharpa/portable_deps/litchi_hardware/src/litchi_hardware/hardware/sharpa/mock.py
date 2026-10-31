"""In-process fake Sharpa Wave actuator."""

from __future__ import annotations

import time

from litchi_hardware.core.errors import ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import HandSide, JointState

from .base import SharpaHandBase
from .config import SharpaConfig
from .schema import SHARPA_OPEN_POSITION, SHARPA_WAVE_JOINT_SCHEMA
from .tactile import SharpaFinger, SharpaTactileFrame, TactileImage, tactile_channel


class MockSharpaHand(SharpaHandBase):
    def __init__(self, config: SharpaConfig) -> None:
        super().__init__(config)
        self._position = SHARPA_OPEN_POSITION
        self._last_position = SHARPA_OPEN_POSITION
        self._last_update = time.monotonic()
        self._stopped = False

    @property
    def transport_name(self) -> str:
        return "fake"

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        self._position = SHARPA_OPEN_POSITION
        self._last_position = SHARPA_OPEN_POSITION
        self._last_update = time.monotonic()
        self._stopped = False
        self._lifecycle.connected()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._stopped = True
        self._lifecycle.disconnected()

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        if self._stopped:
            raise ConnectionError("fake Sharpa hand is stopped")
        self._last_position = self._position
        self._position = positions
        self._last_update = time.monotonic()

    def read_joint_state(self) -> JointState:
        if not self.is_connected:
            raise ConnectionError("fake Sharpa hand is not connected")
        elapsed = max(time.monotonic() - self._last_update, 1e-6)
        velocity = tuple(
            (current - previous) / elapsed
            for current, previous in zip(
                self._position, self._last_position, strict=True
            )
        )
        return JointState(
            SHARPA_WAVE_JOINT_SCHEMA.names,
            self._position,
            velocity,
            (0.0,) * SHARPA_WAVE_JOINT_SCHEMA.size,
            timestamp_s=time.time(),
            schema=SHARPA_WAVE_JOINT_SCHEMA,
        )

    @property
    def tactile_available(self) -> bool:
        return self.config.enable_tactile

    def read_tactile(
        self, finger: SharpaFinger | str, timeout_s: float | None = 0.0
    ) -> SharpaTactileFrame | None:
        del timeout_s
        if not self.is_connected:
            raise ConnectionError("fake Sharpa hand is not connected")
        if not self.tactile_available:
            raise ConnectionError("fake Sharpa tactile is disabled")
        resolved_finger = SharpaFinger(finger)
        side = self._side or HandSide.LEFT
        image = TactileImage(2, 2, bytes((0, 32, 64, 96)))
        return SharpaTactileFrame(
            side=side,
            finger=resolved_finger,
            channel=tactile_channel(side, resolved_finger),
            frame_index=int(time.monotonic() * 30),
            timestamp_s=time.time(),
            raw=image,
            deformation=image,
            wrench=(0.0,) * 6,
        )

    def stop(self) -> None:
        self._stopped = True

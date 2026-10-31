"""HAL adapter for the in-process Manus SDK backend."""

from __future__ import annotations

from threading import RLock

from litchi_hardware.core.errors import (
    ConnectionError,
    DependencyUnavailableError,
)
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import HandSide, HandTrackingFrame

from ._native import CtypesManusBridge, NativeBridge
from .base import ManusTrackerBase
from .config import ManusNativeConfig
from .schema import MANUS_SHARPA_25_SCHEMA
from .vendor import manus_calibration_dir


class ManusNativeTracker(ManusTrackerBase):
    """Read Manus skeletons through the project-owned native bridge."""

    def __init__(
        self, config: ManusNativeConfig, *, bridge: NativeBridge | None = None
    ) -> None:
        if not isinstance(config, ManusNativeConfig):
            raise TypeError("ManusNativeTracker requires ManusNativeConfig")
        super().__init__(config)
        self.config = config
        self._bridge = bridge
        self._lock = RLock()

    @property
    def transport_name(self) -> str:
        return "native"

    @property
    def available_sides(self) -> frozenset[HandSide]:
        if self._bridge is None:
            return frozenset(HandSide)
        return self._bridge.available_sides or frozenset(HandSide)

    def connect(self) -> None:
        with self._lock:
            if self.is_connected:
                return
            if self.connection_state is ConnectionState.FAILED:
                self._lifecycle.reset()
            self._lifecycle.begin_connect()
            try:
                if self._bridge is None:
                    self._bridge = CtypesManusBridge(self.config.native_library)
                calibration_directory = (
                    self.config.calibration_directory
                    or str(manus_calibration_dir())
                )
                self._bridge.connect(
                    self.config.native_host_discovery_s,
                    self.config.native_loopback_only,
                    calibration_directory,
                )
                self._lifecycle.connected()
            except Exception as exc:
                self._lifecycle.failed()
                if isinstance(exc, (ConnectionError, DependencyUnavailableError)):
                    raise
                raise ConnectionError(
                    f"failed to connect native Manus tracker: {exc}"
                ) from exc

    def disconnect(self) -> None:
        with self._lock:
            if self.connection_state is ConnectionState.DISCONNECTED:
                return
            try:
                if self._bridge is not None:
                    self._bridge.disconnect()
            finally:
                self._lifecycle.disconnected()

    def read_frame(
        self, side: HandSide, timeout_s: float | None = None
    ) -> HandTrackingFrame:
        with self._lock:
            if not self.is_connected or self._bridge is None:
                raise ConnectionError("Manus native tracker is not connected")
            timeout = self.config.frame_timeout_s if timeout_s is None else timeout_s
            if timeout < 0:
                raise ValueError("timeout_s must not be negative")
            frame = self._bridge.read_frame(side, timeout)
            if frame is None:
                raise TimeoutError(
                    f"no {side.value} Manus native frame received in {timeout:.3f}s"
                )
            return HandTrackingFrame(
                side=frame.side,
                keypoints=frame.keypoints,
                timestamp_s=frame.timestamp_s,
                schema=MANUS_SHARPA_25_SCHEMA,
                frame_id=self.config.frame_id,
                sequence=frame.sequence,
            )

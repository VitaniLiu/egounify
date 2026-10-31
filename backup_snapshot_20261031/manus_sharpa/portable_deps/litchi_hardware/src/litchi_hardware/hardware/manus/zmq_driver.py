"""Manus tracker backed by the Sharpa integration client's ZMQ stream."""

from __future__ import annotations

import os
import shutil
import subprocess
from collections import deque
from threading import RLock
from typing import Any

from litchi_hardware.core.errors import (
    ConnectionError,
    DependencyUnavailableError,
)
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import HandSide, HandTrackingFrame

from .base import ManusTrackerBase
from .config import ManusZmqConfig
from .protocol import decode_mocap_keypoints
from .schema import MANUS_SHARPA_25_SCHEMA


class ManusZmqTracker(ManusTrackerBase):
    def __init__(self, config: ManusZmqConfig) -> None:
        if not isinstance(config, ManusZmqConfig):
            raise TypeError("ManusZmqTracker requires ManusZmqConfig")
        super().__init__(config)
        self._context: Any = None
        self._socket: Any = None
        self._client_process: subprocess.Popen[bytes] | None = None
        self._pending: dict[HandSide, deque[HandTrackingFrame]] = {
            HandSide.LEFT: deque(maxlen=1),
            HandSide.RIGHT: deque(maxlen=1),
        }
        self._lock = RLock()

    @property
    def transport_name(self) -> str:
        return "zmq-protobuf"

    @property
    def available_sides(self) -> frozenset[HandSide]:
        return frozenset(
            side for side, frames in self._pending.items() if frames
        ) or frozenset(HandSide)

    @staticmethod
    def _load_zmq() -> Any:
        try:
            import zmq
        except ImportError as exc:
            raise DependencyUnavailableError(
                "Manus ZMQ backend requires pyzmq; install litchi-hardware[manus]"
            ) from exc
        return zmq

    def connect(self) -> None:
        with self._lock:
            if self.is_connected:
                return
            if self.connection_state is ConnectionState.FAILED:
                self._lifecycle.reset()
            self._lifecycle.begin_connect()
            try:
                zmq = self._load_zmq()
                if self.config.manage_client and self.config.client_executable:
                    executable = (
                        shutil.which(self.config.client_executable)
                        or os.path.abspath(self.config.client_executable)
                    )
                    command = [
                        executable,
                        *self.config.client_arguments,
                    ]
                    working_directory = (
                        self.config.client_working_directory
                        or os.path.dirname(executable)
                    )
                    self._client_process = subprocess.Popen(
                        command,
                        cwd=working_directory,
                        stdin=subprocess.DEVNULL,
                        stdout=None,
                        stderr=None,
                    )
                self._context = zmq.Context()
                self._socket = self._context.socket(zmq.SUB)
                self._socket.setsockopt(zmq.RCVHWM, 1)
                self._socket.setsockopt(zmq.LINGER, 0)
                self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
                self._socket.connect(self.config.endpoint)
                self._lifecycle.connected()
            except Exception as exc:
                self._cleanup()
                self._lifecycle.failed()
                if isinstance(exc, DependencyUnavailableError):
                    raise
                raise ConnectionError(f"failed to connect Manus tracker: {exc}") from exc

    def disconnect(self) -> None:
        with self._lock:
            if self.connection_state is ConnectionState.DISCONNECTED:
                return
            self._cleanup()
            self._pending[HandSide.LEFT].clear()
            self._pending[HandSide.RIGHT].clear()
            self._lifecycle.disconnected()

    def _cleanup(self) -> None:
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        if self._context is not None:
            self._context.term()
            self._context = None
        if self._client_process is not None:
            if self._client_process.poll() is None:
                self._client_process.terminate()
                try:
                    self._client_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._client_process.kill()
                    self._client_process.wait(timeout=1.0)
            self._client_process = None

    def read_frame(
        self, side: HandSide, timeout_s: float | None = None
    ) -> HandTrackingFrame:
        with self._lock:
            if not self.is_connected or self._socket is None:
                raise ConnectionError("Manus tracker is not connected")
            if self._pending[side]:
                return self._pending[side].pop()

            zmq = self._load_zmq()
            timeout = self.config.frame_timeout_s if timeout_s is None else timeout_s
            if timeout < 0:
                raise ValueError("timeout_s must not be negative")
            if self._socket.poll(round(timeout * 1000), zmq.POLLIN) == 0:
                raise TimeoutError(
                    f"no Manus frame received from {self.config.endpoint} in {timeout:.3f}s"
                )
            packet = decode_mocap_keypoints(self._socket.recv())
            if packet.left:
                self._pending[HandSide.LEFT].append(
                    HandTrackingFrame(
                        HandSide.LEFT,
                        packet.left,
                        packet.timestamp_s,
                        MANUS_SHARPA_25_SCHEMA,
                        self.config.frame_id,
                        packet.sequence,
                    )
                )
            if packet.right:
                self._pending[HandSide.RIGHT].append(
                    HandTrackingFrame(
                        HandSide.RIGHT,
                        packet.right,
                        packet.timestamp_s,
                        MANUS_SHARPA_25_SCHEMA,
                        self.config.frame_id,
                        packet.sequence,
                    )
                )
            if not self._pending[side]:
                raise TimeoutError(f"Manus packet did not contain {side.value} hand")
            return self._pending[side].pop()

"""Connection lifecycle state machine and its invariants."""

from __future__ import annotations

from threading import RLock

from ._compat import StrEnum
from .errors import InvalidStateError


class ConnectionState(StrEnum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    FAILED = "failed"


_ALLOWED_TRANSITIONS: dict[ConnectionState, frozenset[ConnectionState]] = {
    ConnectionState.DISCONNECTED: frozenset({ConnectionState.CONNECTING}),
    ConnectionState.CONNECTING: frozenset(
        {ConnectionState.CONNECTED, ConnectionState.FAILED, ConnectionState.DISCONNECTED}
    ),
    ConnectionState.CONNECTED: frozenset(
        {ConnectionState.DISCONNECTED, ConnectionState.FAILED}
    ),
    ConnectionState.FAILED: frozenset({ConnectionState.DISCONNECTED}),
}


class LifecycleGuard:
    """Thread-safe owner of a device connection state."""

    def __init__(self) -> None:
        self._state = ConnectionState.DISCONNECTED
        self._lock = RLock()

    @property
    def state(self) -> ConnectionState:
        with self._lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        return self.state is ConnectionState.CONNECTED

    def transition(self, target: ConnectionState) -> None:
        with self._lock:
            if target is self._state:
                return
            if target not in _ALLOWED_TRANSITIONS[self._state]:
                raise InvalidStateError(
                    f"invalid connection transition: {self._state.value} -> {target.value}"
                )
            self._state = target

    def begin_connect(self) -> None:
        self.transition(ConnectionState.CONNECTING)

    def connected(self) -> None:
        self.transition(ConnectionState.CONNECTED)

    def failed(self) -> None:
        self.transition(ConnectionState.FAILED)

    def disconnected(self) -> None:
        self.transition(ConnectionState.DISCONNECTED)

    def reset(self) -> None:
        if self.state is ConnectionState.DISCONNECTED:
            return
        if self.state is not ConnectionState.FAILED:
            raise InvalidStateError("only a failed lifecycle can be reset")
        self.disconnected()

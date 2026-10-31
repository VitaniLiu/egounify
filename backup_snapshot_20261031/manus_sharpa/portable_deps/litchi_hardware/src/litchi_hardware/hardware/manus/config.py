"""Typed configuration for Manus tracking backends."""

from __future__ import annotations

from dataclasses import dataclass

from litchi_hardware.core._compat import StrEnum
from litchi_hardware.core.config import DeviceConfig


class ManusBackend(StrEnum):
    """Available Manus transport implementations."""

    NATIVE = "native"
    ZMQ = "zmq"


@dataclass(frozen=True, slots=True)
class ManusConfig(DeviceConfig):
    """Settings shared by every Manus tracker implementation."""

    frame_timeout_s: float = 0.25
    frame_id: str = "manus_hand"

    def __post_init__(self) -> None:
        super(ManusConfig, self).__post_init__()
        if self.frame_timeout_s <= 0:
            raise ValueError("frame_timeout_s must be positive")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")


@dataclass(frozen=True, slots=True)
class ManusNativeConfig(ManusConfig):
    """In-process Manus SDK settings."""

    native_library: str | None = None
    native_host_discovery_s: int = 1
    native_loopback_only: bool = False
    calibration_directory: str | None = None

    def __post_init__(self) -> None:
        super(ManusNativeConfig, self).__post_init__()
        if (
            isinstance(self.native_host_discovery_s, bool)
            or not isinstance(self.native_host_discovery_s, int)
            or not 0 <= self.native_host_discovery_s <= 0xFFFFFFFF
        ):
            raise ValueError(
                "native_host_discovery_s must fit an unsigned 32-bit integer"
            )


@dataclass(frozen=True, slots=True)
class ManusZmqConfig(ManusConfig):
    """ZMQ subscriber and optional managed-client settings."""

    endpoint: str = "tcp://127.0.0.1:2044"
    manage_client: bool = True
    client_executable: str | None = None
    client_working_directory: str | None = None
    client_arguments: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        super(ManusZmqConfig, self).__post_init__()
        if not self.endpoint:
            raise ValueError("endpoint must not be empty")
        object.__setattr__(self, "client_arguments", tuple(self.client_arguments))

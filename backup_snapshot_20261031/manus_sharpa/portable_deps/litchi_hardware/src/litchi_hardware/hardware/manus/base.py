"""Shared lifecycle and identity for Manus tracking backends."""

from __future__ import annotations

from abc import abstractmethod

from litchi_hardware.core.config import DeviceInfo
from litchi_hardware.core.interfaces import HandTrackerInterface
from litchi_hardware.core.lifecycle import ConnectionState, LifecycleGuard

from .config import ManusConfig


class ManusTrackerBase(HandTrackerInterface):
    def __init__(self, config: ManusConfig) -> None:
        self.config = config
        self._lifecycle = LifecycleGuard()
        self._info = DeviceInfo(
            name=config.name,
            vendor="Manus",
            model="MetaGlove",
            metadata={"transport": self.transport_name},
        )

    @property
    @abstractmethod
    def transport_name(self) -> str: ...

    @property
    def info(self) -> DeviceInfo:
        return self._info

    @property
    def connection_state(self) -> ConnectionState:
        return self._lifecycle.state

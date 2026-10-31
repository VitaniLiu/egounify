"""Core configuration and identity value objects."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any


@dataclass(frozen=True, slots=True)
class DeviceConfig:
    """Common device configuration."""

    name: str
    connect_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("device name must not be empty")
        if self.connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")


@dataclass(frozen=True, slots=True)
class DeviceInfo:
    """Backend-neutral device identity and capabilities."""

    name: str
    vendor: str
    model: str
    serial: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name or not self.vendor or not self.model:
            raise ValueError("name, vendor, and model must not be empty")
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))

"""Configuration for a Sharpa Wave hand."""

from __future__ import annotations

from dataclasses import dataclass

from litchi_hardware.core.config import DeviceConfig
from litchi_hardware.core.types import HandSide


@dataclass(frozen=True, slots=True)
class SharpaConfig(DeviceConfig):
    serial: str | None = None
    side: HandSide | None = None
    speed_coefficient: float = 0.3
    current_coefficient: float = 0.6
    interpolate: bool = True
    enable_tactile: bool = True
    discovery_timeout_s: float = 10.0
    sdk_python_path: str | None = None

    def __post_init__(self) -> None:
        super(SharpaConfig, self).__post_init__()
        if not 0.0 <= self.speed_coefficient <= 1.0:
            raise ValueError("speed_coefficient must be in [0, 1]")
        if not 0.0 <= self.current_coefficient <= 1.0:
            raise ValueError("current_coefficient must be in [0, 1]")
        if self.discovery_timeout_s <= 0:
            raise ValueError("discovery_timeout_s must be positive")

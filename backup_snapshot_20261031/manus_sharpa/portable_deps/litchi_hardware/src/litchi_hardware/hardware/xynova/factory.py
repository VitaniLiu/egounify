"""Factory for real and fake core-aligned Xynova hands."""

from __future__ import annotations

from litchi_hardware.core.interfaces import HandInterface

from .config import XynovaConfig
from .mock import MockXynovaHand
from .sdk_driver import XynovaSdkHand


def create_hand(
    config: XynovaConfig | None = None, *, use_fake_hardware: bool = False
) -> HandInterface:
    resolved = config or XynovaConfig(name="xynova")
    if use_fake_hardware:
        return MockXynovaHand(resolved)
    return XynovaSdkHand(resolved)

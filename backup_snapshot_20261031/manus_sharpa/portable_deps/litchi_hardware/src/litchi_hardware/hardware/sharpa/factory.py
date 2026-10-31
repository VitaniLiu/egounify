"""Factory for real and fake Sharpa Wave hands."""

from __future__ import annotations

from litchi_hardware.core.interfaces import HandInterface

from .config import SharpaConfig
from .mock import MockSharpaHand
from .sdk_driver import SharpaSdkHand


def create_hand(
    config: SharpaConfig | None = None, *, use_fake_hardware: bool = False
) -> HandInterface:
    resolved = config or SharpaConfig(name="sharpa")
    if use_fake_hardware:
        return MockSharpaHand(resolved)
    return SharpaSdkHand(resolved)

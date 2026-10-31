"""Factory for real and fake Linker Hand devices."""

from __future__ import annotations

from litchi_hardware.core.interfaces import HandInterface

from .config import LinkerConfig
from .mock import MockLinkerHand
from .sdk_driver import LinkerSdkHand


def create_hand(
    config: LinkerConfig | None = None, *, use_fake_hardware: bool = False
) -> HandInterface:
    resolved = config or LinkerConfig(name="linker")
    if use_fake_hardware:
        return MockLinkerHand(resolved)
    return LinkerSdkHand(resolved)

"""Public Sharpa facade with lazy access to the bundled native SDK."""

from __future__ import annotations

from typing import Any

from .hardware.sharpa import (
    SHARPA_FINGERS,
    SHARPA_JOINT_LIMITS,
    SHARPA_JOINT_NAMES,
    SHARPA_WAVE_JOINT_SCHEMA,
    MockSharpaHand,
    SharpaConfig,
    SharpaFinger,
    SharpaSdkHand,
    SharpaTactileFrame,
    TactileImage,
    create_hand,
    tactile_channel,
)
from .hardware.sharpa.vendor import (
    load_sharpa_sdk,
    sharpa_include_dir,
    sharpa_library_dir,
    sharpa_sdk_root,
)

__all__ = [
    "MockSharpaHand",
    "SHARPA_FINGERS",
    "SHARPA_JOINT_LIMITS",
    "SHARPA_JOINT_NAMES",
    "SHARPA_WAVE_JOINT_SCHEMA",
    "SharpaConfig",
    "SharpaFinger",
    "SharpaSdkHand",
    "SharpaTactileFrame",
    "TactileImage",
    "create_hand",
    "load_sdk",
    "sdk_root",
    "include_dir",
    "library_dir",
    "tactile_channel",
]


load_sdk = load_sharpa_sdk
sdk_root = sharpa_sdk_root
include_dir = sharpa_include_dir
library_dir = sharpa_library_dir


def __getattr__(name: str) -> Any:
    """Proxy unknown attributes to the vendor SDK without loading it eagerly."""

    try:
        return getattr(load_sdk(), name)
    except AttributeError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc

"""Public Manus facade and bundled SDK locations."""

from .hardware.manus import (
    MANUS_SHARPA_25_SCHEMA,
    ManusBackend,
    ManusConfig,
    ManusNativeConfig,
    ManusNativeTracker,
    ManusZmqConfig,
    ManusZmqTracker,
    MockManusTracker,
    create_tracker,
)
from .hardware.manus._native import (
    load_native_bridge,
    native_bridge_include_dir,
    native_bridge_library,
)
from .hardware.manus.vendor import (
    load_manus_library,
    manus_calibration_dir,
    manus_client_executable,
    manus_include_dir,
    manus_library_dir,
    manus_sdk_root,
)

__all__ = [
    "ManusBackend",
    "ManusConfig",
    "ManusNativeConfig",
    "ManusNativeTracker",
    "ManusZmqConfig",
    "ManusZmqTracker",
    "MANUS_SHARPA_25_SCHEMA",
    "MockManusTracker",
    "bridge_library",
    "bridge_include_dir",
    "calibration_dir",
    "client_executable",
    "create_tracker",
    "include_dir",
    "library_dir",
    "load_bridge",
    "load_library",
    "sdk_root",
]

sdk_root = manus_sdk_root
include_dir = manus_include_dir
library_dir = manus_library_dir
bridge_library = native_bridge_library
bridge_include_dir = native_bridge_include_dir
client_executable = manus_client_executable
calibration_dir = manus_calibration_dir
load_library = load_manus_library
load_bridge = load_native_bridge

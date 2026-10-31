"""Factory for real and fake Manus trackers."""

from __future__ import annotations

from dataclasses import replace

from litchi_hardware.core.interfaces import HandTrackerInterface

from .config import (
    ManusBackend,
    ManusConfig,
    ManusNativeConfig,
    ManusZmqConfig,
)
from .mock import MockManusTracker
from .native_driver import ManusNativeTracker
from .vendor import manus_calibration_dir, manus_client_executable
from .zmq_driver import ManusZmqTracker


def create_tracker(
    config: ManusConfig | None = None,
    *,
    backend: ManusBackend | str | None = None,
    use_fake_hardware: bool = False,
) -> HandTrackerInterface:
    selected_backend = ManusBackend(backend) if backend is not None else None
    supplied = config or ManusConfig(name="manus")
    common = {
        "name": supplied.name,
        "connect_timeout_s": supplied.connect_timeout_s,
        "frame_timeout_s": supplied.frame_timeout_s,
        "frame_id": supplied.frame_id,
    }
    if isinstance(supplied, ManusNativeConfig):
        if selected_backend not in (None, ManusBackend.NATIVE):
            raise ValueError("ManusNativeConfig conflicts with backend='zmq'")
        resolved: ManusNativeConfig | ManusZmqConfig = supplied
        selected_backend = ManusBackend.NATIVE
    elif isinstance(supplied, ManusZmqConfig):
        if selected_backend not in (None, ManusBackend.ZMQ):
            raise ValueError("ManusZmqConfig conflicts with backend='native'")
        resolved = supplied
        selected_backend = ManusBackend.ZMQ
    elif selected_backend is ManusBackend.ZMQ:
        resolved = ManusZmqConfig(**common)
    else:
        resolved = ManusNativeConfig(**common)
        selected_backend = ManusBackend.NATIVE
    if use_fake_hardware:
        return MockManusTracker(resolved)
    if selected_backend is ManusBackend.NATIVE:
        return ManusNativeTracker(resolved)
    assert isinstance(resolved, ManusZmqConfig)
    if (
        resolved.manage_client
        and resolved.client_executable is None
        and manus_client_executable().is_file()
    ):
        resolved = replace(
            resolved,
            client_executable=str(manus_client_executable()),
            client_working_directory=(
                resolved.client_working_directory
                or str(manus_calibration_dir())
            ),
        )
    return ManusZmqTracker(resolved)

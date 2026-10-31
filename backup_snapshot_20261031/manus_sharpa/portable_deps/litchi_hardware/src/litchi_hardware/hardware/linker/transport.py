"""Integration transport helpers around the preserved vendor controllers."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import subprocess
import threading
import time
from pathlib import Path
from types import ModuleType
from typing import Any


class CanInterfaceManager:
    """Safe replacement for the legacy SDK's password-bearing CAN helper."""

    def __init__(self, load_yaml: Any = None) -> None:
        self.auto_setup = bool(getattr(load_yaml, "auto_setup_can", False))
        self.bitrate = int(getattr(load_yaml, "bitrate", 1_000_000))

    def open_can(self, can: str = "can0") -> None:
        if not self.auto_setup or self.is_can_up_sysfs(can):
            return
        commands = (
            ["sudo", "-n", "ip", "link", "set", can, "down"],
            [
                "sudo",
                "-n",
                "ip",
                "link",
                "set",
                can,
                "type",
                "can",
                "bitrate",
                str(self.bitrate),
            ],
            ["sudo", "-n", "ip", "link", "set", can, "up"],
        )
        for command in commands:
            subprocess.run(
                command,
                check=True,
                timeout=5,
                capture_output=True,
            )

    def is_can_up_sysfs(self, interface: str = "can0") -> bool:
        path = Path("/sys/class/net") / interface / "operstate"
        try:
            return path.read_text(encoding="utf-8").strip() == "up"
        except OSError:
            return False

    def close_can(self, can: str = "can0") -> bool:
        del can
        return False


def make_libcanbus_communication(sdk: ModuleType, library_path: Path) -> type[Any]:
    """Bind the O30I transport to a wheel-relative libcanbus runtime."""

    class BundledCANFDCommunication(sdk.CANFDCommunication):
        def initialize(self) -> bool:
            try:
                usb = ctypes.util.find_library("usb-1.0")
                if usb:
                    ctypes.CDLL(usb, mode=ctypes.RTLD_GLOBAL)
                self.canDLL = ctypes.CDLL(str(library_path))

                if self.canDLL.CAN_ScanDevice() <= 0:
                    return False
                result = self.canDLL.CAN_OpenDevice(self.canfd_device, self.channel)
                if result != sdk.STATUS_OK:
                    return False
                config = sdk.CanFD_Config(
                    1_000_000,
                    5_000_000,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0x04,
                    0x0,
                    0x1,
                )
                result = self.canDLL.CANFD_Init(
                    self.canfd_device, self.channel, ctypes.byref(config)
                )
                if result != sdk.STATUS_OK:
                    self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                    return False
                result = self.canDLL.CAN_SetFilter(self.canfd_device, self.channel, 0, 0, 0, 0, 1)
                if result != sdk.STATUS_OK:
                    self.canDLL.CAN_CloseDevice(self.canfd_device, self.channel)
                    return False
                self.is_connected = True
                self.running = True
                self._thread = threading.Thread(target=self._receive_loop, daemon=True)
                self._thread.start()
                time.sleep(0.05)
                return self._thread.is_alive()
            except (OSError, AttributeError):
                self.is_connected = False
                return False

    BundledCANFDCommunication.__name__ = "BundledCANFDCommunication"
    return BundledCANFDCommunication


def resolve_library_path(configured: str | None, bundled: Path | None) -> Path:
    candidate = Path(configured).expanduser().resolve() if configured is not None else bundled
    if candidate is None or not candidate.is_file():
        source = configured or os.environ.get("LINKER_HAND_LIBCANBUS") or "bundled runtime"
        raise FileNotFoundError(f"Linker Hand libcanbus not found: {source}")
    return candidate

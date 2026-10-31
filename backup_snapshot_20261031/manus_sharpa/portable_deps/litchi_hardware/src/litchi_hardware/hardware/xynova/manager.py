"""High-level Python dataclasses mirroring the vendor C++ config structs.

These are the ergonomic types users pass to :class:`xynova.Manager`. Each
dataclass has a ``to_c()`` method that produces a ctypes struct wired against
:mod:`litchi_hardware.hardware.xynova._ffi`. Keeping the two layers separate
means the API stays clean while matching the real vendor headers.
"""

from __future__ import annotations

import ctypes as C
from dataclasses import dataclass, field
from enum import IntEnum

from litchi_hardware.hardware.xynova import _ffi


class CommType(IntEnum):
    CAN = _ffi.COMM_TYPE_CAN


@dataclass
class CanConfig:
    iface: str = "can0"
    manager_can: bool = False
    bitrate: int = 1_000_000
    dbitrate: int = 5_000_000
    sample_point: float = 0.8
    dsample_point: float = 0.8

    def to_c(self) -> _ffi.CanConfigC:
        iface = self.iface.encode("utf-8")
        c = _ffi.CanConfigC(
            iface=iface,
            manager_can=self.manager_can,
            bitrate=self.bitrate,
            dbitrate=self.dbitrate,
            sample_point=self.sample_point,
            dsample_point=self.dsample_point,
        )
        c._keep = [iface]
        return c


@dataclass
class CommConfig:
    type: CommType = CommType.CAN
    sub_type: str = "ChuangXinCAN"
    can: CanConfig = field(default_factory=CanConfig)
    logger_name: str = ""

    def to_c(self) -> _ffi.CommConfigC:
        c = _ffi.CommConfigC()
        c.type = int(self.type)
        sub_type = self.sub_type.encode("utf-8")
        c.sub_type = sub_type
        can_c = self.can.to_c()
        c.can = can_c
        # The vendor wraps these ``const char*`` in ``std::string``; a NULL
        # pointer dereferences and crashes, so an empty name must stay non-NULL.
        logger_name = self.logger_name.encode("utf-8")
        c.logger_name = logger_name
        c._keep = [sub_type, logger_name, *can_c._keep]
        return c


@dataclass
class MasterConfig:
    name: str = "master"
    period_us: int = 1000
    cpu_index: int = -1
    comm: CommConfig = field(default_factory=CommConfig)

    def to_c(self) -> _ffi.MasterConfigC:
        c = _ffi.MasterConfigC()
        name = self.name.encode("utf-8")
        c.name = name
        comm_c = self.comm.to_c()
        c.comm = comm_c
        c.period_us = self.period_us
        c.cpu_index = self.cpu_index
        c._keep = [name, *comm_c._keep]
        return c


@dataclass
class ResponseTimeSetting:
    user_timeout_us: int = 50_000
    user_poll_interval_us: int = 1_000
    execute_timeout_us: int = 5_000
    execute_max_retry: int = 3
    execute_poll_interval_us: int = 10_000

    def to_c(self) -> _ffi.ResponseTimeSettingC:
        return _ffi.ResponseTimeSettingC(
            user_timeout_us=self.user_timeout_us,
            user_poll_interval_us=self.user_poll_interval_us,
            execute_timeout_us=self.execute_timeout_us,
            execute_max_retry=self.execute_max_retry,
            execute_poll_interval_us=self.execute_poll_interval_us,
        )


def _fast_defaults() -> ResponseTimeSetting:
    """Vendor's fast-profile numbers from docs §3.1.3."""
    return ResponseTimeSetting(
        user_timeout_us=5_000,
        user_poll_interval_us=100,
        execute_timeout_us=3_000,
        execute_max_retry=0,
        execute_poll_interval_us=3_000,
    )


@dataclass
class SlaveConfig:
    type: str = "DexterousHandSlave"
    id: int = 0xFFFFFFFF
    general_time_setting: ResponseTimeSetting = field(default_factory=ResponseTimeSetting)
    fast_time_setting: ResponseTimeSetting = field(default_factory=_fast_defaults)
    logger_name: str = ""

    def to_c(self) -> _ffi.SlaveConfigC:
        c = _ffi.SlaveConfigC()
        type_name = self.type.encode("utf-8")
        c.type = type_name
        c.id = self.id
        c.general_time_setting = self.general_time_setting.to_c()
        c.fast_time_setting = self.fast_time_setting.to_c()
        logger_name = self.logger_name.encode("utf-8")
        c.logger_name = logger_name
        c._keep = [type_name, logger_name]
        return c


@dataclass
class DeviceConfig:
    type: str = "DexterousHandDevice"
    name: str = "hand"
    master_name: str = "master"
    slaves: list[SlaveConfig] = field(default_factory=list)


class Manager:
    """Singleton wrapper around ``xynova::communication::Manager``.

    Vendor doc says: *"Manager owns all masters and devices."* We store the
    ctypes arrays as attributes so Python doesn't free them mid-call.
    """

    _instance: Manager | None = None

    def __new__(cls) -> Manager:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._keepalive: list[object] = []  # type: ignore[attr-defined]
        return cls._instance

    @classmethod
    def instance(cls) -> Manager:
        return cls()

    # ------------------------------------------------------------------
    def init(self, *, masters: list[MasterConfig], devices: list[DeviceConfig]) -> None:
        from litchi_hardware.hardware.xynova.errors import check

        # Build C-side master array
        master_cs = [m.to_c() for m in masters]
        c_masters = (_ffi.MasterConfigC * len(masters))(*master_cs)
        for mc in master_cs:
            self._keepalive.extend(mc._keep)

        # Build C-side device array — each device owns a slave sub-array
        c_devices_list: list[_ffi.DeviceConfigC] = []
        for dev in devices:
            slave_cs = [s.to_c() for s in dev.slaves]
            slave_array = (_ffi.SlaveConfigC * len(dev.slaves))(*slave_cs)
            self._keepalive.append(slave_array)
            for sc in slave_cs:
                self._keepalive.extend(sc._keep)
            c = _ffi.DeviceConfigC()
            dev_type = dev.type.encode("utf-8")
            dev_name = dev.name.encode("utf-8")
            master_name = dev.master_name.encode("utf-8")
            c.type = dev_type
            c.name = dev_name
            c.master_name = master_name
            c.slaves = C.cast(slave_array, C.POINTER(_ffi.SlaveConfigC))
            c.slave_count = len(dev.slaves)
            c.commands = None
            c.command_count = 0
            self._keepalive.extend([dev_type, dev_name, master_name])
            c_devices_list.append(c)
        c_devices = (_ffi.DeviceConfigC * len(devices))(*c_devices_list)

        # Hold references to prevent GC before/during the C call and afterwards
        self._keepalive.extend([c_masters, c_devices])

        cfg = _ffi.ManagerConfigC(
            masters=C.cast(c_masters, C.POINTER(_ffi.MasterConfigC)),
            master_count=len(masters),
            devices=C.cast(c_devices, C.POINTER(_ffi.DeviceConfigC)),
            device_count=len(devices),
        )
        self._keepalive.append(cfg)
        check(_ffi.lib.XynovaCommunicationManager_init(C.byref(cfg)), "Manager.init")

    def run(self, master_name: str) -> None:
        from litchi_hardware.hardware.xynova.errors import check

        check(
            _ffi.lib.XynovaCommunicationManager_run(master_name.encode("utf-8")),
            f"Manager.run({master_name!r})",
        )

    def stop(self, master_name: str) -> None:
        from litchi_hardware.hardware.xynova.errors import check

        check(
            _ffi.lib.XynovaCommunicationManager_stop(master_name.encode("utf-8")),
            f"Manager.stop({master_name!r})",
        )

    def get_device(self, cls, master_name: str, device_type: str, device_name: str):  # noqa: ANN001
        handle = _ffi.lib.XynovaCommunicationManager_getDevice(
            master_name.encode("utf-8"),
            device_type.encode("utf-8"),
            device_name.encode("utf-8"),
        )
        if not handle:
            return None
        return cls(handle)

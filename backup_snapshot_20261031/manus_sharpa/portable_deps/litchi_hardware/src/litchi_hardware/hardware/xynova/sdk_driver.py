"""Core HAL adapter over the Xynova SDK 2.x ctypes wrapper."""

from __future__ import annotations

import time
from typing import Any

from litchi_hardware.core.errors import (
    CommandRejectedError,
    ConnectionError,
    DependencyUnavailableError,
)
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, TactileFrame

from . import _ffi
from .base import XynovaHandBase
from .config import XynovaConfig
from .errors import XynovaError, XynovaSDKNotLoadedError
from .hand.dexterous_hand import ControlMode
from .hand.dexterous_hand import Device as NativeDevice
from .infra.log import LogLevel, LogManager
from .manager import (
    CanConfig,
    CommConfig,
    CommType,
    Manager,
    MasterConfig,
    SlaveConfig,
)
from .manager import (
    DeviceConfig as NativeDeviceConfig,
)
from .schema import XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA


class XynovaSdkHand(XynovaHandBase):
    """Adapt the vendor manager/device API to the shared hand interface."""

    def __init__(
        self,
        config: XynovaConfig,
        *,
        manager: Any | None = None,
        device_class: type = NativeDevice,
        log_manager: Any = LogManager,
    ) -> None:
        super().__init__(config)
        self._manager = manager
        self._manager_injected = manager is not None
        self._device_class = device_class
        self._log_manager = log_manager
        self._device: Any | None = None
        self._manager_initialized = False
        self._manager_running = False
        self._device_running = False
        self._last_position: tuple[float, ...] | None = None
        self._last_timestamp_s: float | None = None
        self._hardware_version: str | None = None
        self._software_version: str | None = None

    @property
    def transport_name(self) -> str:
        return "xynova-ctypes-sdk"

    def _native_configs(self) -> tuple[MasterConfig, NativeDeviceConfig]:
        logger_name = self.config.resolved_logger_name
        master = MasterConfig(
            name=self.config.master_name,
            period_us=self.config.master_period_us,
            cpu_index=self.config.master_cpu_index,
            comm=CommConfig(
                type=CommType.CAN,
                sub_type=self.config.can_driver,
                can=CanConfig(
                    iface=self.config.can_interface,
                    manager_can=self.config.manager_can,
                    bitrate=self.config.bitrate,
                    dbitrate=self.config.data_bitrate,
                    sample_point=self.config.sample_point,
                    dsample_point=self.config.data_sample_point,
                ),
                logger_name=logger_name,
            ),
        )
        slave = SlaveConfig(
            type=self.config.native_slave_type,
            id=self.config.device_id,
            logger_name=logger_name,
        )
        device = NativeDeviceConfig(
            type=self.config.native_device_type,
            name=self.config.resolved_device_name,
            master_name=master.name,
            slaves=[slave],
        )
        return master, device

    def _prepare_logging(self) -> None:
        levels = {
            "trace": LogLevel.TRACE,
            "debug": LogLevel.DEBUG,
            "info": LogLevel.INFO,
            "warn": LogLevel.WARN,
            "error": LogLevel.ERROR,
            "fatal": LogLevel.FATAL,
        }
        self._log_manager.init_global(levels[self.config.log_level.lower()])
        if not self._log_manager.objects_initialized():
            self._log_manager.init_default_objects()
        logger_name = self.config.resolved_logger_name
        if not self._log_manager.has_logger(logger_name):
            self._log_manager.create_logger(logger_name)

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()

        try:
            if not self._manager_injected and not _ffi.is_loaded():
                raise DependencyUnavailableError(
                    "Xynova SDK runtime is unavailable; install "
                    "litchi-hardware[xynova] for a supported platform or set XYNOVA_LIB"
                )
            self._prepare_logging()
            master, device_config = self._native_configs()
            if self._manager is None:
                self._manager = Manager.instance()
            if not self._manager_initialized:
                self._manager.init(masters=[master], devices=[device_config])
                self._manager_initialized = True
            self._manager.run(master.name)
            self._manager_running = True

            self._device = self._manager.get_device(
                self._device_class,
                master.name,
                device_config.type,
                device_config.name,
            )
            if self._device is None or not self._device.valid():
                raise ConnectionError(f"Xynova device {device_config.name!r} was not created")
            self._device.run()
            self._device_running = True
            self._device.set_joint_control_mode_uniform(ControlMode.ANGLE)
            if self.config.enable_tactile:
                self._device.enable_tactile_wrench(True)
            self._model = self._device.get_product_model() or self._model
            self._serial = self._device.get_product_sn() or None
            self._hardware_version = self._device.get_hardware_version() or None
            self._software_version = self._device.get_software_version() or None
            self._last_position = None
            self._last_timestamp_s = None
            self._lifecycle.connected()
        except Exception as exc:
            self._cleanup()
            self._lifecycle.failed()
            if isinstance(
                exc,
                (
                    CommandRejectedError,
                    ConnectionError,
                    DependencyUnavailableError,
                ),
            ):
                raise
            raise ConnectionError(f"failed to connect Xynova hand: {exc}") from exc

    def _cleanup(self) -> None:
        if self._device is not None and self._device_running:
            try:
                self._device.stop()
            except Exception:
                pass
            self._device_running = False
        if self._manager is not None and self._manager_running:
            try:
                self._manager.stop(self.config.master_name)
            except Exception:
                pass
            self._manager_running = False
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._cleanup()
        self._lifecycle.disconnected()

    def _radians_to_sdk(self, positions: tuple[float, ...]) -> list[float]:
        return [
            gain * value + offset
            for value, gain, offset in zip(
                positions,
                self.config.servo_gains,
                self.config.servo_offsets,
                strict=True,
            )
        ]

    def _sdk_to_radians(self, positions: list[float]) -> tuple[float, ...]:
        return tuple(
            (float(value) - offset) / gain
            for value, gain, offset in zip(
                positions,
                self.config.servo_gains,
                self.config.servo_offsets,
                strict=True,
            )
        )

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        if self._device is None or not self._device_running:
            raise ConnectionError("Xynova device is stopped or unavailable")
        sdk_positions = self._radians_to_sdk(positions)
        try:
            if self.config.async_control:
                self._device.set_joint_angle_async(sdk_positions)
            else:
                self._device.set_joint_angle(sdk_positions)
        except (XynovaError, XynovaSDKNotLoadedError) as exc:
            raise CommandRejectedError(f"Xynova joint position command failed: {exc}") from exc

    def read_joint_state(self) -> JointState:
        if not self.is_connected or self._device is None:
            raise ConnectionError("Xynova hand is not connected")
        try:
            sdk_positions, timestamp_us = self._device.get_joint_angle()
            efforts: tuple[float, ...] = ()
        except (XynovaError, XynovaSDKNotLoadedError) as exc:
            raise ConnectionError(f"failed to read Xynova joint state: {exc}") from exc
        if len(sdk_positions) != XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.size:
            raise ConnectionError(
                f"Xynova SDK returned {len(sdk_positions)} joint angles; "
                f"expected {XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.size}"
            )

        positions = self._sdk_to_radians(sdk_positions)
        timestamp_s = timestamp_us / 1_000_000 if timestamp_us > 0 else time.time()
        velocity: tuple[float, ...] = ()
        if (
            self._last_position is not None
            and self._last_timestamp_s is not None
            and timestamp_s > self._last_timestamp_s
        ):
            elapsed = timestamp_s - self._last_timestamp_s
            velocity = tuple(
                (current - previous) / elapsed
                for current, previous in zip(positions, self._last_position, strict=True)
            )
        self._last_position = positions
        self._last_timestamp_s = timestamp_s
        return JointState(
            XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.names,
            positions,
            velocity,
            efforts,
            timestamp_s=timestamp_s,
            schema=XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA,
        )

    def read_tactile(self, timeout_s: float | None = None) -> TactileFrame:
        del timeout_s
        if not self.is_connected or self._device is None:
            raise ConnectionError("Xynova hand is not connected")
        if not self.config.enable_tactile:
            raise ConnectionError("Xynova tactile feedback is disabled")
        try:
            wrenches, timestamp_us = self._device.get_tactile_wrenches()
        except (XynovaError, XynovaSDKNotLoadedError) as exc:
            raise ConnectionError(f"failed to read Xynova tactile feedback: {exc}") from exc
        if len(wrenches) != _ffi.TACTILE_NUM or any(
            len(wrench.values) != 6 for wrench in wrenches
        ):
            raise ConnectionError(
                "Xynova SDK returned an invalid tactile wrench frame; "
                f"expected {_ffi.TACTILE_NUM} six-axis values"
            )
        timestamp_s = timestamp_us / 1_000_000 if timestamp_us > 0 else time.time()
        return TactileFrame(
            values=tuple(
                value if axis < int(wrench.dimension) else 0.0
                for wrench in wrenches
                for axis, value in enumerate(wrench.values)
            ),
            shape=(_ffi.TACTILE_NUM, 6),
            timestamp_s=timestamp_s,
            frame_id="xynova_tactile_wrench",
        )

    def stop(self) -> None:
        if self._device is None or not self._device_running:
            return
        try:
            self._device.stop()
        except (XynovaError, XynovaSDKNotLoadedError) as exc:
            raise CommandRejectedError(f"failed to stop Xynova hand: {exc}") from exc
        self._device_running = False

from __future__ import annotations

import ctypes as C
import os
import subprocess
import sys
from typing import Any

import pytest

from litchi_hardware import xynova
from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.types import DeviceAction
from litchi_hardware.hardware.xynova import _ffi
from litchi_hardware.hardware.xynova.base import XynovaHandBase


def test_public_facade_exposes_wrapper() -> None:
    assert len(xynova.JOINT_NAMES) == 23
    assert xynova.TACTILE_NUM == 15
    assert xynova.TACTILE_FORCE_NUM == xynova.TACTILE_NUM
    assert xynova.JointID.FF1 == 1
    assert xynova.JointID.WR2 == 23
    assert xynova.ControlMode.SERVO is xynova.ControlMode.ANGLE
    assert xynova.XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.names == xynova.JOINT_NAMES
    assert xynova.XYNOVA_XZ_JOINT_SCHEMA is xynova.XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA
    assert issubclass(XynovaHandBase, HandInterface)


def test_fake_xynova_implements_core_hand_interface() -> None:
    config = xynova.XynovaConfig(
        name="fake",
        open_position=(0.0,) * 23,
        close_position=(0.5,) * 23,
    )
    hand = xynova.create_hand(config, use_fake_hardware=True)

    assert isinstance(hand, HandInterface)
    hand.connect()
    hand.send_action(DeviceAction({"joint_position": (0.25,) * 23}))
    assert hand.read_joint_state().position == (0.25,) * 23
    assert hand.get_observation().features["joint_state"].schema is (
        xynova.XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA
    )
    hand.open()
    assert hand.read_joint_state().position == (0.0,) * 23
    hand.close()
    assert hand.read_joint_state().position == (0.5,) * 23
    hand.disconnect()


def test_fake_xynova_validates_core_actions_and_presets() -> None:
    hand = xynova.create_hand(xynova.XynovaConfig(name="fake"), use_fake_hardware=True)
    hand.connect()
    with pytest.raises(CommandRejectedError, match="23"):
        hand.send_action(DeviceAction({"joint_position": (0.0,)}))
    with pytest.raises(CommandRejectedError, match="not configured"):
        hand.open()
    hand.stop()
    with pytest.raises(ConnectionError, match="stopped"):
        hand.send_action(DeviceAction({"joint_position": (0.0,) * 23}))
    hand.disconnect()


def test_xynova_config_rejects_invalid_calibration() -> None:
    with pytest.raises(ValueError, match="23"):
        xynova.XynovaConfig(name="invalid", servo_gains=(1.0,))
    with pytest.raises(ValueError, match="zero"):
        xynova.XynovaConfig(name="invalid", servo_gains=(0.0,) * 23)
    with pytest.raises(ValueError, match="not supported"):
        xynova.XynovaConfig(name="invalid", enable_joint_torque=True)


class _FakeLogManager:
    initialized = False
    loggers: set[str] = set()

    @classmethod
    def init_global(cls, level: object) -> None:
        del level

    @classmethod
    def objects_initialized(cls) -> bool:
        return cls.initialized

    @classmethod
    def init_default_objects(cls) -> None:
        cls.initialized = True

    @classmethod
    def has_logger(cls, name: str) -> bool:
        return name in cls.loggers

    @classmethod
    def create_logger(cls, name: str) -> None:
        cls.loggers.add(name)


class _FakeNativeDevice:
    def __init__(self) -> None:
        self.position = [1.0] * 23
        self.tactile = [
            xynova.TactileWrench(
                timestamp_us=1_000_000,
                region=xynova.TactileID(index + 1),
                dimension=(
                    xynova.WrenchDimension.THREE_D
                    if index == 1
                    else xynova.WrenchDimension.SIX_D
                ),
                values=tuple(float(index) + axis / 10 for axis in range(6)),
            )
            for index in range(15)
        ]
        self.timestamp_us = 1_000_000
        self.calls: list[str] = []
        self.running = False

    def valid(self) -> bool:
        return True

    def run(self) -> None:
        self.running = True
        self.calls.append("run")

    def stop(self) -> None:
        self.running = False
        self.calls.append("stop")

    def close(self) -> None:
        self.calls.append("close")

    def set_joint_control_mode_uniform(self, mode: object) -> None:
        self.calls.append(f"mode:{int(mode)}")

    def enable_tactile_wrench(self, enable: bool) -> None:
        self.calls.append(f"tactile:{enable}")

    def set_joint_angle_async(self, values: list[float]) -> None:
        self.position = values
        self.calls.append("angle_async")

    def set_joint_angle(self, values: list[float]) -> None:
        self.position = values
        self.calls.append("angle")

    def get_joint_angle(self) -> tuple[list[float], int]:
        return self.position, self.timestamp_us

    def get_tactile_wrenches(self) -> tuple[list[xynova.TactileWrench], int]:
        return self.tactile, self.timestamp_us

    def get_product_model(self) -> str:
        return "Dexterous Hand Test"

    def get_product_sn(self) -> str:
        return "XY-TEST"

    def get_hardware_version(self) -> str:
        return "HW1"

    def get_software_version(self) -> str:
        return "SW1"


class _FakeNativeManager:
    def __init__(self, device: _FakeNativeDevice) -> None:
        self.device = device
        self.calls: list[str] = []
        self.masters: list[Any] = []
        self.devices: list[Any] = []

    def init(self, *, masters: list[Any], devices: list[Any]) -> None:
        self.calls.append("init")
        self.masters = masters
        self.devices = devices

    def run(self, master_name: str) -> None:
        self.calls.append(f"run:{master_name}")

    def stop(self, master_name: str) -> None:
        self.calls.append(f"stop:{master_name}")

    def get_device(
        self,
        device_class: type,
        master_name: str,
        device_type: str,
        device_name: str,
    ) -> _FakeNativeDevice:
        del device_class, master_name, device_type, device_name
        return self.device


def test_sdk_adapter_maps_core_radians_to_vendor_angle_values() -> None:
    device = _FakeNativeDevice()
    manager = _FakeNativeManager(device)
    config = xynova.XynovaConfig(
        name="hal",
        can_interface="0.0",
        device_id=2,
        servo_gains=(2.0,) * 23,
        servo_offsets=(1.0,) * 23,
    )
    hand = xynova.XynovaSdkHand(
        config,
        manager=manager,
        device_class=object,
        log_manager=_FakeLogManager,
    )

    hand.connect()
    assert hand.info.serial == "XY-TEST"
    assert hand.info.model == "Dexterous Hand Test"
    assert manager.devices[0].slaves[0].id == 2
    assert manager.masters[0].comm.can.iface == "0.0"

    hand.send_action(DeviceAction({"joint_position": (0.5,) * 23}))
    assert device.position == [2.0] * 23
    state = hand.read_joint_state()
    assert state.position == (0.5,) * 23
    assert state.schema is xynova.XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA
    hand.disconnect()
    assert manager.calls[-1] == "stop:xynova_master"
    assert device.calls[-1] == "close"


def test_tactile_flag_enables_wrench_feedback() -> None:
    device = _FakeNativeDevice()
    manager = _FakeNativeManager(device)
    config = xynova.XynovaConfig(
        name="feedback",
        enable_tactile=True,
    )
    hand = xynova.XynovaSdkHand(
        config,
        manager=manager,
        device_class=object,
        log_manager=_FakeLogManager,
    )

    hand.connect()
    state = hand.read_joint_state()
    tactile = hand.read_tactile()

    assert "tactile:True" in device.calls
    assert state.effort == ()
    assert tactile.shape == (15, 6)
    assert tactile.values[:6] == (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)
    assert tactile.values[6:12] == (1.0, 1.1, 1.2, 0.0, 0.0, 0.0)
    assert tactile.timestamp_s == 1.0
    hand.disconnect()


def test_mock_feedback_flags_control_optional_feedback() -> None:
    disabled = xynova.create_hand(
        xynova.XynovaConfig(name="disabled"), use_fake_hardware=True
    )
    disabled.connect()
    assert disabled.read_joint_state().effort == ()
    with pytest.raises(ConnectionError, match="disabled"):
        disabled.read_tactile()
    disabled.disconnect()

    enabled = xynova.create_hand(
        xynova.XynovaConfig(
            name="enabled",
            enable_tactile=True,
        ),
        use_fake_hardware=True,
    )
    enabled.connect()
    assert enabled.read_joint_state().effort == ()
    assert enabled.read_tactile().shape == (15, 6)
    enabled.disconnect()


def test_declared_native_api_and_struct_layout() -> None:
    symbols = _ffi.declared_symbols()
    assert len(symbols) > 30
    assert "XynovaCommunicationManager_init" in symbols
    assert "XynovaDexterousHand_setJointAngle_async" in symbols
    assert "XynovaDexterousHand_getJointAngle" in symbols

    assert C.sizeof(C.c_void_p) == 8
    assert C.sizeof(_ffi.CanConfigC) == 32
    assert C.sizeof(_ffi.CommConfigC) == 56
    assert C.sizeof(_ffi.MasterConfigC) == 80
    assert C.sizeof(_ffi.SlaveConfigC) == 64
    assert C.sizeof(_ffi.DeviceConfigC) == 56
    assert C.sizeof(_ffi.ManagerConfigC) == 32
    assert C.sizeof(_ffi.XynovaValueC) == 8
    assert C.sizeof(_ffi.XynovaTypedValueC) == 16
    assert C.sizeof(_ffi.XynovaIdValueC) == 24
    assert C.sizeof(_ffi.JointAngleControlC) == 552
    assert C.sizeof(_ffi.DexterousHandErrorStateC) == 568
    assert C.sizeof(_ffi.TactileWrenchStatesC) == 1080
    assert C.sizeof(_ffi.TactileTaxelStatesC) == 16
    assert C.sizeof(_ffi.RangeStatesC) == 16


def test_angle_command_payload_uses_sdk_ids_and_float_radians() -> None:
    values = [index / 10 for index in range(23)]
    payload = xynova.Device._angle_payload(values)

    assert [item.id for item in payload.values] == list(range(1, 24))
    assert all(item.data.type == _ffi.DATA_TYPE_FLOAT for item in payload.values)
    assert [item.data.value.f32 for item in payload.values] == pytest.approx(values)


def test_bundled_runtime_metadata_and_symbols() -> None:
    bundled = xynova.bundled_library_path()
    assert bundled is not None
    assert bundled.is_file()
    assert xynova.is_sdk_loaded()
    assert xynova.sdk_build_info()["version"] == "2.0.0"

    missing = [name for name in _ffi.declared_symbols() if not hasattr(_ffi.lib, name)]
    assert not missing


def test_missing_override_degrades_at_first_sdk_call() -> None:
    env = os.environ.copy()
    env["XYNOVA_LIB"] = "/definitely/missing/libxynova.so"
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "from litchi_hardware.hardware.xynova import _ffi; "
                "from litchi_hardware.hardware.xynova.errors import "
                "XynovaSDKNotLoadedError; "
                "assert not _ffi.is_loaded(); "
                "\ntry: _ffi.lib.XynovaDexterousHand_run\n"
                "except XynovaSDKNotLoadedError: pass\n"
                "else: raise AssertionError('stub did not raise')"
            ),
        ],
        check=False,
        capture_output=True,
        env=env,
        text=True,
    )
    assert result.returncode == 0, result.stderr

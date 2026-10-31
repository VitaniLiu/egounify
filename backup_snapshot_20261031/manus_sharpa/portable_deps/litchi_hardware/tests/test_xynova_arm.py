from __future__ import annotations

import ctypes
from types import SimpleNamespace
from typing import Any

import pytest

from litchi_hardware import ArmInterface, xynova
from litchi_hardware.core.errors import CommandRejectedError
from litchi_hardware.core.types import DeviceAction, Pose


def test_public_facade_exposes_xynova_arm_sdk_assets() -> None:
    assert xynova.arm_include_dir().joinpath("xynova", "robot.h").is_file()
    assert xynova.arm_sdk_library().is_file()
    assert xynova.arm_bridge_library().is_file()
    assert xynova.arm_joint_schema(xynova.XynovaArmModel.XYNOVA_AR).size == 7

    library = ctypes.CDLL(str(xynova.arm_bridge_library()))
    library.litchi_xynova_arm_bridge_abi_version.argtypes = []
    library.litchi_xynova_arm_bridge_abi_version.restype = ctypes.c_uint32
    assert library.litchi_xynova_arm_bridge_abi_version() == 1


@pytest.mark.parametrize(
    ("model", "joint_count"),
    [
        (xynova.XynovaArmModel.XYNOVA, 6),
        (xynova.XynovaArmModel.XYNOVA_CR5, 5),
        (xynova.XynovaArmModel.XYNOVA_ER, 7),
        (xynova.XynovaArmModel.XYNOVA_AR, 7),
        (xynova.XynovaArmModel.STANDARD, 6),
        (xynova.XynovaArmModel.PCB3, 3),
        (xynova.XynovaArmModel.PCB4, 4),
    ],
)
def test_mock_arm_implements_core_interface(
    model: xynova.XynovaArmModel, joint_count: int
) -> None:
    arm = xynova.create_arm(
        xynova.XynovaArmConfig(name="fake", model=model),
        use_fake_hardware=True,
    )
    assert isinstance(arm, ArmInterface)
    arm.connect()
    with pytest.raises(CommandRejectedError, match="not powered"):
        arm.send_action(DeviceAction({"joint_position": (0.1,) * joint_count}))

    arm.power_on()  # type: ignore[attr-defined]
    arm.send_action(DeviceAction({"joint_position": (0.1,) * joint_count}))
    state = arm.read_joint_state()
    assert state.position == (0.1,) * joint_count
    assert state.velocity == (0.0,) * joint_count
    assert state.effort == (0.0,) * joint_count
    assert state.schema is xynova.arm_joint_schema(model)
    assert arm.read_tcp_pose() == Pose.identity()
    if model in xynova.COLLABORATIVE_MODELS:
        assert arm.read_wrench() == (0.0,) * 6
    arm.stop()
    arm.disconnect()


def test_arm_config_and_actions_are_validated() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        xynova.XynovaArmConfig(name="bad", model="not-a-model")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="default_speed"):
        xynova.XynovaArmConfig(name="bad", default_speed=0.0)

    arm = xynova.create_arm(
        xynova.XynovaArmConfig(name="fake", auto_power_on=True),
        use_fake_hardware=True,
    )
    arm.connect()
    with pytest.raises(CommandRejectedError, match="7"):
        arm.send_action(DeviceAction({"joint_position": (0.0,)}))
    with pytest.raises(CommandRejectedError, match="requires"):
        arm.send_action(DeviceAction({"velocity": (0.0,) * 7}))
    arm.disconnect()


class _FakeNativeArm:
    instances: list[_FakeNativeArm] = []

    def __init__(self, model_code: int, remote_ip: str, local_ip: str) -> None:
        self.model_code = model_code
        self.remote_ip = remote_ip
        self.local_ip = local_ip
        self.calls: list[Any] = []
        self.position = (0.0,) * 7
        self.info = SimpleNamespace(
            joint_count=7,
            id="ARM-7-TEST",
            mac="00:11:22:33:44:55",
            controller_version="3.2.1",
            model="Xynova AR Test",
            sdk_version="0.7.1",
        )
        self.instances.append(self)

    def read_state(self) -> tuple[tuple[float, ...], ...]:
        return self.position, (0.2,) * 7, (0.3,) * 7

    def read_tcp_pose(self) -> tuple[float, ...]:
        return (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)

    def read_wrench(self) -> tuple[float, ...]:
        return (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def set_power(self, enabled: bool) -> None:
        self.calls.append(("power", enabled))

    def command_joint_position(
        self, positions: tuple[float, ...], speed: float, zone: float
    ) -> None:
        self.position = positions
        self.calls.append(("position", speed, zone))

    def stop(self) -> None:
        self.calls.append("stop")

    def close(self) -> None:
        self.calls.append("close")


def test_sdk_arm_maps_native_state_identity_and_commands() -> None:
    config = xynova.XynovaArmConfig(
        name="arm",
        remote_ip="192.168.1.160",
        local_ip="192.168.1.133",
        model=xynova.XynovaArmModel.XYNOVA_AR,
        default_speed=25.0,
        default_zone=0.0,
        auto_power_on=True,
        power_off_on_disconnect=True,
    )
    arm = xynova.XynovaSdkArm(config, native_factory=_FakeNativeArm)
    arm.connect()
    native = _FakeNativeArm.instances[-1]

    assert native.model_code == 3
    assert arm.info.serial == "ARM-7-TEST"
    assert arm.info.model == "Xynova AR Test"
    assert native.calls == [("power", True)]
    state = arm.read_joint_state()
    assert state.velocity == (0.2,) * 7
    assert state.effort == (0.3,) * 7
    assert arm.read_tcp_pose() == Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert arm.read_wrench() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    arm.send_action(DeviceAction({"joint_position": (0.5,) * 7}))
    assert native.position == (0.5,) * 7
    assert native.calls[-1] == ("position", 25.0, 0.0)
    arm.stop()
    arm.disconnect()
    assert native.calls[-3:] == ["stop", ("power", False), "close"]

from __future__ import annotations

from typing import Any

import pytest

from litchi_hardware import DeviceAction, linker
from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import HandSide
from litchi_hardware.hardware.linker.base import LinkerHandBase


def test_public_facade_exposes_models_schemas_and_vendor_assets() -> None:
    assert issubclass(LinkerHandBase, HandInterface)
    assert linker.LinkerModel.G20.value == "G20"
    assert linker.LinkerModel.O30I.value == "O30I"
    assert linker.LINKER_G20_JOINT_SCHEMA.names == linker.G20_JOINT_NAMES
    assert linker.LINKER_O30I_JOINT_SCHEMA.names == linker.O30I_JOINT_NAMES
    assert len(linker.G20_JOINT_NAMES) == 20
    assert len(linker.O30I_JOINT_NAMES) == 20
    assert linker.G20_JOINT_NAMES != linker.O30I_JOINT_NAMES
    assert linker.protocol_manual().is_file()
    assert (linker.legacy_sdk_root() / "linker_hand_api.py").is_file()
    assert linker.bundled_libcanbus_path().is_file()
    assert (linker.sdk_root() / "manifest.json").is_file()


@pytest.mark.parametrize("model", list(linker.LinkerModel))
def test_fake_linker_implements_core_hand_interface(
    model: linker.LinkerModel,
) -> None:
    config = linker.LinkerConfig(
        name="fake",
        model=model,
        side=HandSide.LEFT,
        enable_tactile=True,
    )
    hand = linker.create_hand(config, use_fake_hardware=True)

    assert isinstance(hand, HandInterface)
    hand.connect()
    assert hand.info.vendor == "Linker Hand"
    assert hand.info.model == model.value
    hand.send_action(DeviceAction({"joint_position": (0.25,) * 20}))
    assert hand.read_joint_state().position == (0.25,) * 20
    hand.close()
    assert hand.read_joint_state().position == (1.0,) * 20
    hand.open()
    assert hand.read_joint_state().position == (0.0,) * 20
    assert hand.read_tactile("index").shape == (2, 2)
    hand.stop()
    with pytest.raises(ConnectionError, match="stopped"):
        hand.send_action(DeviceAction({"joint_position": (0.5,) * 20}))
    hand.disconnect()


def test_linker_config_validates_transport_calibration_and_speed() -> None:
    with pytest.raises(ValueError, match="socketcan"):
        linker.LinkerConfig(
            name="g20",
            model="G20",
            transport="libcanbus",
        )
    with pytest.raises(ValueError, match="20"):
        linker.LinkerConfig(name="bad", position_gains=(1.0,))
    with pytest.raises(ValueError, match="outside"):
        linker.LinkerConfig(name="bad", position_gains=(256.0,) * 20)
    with pytest.raises(ValueError, match="20 vendor"):
        linker.LinkerConfig(name="o30", initial_speed=(100,) * 5)
    with pytest.raises(ValueError, match="5 vendor"):
        linker.LinkerConfig(
            name="g20",
            model="G20",
            initial_speed=(100,) * 20,
        )


def test_fake_linker_rejects_invalid_actions() -> None:
    hand = linker.create_hand(linker.LinkerConfig(name="fake"), use_fake_hardware=True)
    with pytest.raises(ConnectionError, match="not connected"):
        hand.send_action(DeviceAction({"joint_position": (0.0,) * 20}))
    hand.connect()
    with pytest.raises(CommandRejectedError, match="20"):
        hand.send_action(DeviceAction({"joint_position": (0.0,)}))
    with pytest.raises(CommandRejectedError, match="limits"):
        hand.send_action(DeviceAction({"joint_position": (2.0,) * 20}))


class _FakeO30IController:
    def __init__(self) -> None:
        self.is_connected = True
        self.positions = [51] * 20
        self.speed: list[int] | None = None
        self.enabled: list[int] | None = None
        self.setup_called = False
        self.closed = False

    def get_device_info(self) -> dict[str, str]:
        return {
            "设备唯一标识": "O30-TEST",
            "控制板版本": "O30-FW",
        }

    def setup(self) -> bool:
        self.setup_called = True
        return True

    def set_target_velocity(self, values: list[int]) -> bool:
        self.speed = values
        return True

    def set_target_position(self, values: list[int]) -> bool:
        self.positions = values
        return True

    def get_current_position(self) -> list[int]:
        return self.positions

    def read_tactile_matrix(self, finger: int) -> list[list[int]]:
        return [[finger, 2], [3, 4]]

    def set_joint_enable(self, values: list[int]) -> bool:
        self.enabled = values
        return True

    def close(self) -> None:
        self.closed = True


class _FakeG20Controller:
    def __init__(self) -> None:
        self.positions = [64] * 20
        self.speed: list[int] | None = None
        self.closed = False

    def get_serial_number(self) -> str:
        return "G20-TEST"

    def get_version(self) -> str:
        return "G20-FW"

    def set_speed(self, values: list[int]) -> None:
        self.speed = values

    def set_joint_positions(self, values: list[int]) -> None:
        self.positions = values

    def get_current_status(self) -> list[int]:
        return self.positions

    def get_index_matrix_touch(self) -> list[list[int]]:
        return [[1, 2, 3], [4, 5, 6]]

    def close_can_interface(self) -> None:
        self.closed = True


@pytest.mark.parametrize(
    ("model", "controller_type", "expected_serial"),
    [
        (linker.LinkerModel.O30I, _FakeO30IController, "O30-TEST"),
        (linker.LinkerModel.G20, _FakeG20Controller, "G20-TEST"),
    ],
)
def test_sdk_adapter_converts_radians_and_preserves_model_schema(
    model: linker.LinkerModel,
    controller_type: type[Any],
    expected_serial: str,
) -> None:
    controller = controller_type()
    speed = (200,) * (20 if model is linker.LinkerModel.O30I else 5)
    config = linker.LinkerConfig(
        name="real",
        model=model,
        position_gains=(100.0,) * 20,
        position_offsets=(10.0,) * 20,
        initial_speed=speed,
        enable_tactile=True,
    )
    hand = linker.LinkerSdkHand(config, controller_factory=lambda _config: controller)

    hand.connect()
    assert hand.connection_state is ConnectionState.CONNECTED
    assert hand.info.serial == expected_serial
    assert hand.info.metadata["firmware_version"].endswith("-FW")
    assert controller.speed == list(speed)
    hand.send_action(DeviceAction({"joint_position": (0.5,) * 20}))
    assert controller.positions == [60] * 20
    state = hand.read_joint_state()
    assert state.position == (0.5,) * 20
    assert state.schema is linker.schema_for_model(model)
    if model is linker.LinkerModel.O30I:
        assert hand.read_tactile("middle").shape == (2, 2)
        hand.stop()
        assert controller.enabled == [0] * 20
    else:
        assert hand.read_tactile("index").shape == (2, 3)
    hand.disconnect()
    assert controller.closed


def test_sdk_adapter_translates_missing_identity_to_failed_lifecycle() -> None:
    controller = _FakeO30IController()
    controller.get_device_info = lambda: {"设备唯一标识": ""}
    hand = linker.LinkerSdkHand(
        linker.LinkerConfig(name="missing"),
        controller_factory=lambda _config: controller,
    )
    with pytest.raises(ConnectionError, match="identity"):
        hand.connect()
    assert hand.connection_state is ConnectionState.FAILED
    assert controller.closed

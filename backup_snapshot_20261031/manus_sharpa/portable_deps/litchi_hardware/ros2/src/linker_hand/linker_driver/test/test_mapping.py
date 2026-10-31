import pytest
from linker_driver.config_loader import build_hand_config
from linker_driver.joint_mapping import JointMapping, UnknownJointError

from litchi_hardware.hardware.linker import (
    G20_JOINT_NAMES,
    O30I_JOINT_NAMES,
)


def test_o30_mapping_reorders_prefixed_joint_names() -> None:
    mapping = JointMapping(O30I_JOINT_NAMES, "right")
    names = [f"right_{name}" for name in reversed(O30I_JOINT_NAMES)]
    values = list(range(20))
    result = mapping.ros_positions_to_core(names, values)
    assert result == tuple(reversed(values))


def test_mapping_validates_complete_joint_state() -> None:
    mapping = JointMapping(G20_JOINT_NAMES, "left")
    with pytest.raises(ValueError, match="all 20"):
        mapping.ros_positions_to_core(["thumb_root1"], [0.0])
    with pytest.raises(UnknownJointError):
        mapping.ros_positions_to_core([*G20_JOINT_NAMES[:-1], "unknown"], [0.0] * 20)


def test_config_loader_selects_model_schema_and_calibration() -> None:
    config = build_hand_config(
        {
            "model": "o30i",
            "side": "left",
            "joint_calibration": {"thumb_roll": {"gain": 200.0, "offset": 10.0}},
        }
    )
    assert config.model.value == "O30I"
    assert config.side.value == "left"
    assert config.position_gains[0] == 200.0
    assert config.position_offsets[0] == 10.0
    assert config.position_gains[1:] == (255.0,) * 19

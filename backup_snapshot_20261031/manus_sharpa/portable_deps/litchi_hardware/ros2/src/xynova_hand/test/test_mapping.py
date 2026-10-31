from __future__ import annotations

import math

import pytest
from xynova_hand.config_loader import build_hand_config
from xynova_hand.joint_mapping import (
    JointMapping,
    UnknownJointError,
    load_calibration_from_dict,
)


def test_build_hand_config_uses_core_adapter_and_safe_logger() -> None:
    config = build_hand_config({})

    assert config.logger_name == "xynova_hand"
    assert config.master_name == "xynova_master"
    assert config.native_device_name == "dexterous_hand"
    assert config.native_device_type == "DexterousHandDevice"
    assert config.native_slave_type == "DexterousHandSlave"
    assert config.device_id == 0xFFFFFFFF


def test_build_hand_config_maps_optional_tactile_flag() -> None:
    config = build_hand_config({"enable_tactile": True})

    assert config.enable_tactile


def test_partial_command_maps_by_joint_name() -> None:
    mapping = JointMapping()
    result = mapping.ros_positions_to_core(["WR2", "FF1"], [0.25, 1.0])

    assert result[0] == 1.0
    assert result[-1] == 0.25
    assert math.isnan(result[1])


def test_unnamed_full_command_uses_core_order() -> None:
    mapping = JointMapping()
    result = mapping.ros_positions_to_core([], [0.1] * 23)

    assert result == [0.1] * 23


def test_calibration_is_transferred_to_hal_config() -> None:
    config = build_hand_config({"joint_calibration": {"TH1": {"gain": 2.0, "offset": 1.0}}})

    assert config.servo_gains[17] == 2.0
    assert config.servo_offsets[17] == 1.0


def test_unknown_calibration_joint_is_rejected() -> None:
    with pytest.raises(UnknownJointError):
        load_calibration_from_dict({"NOT_A_JOINT": {"gain": 1.0}})

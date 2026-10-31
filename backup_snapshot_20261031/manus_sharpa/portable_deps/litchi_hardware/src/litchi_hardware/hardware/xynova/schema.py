"""Xynova SDK 2.x Dexterous Hand joint ordering exposed through the HAL."""

from litchi_hardware.core.types import JointSchema

from .hand.dexterous_hand import JOINT_NAMES

XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA = JointSchema(
    id="xynova_dexterous_hand_23_v2",
    names=JOINT_NAMES,
    unit="rad",
)

XYNOVA_JOINT_NAMES = XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.names

# Backward-compatible name retained for code importing the 1.x schema symbol.
XYNOVA_XZ_JOINT_SCHEMA = XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA

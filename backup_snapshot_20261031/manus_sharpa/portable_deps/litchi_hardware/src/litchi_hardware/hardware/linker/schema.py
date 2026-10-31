"""Joint ordering and limits for supported Linker Hand models."""

from __future__ import annotations

from litchi_hardware.core.types import JointSchema

G20_JOINT_NAMES: tuple[str, ...] = (
    "thumb_root1",
    "index_root1",
    "middle_root1",
    "ring_root1",
    "little_root1",
    "thumb_yaw",
    "index_yaw",
    "middle_yaw",
    "ring_yaw",
    "little_yaw",
    "thumb_roll",
    "reserved_1",
    "reserved_2",
    "reserved_3",
    "reserved_4",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip",
)

O30I_JOINT_NAMES: tuple[str, ...] = (
    "thumb_roll",
    "thumb_yaw",
    "index_yaw",
    "middle_yaw",
    "ring_yaw",
    "little_yaw",
    "thumb_root1",
    "index_root1",
    "middle_root1",
    "ring_root1",
    "little_root1",
    "index_root2",
    "middle_root2",
    "ring_root2",
    "little_root2",
    "thumb_tip",
    "index_tip",
    "middle_tip",
    "ring_tip",
    "little_tip",
)

_MINIMUM = (0.0,) * 20
_MAXIMUM = (1.0,) * 20

LINKER_G20_JOINT_SCHEMA = JointSchema(
    id="linker_g20_20_v1",
    names=G20_JOINT_NAMES,
    unit="rad",
    minimum=_MINIMUM,
    maximum=_MAXIMUM,
)

LINKER_O30I_JOINT_SCHEMA = JointSchema(
    id="linker_o30i_20_v1",
    names=O30I_JOINT_NAMES,
    unit="rad",
    minimum=_MINIMUM,
    maximum=_MAXIMUM,
)

LINKER_OPEN_POSITION: tuple[float, ...] = _MINIMUM
LINKER_CLOSE_POSITION: tuple[float, ...] = _MAXIMUM

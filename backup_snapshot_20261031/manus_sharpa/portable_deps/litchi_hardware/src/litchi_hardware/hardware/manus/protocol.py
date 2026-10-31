"""Decoder for the MocapKeypoints protobuf emitted by the Sharpa Manus client."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Any

from litchi_hardware.core.errors import DependencyUnavailableError, ProtocolError
from litchi_hardware.core.types import Pose


@dataclass(frozen=True, slots=True)
class MocapPacket:
    timestamp_s: float
    sequence: int | None
    left: tuple[Pose, ...]
    right: tuple[Pose, ...]


def _add_field(
    message: Any,
    name: str,
    number: int,
    field_type: int,
    *,
    type_name: str = "",
    repeated: bool = False,
) -> None:
    field = message.field.add()
    field.name = name
    field.number = number
    field.type = field_type
    field.label = 3 if repeated else 1
    if type_name:
        field.type_name = type_name


@lru_cache(maxsize=1)
def _message_class() -> type[Any]:
    try:
        from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
    except ImportError as exc:
        raise DependencyUnavailableError(
            "Manus protocol decoding requires protobuf; install litchi-hardware[manus]"
        ) from exc

    file_descriptor = descriptor_pb2.FileDescriptorProto()
    file_descriptor.name = "litchi_manus_mocap.proto"
    file_descriptor.package = "proto"
    file_descriptor.syntax = "proto3"
    scalar = descriptor_pb2.FieldDescriptorProto

    stamp = file_descriptor.message_type.add()
    stamp.name = "Stamp"
    _add_field(stamp, "sec", 1, scalar.TYPE_INT32)
    _add_field(stamp, "nanosec", 2, scalar.TYPE_INT32)

    header = file_descriptor.message_type.add()
    header.name = "Header"
    _add_field(header, "stamp", 1, scalar.TYPE_MESSAGE, type_name=".proto.Stamp")
    _add_field(header, "frame_id", 2, scalar.TYPE_STRING)

    point = file_descriptor.message_type.add()
    point.name = "Point"
    for index, name in enumerate(("x", "y", "z"), start=1):
        _add_field(point, name, index, scalar.TYPE_FLOAT)

    quaternion = file_descriptor.message_type.add()
    quaternion.name = "Quaternion"
    for index, name in enumerate(("w", "x", "y", "z"), start=1):
        _add_field(quaternion, name, index, scalar.TYPE_FLOAT)

    pose = file_descriptor.message_type.add()
    pose.name = "Pose"
    _add_field(pose, "position", 1, scalar.TYPE_MESSAGE, type_name=".proto.Point")
    _add_field(
        pose, "orientation", 2, scalar.TYPE_MESSAGE, type_name=".proto.Quaternion"
    )

    mocap = file_descriptor.message_type.add()
    mocap.name = "MocapKeypoints"
    _add_field(mocap, "header", 1, scalar.TYPE_MESSAGE, type_name=".proto.Header")
    _add_field(
        mocap,
        "left_mocap_pose",
        2,
        scalar.TYPE_MESSAGE,
        type_name=".proto.Pose",
        repeated=True,
    )
    _add_field(
        mocap,
        "right_mocap_pose",
        3,
        scalar.TYPE_MESSAGE,
        type_name=".proto.Pose",
        repeated=True,
    )

    pool = descriptor_pool.DescriptorPool()
    pool.Add(file_descriptor)
    descriptor = pool.FindMessageTypeByName("proto.MocapKeypoints")
    if hasattr(message_factory, "GetMessageClass"):
        return message_factory.GetMessageClass(descriptor)
    return message_factory.MessageFactory(pool).GetPrototype(descriptor)


def _decode_poses(messages: Any) -> tuple[Pose, ...]:
    result = []
    for item in messages:
        result.append(
            Pose(
                (item.position.x, item.position.y, item.position.z),
                (
                    item.orientation.x,
                    item.orientation.y,
                    item.orientation.z,
                    item.orientation.w,
                ),
            )
        )
    return tuple(result)


def decode_mocap_keypoints(payload: bytes) -> MocapPacket:
    """Decode the vendor's ``MocapKeypoints`` wire format."""

    message = _message_class()()
    try:
        message.ParseFromString(payload)
        timestamp_s = float(message.header.stamp.sec) + (
            float(message.header.stamp.nanosec) * 1e-9
        )
        sequence = int(message.header.frame_id)
    except (ValueError, TypeError) as exc:
        raise ProtocolError(f"invalid Manus MocapKeypoints payload: {exc}") from exc

    left = _decode_poses(message.left_mocap_pose)
    right = _decode_poses(message.right_mocap_pose)
    if left and len(left) != 25:
        raise ProtocolError(f"expected 25 left Manus keypoints, received {len(left)}")
    if right and len(right) != 25:
        raise ProtocolError(f"expected 25 right Manus keypoints, received {len(right)}")
    if not left and not right:
        raise ProtocolError("Manus packet contains neither left nor right keypoints")
    return MocapPacket(timestamp_s, sequence, left, right)

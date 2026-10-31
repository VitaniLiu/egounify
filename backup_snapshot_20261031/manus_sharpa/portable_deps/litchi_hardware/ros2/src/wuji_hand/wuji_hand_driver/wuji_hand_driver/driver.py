from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

JOINT_COUNT = 20
FINGER_COUNT = 5
JOINTS_PER_FINGER = 4
NODES_PER_FINGER = 5


@dataclass(frozen=True)
class JointCommandValues:
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]


@dataclass(frozen=True)
class JointFeedbackValues:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    efforts: tuple[float, ...]
    frame_id: str


def joint_names(side: str) -> tuple[str, ...]:
    normalized_side = side.strip().lower()
    if normalized_side not in {"left", "right"}:
        raise ValueError("hand side must be 'left' or 'right'")
    return tuple(
        f"{normalized_side}_finger{finger}_joint{joint}"
        for finger in range(1, FINGER_COUNT + 1)
        for joint in range(1, JOINTS_PER_FINGER + 1)
    )


def normalize_command(
    *,
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    efforts: Sequence[float],
    expected_names: Sequence[str],
) -> JointCommandValues:
    expected = tuple(expected_names)
    if len(expected) != JOINT_COUNT or len(set(expected)) != JOINT_COUNT:
        raise ValueError("expected joint names must contain 20 unique entries")
    if len(positions) != JOINT_COUNT:
        raise ValueError(
            f"Wuji Hand 2 command requires {JOINT_COUNT} positions, "
            f"received {len(positions)}"
        )
    if velocities and len(velocities) != JOINT_COUNT:
        raise ValueError(
            f"Wuji Hand 2 command velocity requires {JOINT_COUNT} values, "
            f"received {len(velocities)}"
        )
    if efforts and len(efforts) != JOINT_COUNT:
        raise ValueError(
            f"Wuji Hand 2 command effort requires {JOINT_COUNT} values, "
            f"received {len(efforts)}"
        )

    position_values = tuple(float(value) for value in positions)
    velocity_values = (
        tuple(float(value) for value in velocities)
        if velocities
        else (0.0,) * JOINT_COUNT
    )
    effort_values = (
        tuple(float(value) for value in efforts)
        if efforts
        else (0.0,) * JOINT_COUNT
    )

    if names:
        if len(names) != JOINT_COUNT or len(set(names)) != JOINT_COUNT:
            raise ValueError("named Wuji Hand 2 commands require 20 unique names")
        indices = {name: index for index, name in enumerate(names)}
        missing = [name for name in expected if name not in indices]
        if missing:
            raise ValueError("joint command is missing: " + ", ".join(missing))
        order = tuple(indices[name] for name in expected)
        position_values = tuple(position_values[index] for index in order)
        velocity_values = tuple(velocity_values[index] for index in order)
        effort_values = tuple(effort_values[index] for index in order)

    values = position_values + velocity_values + effort_values
    if any(not math.isfinite(value) for value in values):
        raise ValueError("Wuji Hand 2 command values must be finite")
    return JointCommandValues(position_values, velocity_values, effort_values)


def feedback_values(frame: Any, *, side: str) -> JointFeedbackValues:
    expected_names = joint_names(side)
    entries = tuple(frame.joints)
    if int(frame.num_joints) != len(entries):
        raise ValueError(
            "Wuji Hand 2 joint-state count does not match the received entries"
        )

    by_index = {}
    for entry in entries:
        joint_id = int(entry.nid)
        finger_index, joint_index_in_finger = divmod(
            joint_id - 1, NODES_PER_FINGER
        )
        if (
            joint_id <= 0
            or finger_index >= FINGER_COUNT
            or joint_index_in_finger >= JOINTS_PER_FINGER
        ):
            raise ValueError(f"Wuji Hand 2 reported invalid joint id {joint_id}")
        joint_index = finger_index * JOINTS_PER_FINGER + joint_index_in_finger
        if joint_index in by_index:
            raise ValueError(f"Wuji Hand 2 reported duplicate joint id {joint_id}")
        by_index[joint_index] = entry

    ordered_indices = tuple(sorted(by_index))
    positions = tuple(float(by_index[index].position) for index in ordered_indices)
    velocities = tuple(float(by_index[index].velocity) for index in ordered_indices)
    efforts = tuple(float(by_index[index].effort) for index in ordered_indices)
    if any(not math.isfinite(value) for value in positions + velocities + efforts):
        raise ValueError("Wuji Hand 2 feedback values must be finite")

    frame_id = str(getattr(frame.header, "frame_id", "")).strip()
    if not frame_id:
        frame_id = "l_wrist" if side == "left" else "r_wrist"
    return JointFeedbackValues(
        names=tuple(expected_names[index] for index in ordered_indices),
        positions=positions,
        velocities=velocities,
        efforts=efforts,
        frame_id=frame_id,
    )

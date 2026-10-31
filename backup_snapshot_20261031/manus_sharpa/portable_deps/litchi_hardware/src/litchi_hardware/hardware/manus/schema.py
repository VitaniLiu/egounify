"""Manus hand-skeleton layouts exposed by this package."""

from litchi_hardware.core.types import HandSkeletonSchema

MANUS_SHARPA_25_KEYPOINT_NAMES: tuple[str, ...] = (
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "pinky_metacarpal",
    "pinky_proximal",
    "pinky_intermediate",
    "pinky_distal",
    "pinky_tip",
)

MANUS_SHARPA_25_SCHEMA = HandSkeletonSchema(
    id="manus_sharpa_25_v1",
    keypoint_names=MANUS_SHARPA_25_KEYPOINT_NAMES,
)

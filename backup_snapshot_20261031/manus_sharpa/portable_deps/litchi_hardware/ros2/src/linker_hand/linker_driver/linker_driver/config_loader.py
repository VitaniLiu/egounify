"""Translate ROS/YAML settings into the ROS-independent Linker HAL config."""

from __future__ import annotations

from typing import Any

from litchi_hardware.hardware.linker import (
    LinkerConfig,
    LinkerModel,
    schema_for_model,
)


def build_hand_config(params: dict[str, Any]) -> LinkerConfig:
    model = LinkerModel(str(params.get("model", "O30I")).upper())
    schema = schema_for_model(model)
    calibration = params.get("joint_calibration", {}) or {}
    unknown = sorted(set(calibration) - set(schema.names))
    if unknown:
        raise ValueError(f"calibration contains unknown joints: {unknown}")
    gains = tuple(float(calibration.get(name, {}).get("gain", 255.0)) for name in schema.names)
    offsets = tuple(float(calibration.get(name, {}).get("offset", 0.0)) for name in schema.names)
    initial_speed_value = params.get("initial_speed", [])
    initial_speed = (
        tuple(int(value) for value in initial_speed_value) if initial_speed_value else None
    )
    frame_id_value = int(params.get("frame_id", -1))
    return LinkerConfig(
        name=str(params.get("name", "linker_hand")),
        model=model,
        side=str(params.get("side", "right")).lower(),
        transport=str(params.get("transport", "socketcan")).lower(),
        channel=str(params.get("channel", "can0")),
        bitrate=int(params.get("bitrate", 1_000_000)),
        data_bitrate=int(params.get("data_bitrate", 5_000_000)),
        auto_setup_can=bool(params.get("auto_setup_can", False)),
        frame_id=None if frame_id_value < 0 else frame_id_value,
        canfd_device=int(params.get("canfd_device", 0)),
        canfd_channel=int(params.get("canfd_channel", 0)),
        libcanbus_path=str(params.get("libcanbus_path", "")).strip() or None,
        position_gains=gains,
        position_offsets=offsets,
        initial_speed=initial_speed,
        enable_tactile=bool(params.get("enable_tactile", False)),
        verify_identity=bool(params.get("verify_identity", True)),
    )

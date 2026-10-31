"""Build the core Xynova HAL configuration from ROS parameters."""

from __future__ import annotations

from typing import Any

from litchi_hardware.hardware.xynova import XYNOVA_JOINT_NAMES, XynovaConfig
from xynova_hand.joint_mapping import load_calibration_from_dict


def build_hand_config(params: dict[str, Any]) -> XynovaConfig:
    """Translate ROS/YAML parameter names into the ROS-independent config."""

    calibrations = load_calibration_from_dict(params.get("joint_calibration", {}) or {})
    gains = tuple(
        calibrations[name].gain if name in calibrations else 1.0 for name in XYNOVA_JOINT_NAMES
    )
    offsets = tuple(
        calibrations[name].offset if name in calibrations else 0.0 for name in XYNOVA_JOINT_NAMES
    )

    comm_logger = str(params.get("comm.logger_name", "xynova_hand"))
    slave_logger = str(params.get("slave.logger_name", comm_logger))
    if comm_logger != slave_logger:
        raise ValueError(
            "comm.logger_name and slave.logger_name must match when using the core Xynova adapter"
        )

    return XynovaConfig(
        name=str(params.get("device.name", "dexterous_hand")),
        can_interface=str(params.get("can.iface", "0.0")),
        can_driver=str(params.get("comm.sub_type", "ChuangXinCAN")),
        manager_can=bool(params.get("can.manager_can", False)),
        device_id=int(params.get("slave.id", 0xFFFFFFFF)),
        bitrate=int(params.get("can.bitrate", 1_000_000)),
        data_bitrate=int(params.get("can.dbitrate", 5_000_000)),
        sample_point=float(params.get("can.sample_point", 0.8)),
        data_sample_point=float(params.get("can.dsample_point", 0.8)),
        master_name=str(params.get("master.name", "xynova_master")),
        master_period_us=int(params.get("master.period_us", 1_000)),
        master_cpu_index=int(params.get("master.cpu_index", -1)),
        native_device_name=str(params.get("device.name", "dexterous_hand")),
        native_device_type=str(params.get("device.type", "DexterousHandDevice")),
        native_slave_type=str(params.get("slave.type", "DexterousHandSlave")),
        logger_name=comm_logger,
        log_level=str(params.get("sdk_log_level", "info")),
        enable_tactile=bool(params.get("enable_tactile", False)),
        servo_gains=gains,
        servo_offsets=offsets,
    )

"""Factory for real and mock Xynova arms."""

from litchi_hardware.core.interfaces import ArmInterface

from .config import XynovaArmConfig
from .mock import MockXynovaArm
from .sdk_driver import XynovaSdkArm


def create_arm(
    config: XynovaArmConfig, *, use_fake_hardware: bool = False
) -> ArmInterface:
    if use_fake_hardware:
        return MockXynovaArm(config)
    return XynovaSdkArm(config)

"""Configuration and model metadata for Xynova robot arms."""

from __future__ import annotations

import math
from dataclasses import dataclass

from litchi_hardware.core._compat import StrEnum
from litchi_hardware.core.config import DeviceConfig


class XynovaArmModel(StrEnum):
    XYNOVA = "xynova"
    XYNOVA_CR5 = "xynova_cr5"
    XYNOVA_ER = "xynova_er"
    XYNOVA_AR = "xynova_ar"
    STANDARD = "standard"
    PCB3 = "pcb3"
    PCB4 = "pcb4"


MODEL_JOINT_COUNTS = {
    XynovaArmModel.XYNOVA: 6,
    XynovaArmModel.XYNOVA_CR5: 5,
    XynovaArmModel.XYNOVA_ER: 7,
    XynovaArmModel.XYNOVA_AR: 7,
    XynovaArmModel.STANDARD: 6,
    XynovaArmModel.PCB3: 3,
    XynovaArmModel.PCB4: 4,
}

MODEL_NATIVE_CODES = {
    model: index for index, model in enumerate(XynovaArmModel)
}

COLLABORATIVE_MODELS = frozenset(
    {
        XynovaArmModel.XYNOVA,
        XynovaArmModel.XYNOVA_CR5,
        XynovaArmModel.XYNOVA_ER,
        XynovaArmModel.XYNOVA_AR,
    }
)


@dataclass(frozen=True, slots=True)
class XynovaArmConfig(DeviceConfig):
    """Network, model, and safe motion defaults for one Xynova arm."""

    remote_ip: str = "192.168.1.160"
    local_ip: str = ""
    model: XynovaArmModel = XynovaArmModel.XYNOVA_AR
    default_speed: float = -1.0
    default_zone: float = -1.0
    auto_power_on: bool = False
    power_off_on_disconnect: bool = False

    def __post_init__(self) -> None:
        super(XynovaArmConfig, self).__post_init__()
        if not self.remote_ip:
            raise ValueError("remote_ip must not be empty")
        try:
            model = XynovaArmModel(self.model)
        except ValueError as exc:
            raise ValueError(f"unsupported Xynova arm model: {self.model!r}") from exc
        if not math.isfinite(self.default_speed) or (
            self.default_speed != -1.0 and self.default_speed <= 0.0
        ):
            raise ValueError("default_speed must be -1 (SDK default) or positive")
        if not math.isfinite(self.default_zone) or self.default_zone < -1.0:
            raise ValueError("default_zone must be -1 (SDK default) or non-negative")
        object.__setattr__(self, "model", model)

    @property
    def joint_count(self) -> int:
        return MODEL_JOINT_COUNTS[self.model]

    @property
    def native_model_code(self) -> int:
        return MODEL_NATIVE_CODES[self.model]

    @property
    def supports_wrench(self) -> bool:
        return self.model in COLLABORATIVE_MODELS

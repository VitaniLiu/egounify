"""Named joint schemas for supported Xynova arm families."""

from litchi_hardware.core.types import JointSchema

from .config import MODEL_JOINT_COUNTS, XynovaArmModel


def _schema(model: XynovaArmModel) -> JointSchema:
    count = MODEL_JOINT_COUNTS[model]
    return JointSchema(
        id=f"xynova_arm_{model.value}_{count}_v1",
        names=tuple(f"joint_{index}" for index in range(1, count + 1)),
        unit="rad",
    )


XYNOVA_ARM_JOINT_SCHEMAS = {model: _schema(model) for model in XynovaArmModel}


def arm_joint_schema(model: XynovaArmModel | str) -> JointSchema:
    return XYNOVA_ARM_JOINT_SCHEMAS[XynovaArmModel(model)]

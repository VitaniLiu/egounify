#!/usr/bin/env python3
"""Read Xynova arm identity, joint state, and TCP pose through ArmInterface."""

from __future__ import annotations

import argparse
import sys

from litchi_hardware.core.errors import DeviceError
from litchi_hardware.hardware.xynova import (
    XynovaArmConfig,
    XynovaArmModel,
    create_arm,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remote-ip", default="192.168.1.160")
    parser.add_argument("--local-ip", default="")
    parser.add_argument(
        "--model",
        choices=tuple(model.value for model in XynovaArmModel),
        default=XynovaArmModel.XYNOVA_AR.value,
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="use the mock backend without connecting to hardware",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    arm = create_arm(
        XynovaArmConfig(
            name="example_arm",
            remote_ip=args.remote_ip,
            local_ip=args.local_ip,
            model=XynovaArmModel(args.model),
        ),
        use_fake_hardware=args.fake,
    )
    try:
        arm.connect()
        info = arm.info
        state = arm.read_joint_state()
        pose = arm.read_tcp_pose()
        print(f"device: {info.vendor} {info.model}")
        print(f"id: {info.serial or 'unknown'}")
        print(f"controller: {info.metadata.get('controller_version') or 'unknown'}")
        print(f"SDK: {info.metadata.get('sdk_version') or 'unknown'}")
        print(f"TCP position: {pose.position} m")
        print(f"TCP orientation: {pose.orientation} xyzw")
        for name, position, velocity, effort in zip(
            state.names,
            state.position,
            state.velocity,
            state.effort,
            strict=True,
        ):
            print(
                f"  {name}: position={position:.6f} rad "
                f"velocity={velocity:.6f} rad/s effort={effort:.6f} Nm"
            )
        return 0
    except DeviceError as exc:
        print(f"read failed: {exc}", file=sys.stderr)
        return 1
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""通过统一 litchi_hardware HandInterface 读取 Xynova 状态。"""

from __future__ import annotations

import argparse
import sys

from litchi_hardware.core.errors import DeviceError
from litchi_hardware.hardware.xynova import (
    XynovaConfig,
    create_hand,
)


def auto_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-interface", default="0.0")
    parser.add_argument("--can-driver", default="ChuangXinCAN")
    parser.add_argument("--device-id", type=auto_int, default=0xFFFFFFFF)
    parser.add_argument(
        "--fake",
        action="store_true",
        help="使用实现相同 core interface 的 mock backend",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    hand = create_hand(
        XynovaConfig(
            name="example_hand",
            can_interface=args.can_interface,
            can_driver=args.can_driver,
            device_id=args.device_id,
        ),
        use_fake_hardware=args.fake,
    )

    try:
        hand.connect()
        state = hand.read_joint_state()
        info = hand.info
        print(f"设备: {info.vendor} {info.model}")
        print(f"序列号: {info.serial or 'unknown'}")
        print(f"传输: {info.metadata['transport']}")
        print(f"硬件版本: {info.metadata.get('hardware_version') or 'unknown'}")
        print(f"软件版本: {info.metadata.get('software_version') or 'unknown'}")
        print(f"关节时间戳: {state.timestamp_s:.6f} s")
        for name, position in zip(state.names, state.position, strict=True):
            print(f"  {name:>3}: {position:.6f} rad")
        return 0
    except DeviceError as exc:
        print(f"读取失败: {exc}", file=sys.stderr)
        return 1
    finally:
        hand.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

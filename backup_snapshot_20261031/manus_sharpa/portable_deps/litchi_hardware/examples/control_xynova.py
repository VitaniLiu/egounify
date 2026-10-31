#!/usr/bin/env python3
"""通过统一 HandInterface 让 Xynova FF1 小幅平滑往返一次。

默认不连接设备、不发送命令。确认环境安全后显式传入 ``--execute``。
命令单位为 core interface 和 SDK Angle API 规定的弧度。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from litchi_hardware.core.errors import DeviceError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.types import DeviceAction
from litchi_hardware.hardware.xynova import (
    XYNOVA_JOINT_NAMES,
    XynovaConfig,
    create_hand,
)

DEFAULT_AMPLITUDE_RAD = 0.1
MAX_AMPLITUDE_RAD = 0.5
DEFAULT_DURATION_S = 4.0
COMMAND_RATE_HZ = 50.0


def auto_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="确认环境安全并允许实际发送运动指令",
    )
    parser.add_argument("--can-interface", default="0.0")
    parser.add_argument("--can-driver", default="ChuangXinCAN")
    parser.add_argument("--device-id", type=auto_int, default=0xFFFFFFFF)
    parser.add_argument(
        "--amplitude-rad",
        type=float,
        default=DEFAULT_AMPLITUDE_RAD,
        help=f"FF1 弧度幅度，范围 (0, {MAX_AMPLITUDE_RAD}]",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="完成一次往返的秒数，至少 2 秒",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 0.0 < args.amplitude_rad <= MAX_AMPLITUDE_RAD:
        raise ValueError(f"--amplitude-rad 必须在 (0, {MAX_AMPLITUDE_RAD}] 范围内")
    if args.duration < 2.0:
        raise ValueError("--duration 不能小于 2 秒")


def read_fresh_state(hand: HandInterface, timeout_s: float = 2.0) -> tuple[float, ...]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = hand.read_joint_state()
        if len(state.position) == len(XYNOVA_JOINT_NAMES) and all(
            math.isfinite(value) for value in state.position
        ):
            return state.position
        time.sleep(0.02)
    raise RuntimeError(f"在 2 秒内未收到有效的 {len(XYNOVA_JOINT_NAMES)} 关节反馈")


def send_positions(hand: HandInterface, positions: tuple[float, ...]) -> None:
    hand.send_action(DeviceAction({"joint_position": positions}))


def move_index_finger_once(
    hand: HandInterface,
    baseline: tuple[float, ...],
    *,
    amplitude_rad: float,
    duration_s: float,
) -> None:
    ff1_index = XYNOVA_JOINT_NAMES.index("FF1")
    period_s = 1.0 / COMMAND_RATE_HZ
    start = time.monotonic()

    while True:
        cycle_start = time.monotonic()
        progress = min((cycle_start - start) / duration_s, 1.0)
        offset = amplitude_rad * 0.5 * (1.0 - math.cos(2.0 * math.pi * progress))
        command = list(baseline)
        command[ff1_index] = baseline[ff1_index] + offset
        send_positions(hand, tuple(command))
        if progress >= 1.0:
            break
        time.sleep(max(0.0, period_s - (time.monotonic() - cycle_start)))


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    if not args.execute:
        print("未连接设备或发送命令。确认安全后添加 --execute。")
        return 0

    hand = create_hand(
        XynovaConfig(
            name="index_motion_hand",
            can_interface=args.can_interface,
            can_driver=args.can_driver,
            device_id=args.device_id,
        )
    )
    baseline: tuple[float, ...] | None = None
    command_started = False

    try:
        hand.connect()
        baseline = read_fresh_state(hand)
        ff1_index = XYNOVA_JOINT_NAMES.index("FF1")
        print(f"FF1 起始位置: {baseline[ff1_index]:.6f} rad")
        send_positions(hand, baseline)
        command_started = True
        move_index_finger_once(
            hand,
            baseline,
            amplitude_rad=args.amplitude_rad,
            duration_s=args.duration,
        )
        send_positions(hand, baseline)
        command_started = False
        print("往返完成，FF1 已恢复起始目标。")
        return 0
    except KeyboardInterrupt:
        print("\n收到中断，正在恢复起始目标。", file=sys.stderr)
        return 130
    except (DeviceError, RuntimeError) as exc:
        print(f"运动失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if command_started and baseline is not None:
            try:
                send_positions(hand, baseline)
                time.sleep(0.1)
            except DeviceError as exc:
                print(f"恢复起始目标失败: {exc}", file=sys.stderr)
        try:
            hand.stop()
        finally:
            hand.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

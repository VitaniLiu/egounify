#!/usr/bin/env python3
"""通过统一 HandInterface 让 Linker 食指根关节小幅平滑往返一次。

支持 G20 和 O30I。默认不连接设备、不发送命令；确认环境安全并核对标定后，
必须显式传入 ``--execute``。HAL 命令单位为弧度，默认标定范围为 0..1 rad。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from litchi_hardware import linker
from litchi_hardware.core.errors import DeviceError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.types import DeviceAction, HandSide, JointSchema

DEFAULT_JOINT = "index_root1"
DEFAULT_AMPLITUDE_RAD = 0.1
MAX_AMPLITUDE_RAD = 0.25
DEFAULT_DURATION_S = 4.0
DEFAULT_COMMAND_RATE_HZ = 25.0


def auto_int(value: str) -> int:
    return int(value, 0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="确认环境安全并允许实际发送运动指令",
    )
    parser.add_argument("--fake", action="store_true", help="使用无硬件 mock backend")
    parser.add_argument("--model", choices=("G20", "O30I"), default="O30I")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--transport",
        choices=("socketcan", "libcanbus"),
        default="socketcan",
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--frame-id", type=auto_int)
    parser.add_argument("--canfd-device", type=int, default=0)
    parser.add_argument("--canfd-channel", type=int, default=0)
    parser.add_argument("--libcanbus-path")
    parser.add_argument(
        "--joint",
        default=DEFAULT_JOINT,
        help=f"要运动的 HAL 关节名，默认 {DEFAULT_JOINT}",
    )
    parser.add_argument(
        "--amplitude-rad",
        type=float,
        default=DEFAULT_AMPLITUDE_RAD,
        help=f"相对起点的运动幅度，范围 (0, {MAX_AMPLITUDE_RAD}] rad",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="完成一次往返的秒数，至少 2 秒",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=DEFAULT_COMMAND_RATE_HZ,
        help="命令发送频率 Hz，范围 (0, 100]",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace, schema: JointSchema) -> None:
    if args.joint not in schema.names:
        available = ", ".join(schema.names)
        raise ValueError(f"--joint {args.joint!r} 不属于 {args.model}；可选：{available}")
    if not 0.0 < args.amplitude_rad <= MAX_AMPLITUDE_RAD:
        raise ValueError(f"--amplitude-rad 必须在 (0, {MAX_AMPLITUDE_RAD}] 范围内")
    if args.duration < 2.0:
        raise ValueError("--duration 不能小于 2 秒")
    if not 0.0 < args.command_rate <= 100.0:
        raise ValueError("--command-rate 必须在 (0, 100] 范围内")
    if args.model == "G20" and args.transport != "socketcan":
        raise ValueError("G20 只支持 socketcan transport")


def read_fresh_state(
    hand: HandInterface,
    schema: JointSchema,
    timeout_s: float = 2.0,
) -> tuple[float, ...]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = hand.read_joint_state()
        if (
            state.names == schema.names
            and len(state.position) == schema.size
            and all(math.isfinite(value) for value in state.position)
        ):
            return state.position
        time.sleep(0.02)
    raise RuntimeError(f"在 {timeout_s:.1f} 秒内未收到有效的 {schema.size} 关节反馈")


def send_positions(hand: HandInterface, positions: tuple[float, ...]) -> None:
    hand.send_action(DeviceAction({"joint_position": positions}))


def movement_direction(
    baseline: float,
    limits: tuple[float, float],
    amplitude_rad: float,
) -> float:
    lower, upper = limits
    positive_room = upper - baseline
    negative_room = baseline - lower
    if positive_room >= amplitude_rad:
        return 1.0
    if negative_room >= amplitude_rad:
        return -1.0
    raise RuntimeError(
        f"起点 {baseline:.6f} rad 距关节限位不足 {amplitude_rad:.6f} rad"
    )


def move_joint_once(
    hand: HandInterface,
    schema: JointSchema,
    baseline: tuple[float, ...],
    *,
    joint_name: str,
    amplitude_rad: float,
    duration_s: float,
    command_rate_hz: float,
) -> None:
    joint_index = schema.names.index(joint_name)
    direction = movement_direction(
        baseline[joint_index],
        schema.limits[joint_index],
        amplitude_rad,
    )
    period_s = 1.0 / command_rate_hz
    start = time.monotonic()

    while True:
        cycle_start = time.monotonic()
        progress = min((cycle_start - start) / duration_s, 1.0)
        excursion = amplitude_rad * 0.5 * (1.0 - math.cos(2.0 * math.pi * progress))
        command = list(baseline)
        command[joint_index] = baseline[joint_index] + direction * excursion
        send_positions(hand, tuple(command))
        if progress >= 1.0:
            break
        time.sleep(max(0.0, period_s - (time.monotonic() - cycle_start)))


def build_config(args: argparse.Namespace) -> linker.LinkerConfig:
    return linker.LinkerConfig(
        name=f"control_{args.side}_{args.model.lower()}",
        model=args.model,
        side=HandSide(args.side),
        transport=args.transport,
        channel=args.channel,
        frame_id=args.frame_id,
        canfd_device=args.canfd_device,
        canfd_channel=args.canfd_channel,
        libcanbus_path=args.libcanbus_path,
        verify_identity=not args.fake,
    )


def main() -> int:
    args = parse_args()
    schema = linker.schema_for_model(linker.LinkerModel(args.model))
    try:
        validate_args(args, schema)
        config = build_config(args)
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    if not args.execute:
        print(
            "未连接设备或发送命令。确认手周围安全、急停可用且标定正确后添加 --execute。"
        )
        print(
            f"计划：{args.model} {args.side} hand，关节 {args.joint}，"
            f"幅度 {args.amplitude_rad:.3f} rad，时长 {args.duration:.1f} s。"
        )
        return 0

    hand = linker.create_hand(config, use_fake_hardware=args.fake)
    baseline: tuple[float, ...] | None = None
    command_started = False

    try:
        hand.connect()
        baseline = read_fresh_state(hand, schema)
        joint_index = schema.names.index(args.joint)
        direction = movement_direction(
            baseline[joint_index],
            schema.limits[joint_index],
            args.amplitude_rad,
        )
        target = baseline[joint_index] + direction * args.amplitude_rad
        print(hand.info)
        print(
            f"{args.joint} 起点 {baseline[joint_index]:.6f} rad，"
            f"峰值目标 {target:.6f} rad"
        )

        send_positions(hand, baseline)
        command_started = True
        move_joint_once(
            hand,
            schema,
            baseline,
            joint_name=args.joint,
            amplitude_rad=args.amplitude_rad,
            duration_s=args.duration,
            command_rate_hz=args.command_rate,
        )
        send_positions(hand, baseline)
        time.sleep(0.1)
        command_started = False
        print(f"往返完成，{args.joint} 已恢复起始目标。")
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

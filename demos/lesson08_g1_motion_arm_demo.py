#!/usr/bin/env python3
"""Lesson 08: G1 低速移动与手臂预设动作安全示例。

默认是 dry-run，不导入 Unitree SDK，也不会连接机器人。只有显式提供
``--execute --acknowledge-safety READY`` 后才会创建 LocoClient 和
G1ArmActionClient 并发送动作。
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol, Sequence, Union


ARM_ACTIONS = {
    "hands-up": 15,
    "wave": 25,
    "handshake": 27,
}
SUPPORTED_CONTROL_FSM_IDS = frozenset({500, 501, 801})


@dataclass(frozen=True)
class SafetyLimits:
    max_vx: float = 0.10
    max_vy: float = 0.08
    max_vyaw: float = 0.15
    max_move_duration: float = 1.0
    max_action_duration: float = 8.0
    max_plan_duration: float = 20.0


@dataclass(frozen=True)
class MoveStep:
    vx: float
    vy: float
    vyaw: float
    duration: float
    name: str = "低速移动"


@dataclass(frozen=True)
class ArmActionStep:
    action: str
    duration: float


@dataclass(frozen=True)
class StopStep:
    name: str = "显式停止"


PlanStep = Union[MoveStep, ArmActionStep, StopStep]


class PlanValidationError(ValueError):
    pass


class PlanTimeoutError(TimeoutError):
    pass


def build_plan(action: str) -> list[PlanStep]:
    """根据白名单名称构建演示计划，不接受任意动作 ID。"""

    plans: dict[str, list[PlanStep]] = {
        "move-forward": [
            MoveStep(0.08, 0.0, 0.0, 0.8, "向前低速移动"),
            StopStep(),
        ],
        "move-left": [
            MoveStep(0.0, 0.06, 0.0, 0.8, "向左低速侧移"),
            StopStep(),
        ],
        "turn-left": [
            MoveStep(0.0, 0.0, 0.12, 0.7, "低速左转"),
            StopStep(),
        ],
        "hands-up": [ArmActionStep("hands-up", 3.0), StopStep()],
        "wave": [ArmActionStep("wave", 4.0), StopStep()],
        "handshake": [ArmActionStep("handshake", 5.0), StopStep()],
        "demo": [
            MoveStep(0.08, 0.0, 0.0, 0.8, "向前低速移动"),
            StopStep(),
            ArmActionStep("hands-up", 3.0),
            ArmActionStep("wave", 4.0),
            ArmActionStep("handshake", 5.0),
            StopStep(),
        ],
    }
    try:
        return list(plans[action])
    except KeyError as exc:
        raise PlanValidationError(f"动作不在白名单中：{action}") from exc


def validate_plan(
    plan: Sequence[PlanStep], limits: SafetyLimits | None = None
) -> float:
    """纯 Python 校验速度、时长与手臂动作白名单，返回预计总时长。"""

    limits = limits or SafetyLimits()
    total_duration = 0.0
    for index, step in enumerate(plan, start=1):
        if isinstance(step, MoveStep):
            if abs(step.vx) > limits.max_vx:
                raise PlanValidationError(f"步骤 {index} 的 vx 超过安全上限")
            if abs(step.vy) > limits.max_vy:
                raise PlanValidationError(f"步骤 {index} 的 vy 超过安全上限")
            if abs(step.vyaw) > limits.max_vyaw:
                raise PlanValidationError(f"步骤 {index} 的 vyaw 超过安全上限")
            if not 0.0 < step.duration <= limits.max_move_duration:
                raise PlanValidationError(f"步骤 {index} 的移动时长不安全")
            total_duration += step.duration
        elif isinstance(step, ArmActionStep):
            if step.action not in ARM_ACTIONS:
                raise PlanValidationError(f"步骤 {index} 的手臂动作不在白名单中")
            if not 0.0 < step.duration <= limits.max_action_duration:
                raise PlanValidationError(f"步骤 {index} 的手臂动作时长不安全")
            total_duration += step.duration
        elif not isinstance(step, StopStep):
            raise PlanValidationError(f"步骤 {index} 类型未知")

    if total_duration > limits.max_plan_duration:
        raise PlanValidationError("演示计划总时长超过安全上限")
    return total_duration


class G1Adapter(Protocol):
    def move(self, vx: float, vy: float, vyaw: float) -> None: ...

    def stop_move(self) -> None: ...

    def arm_action(self, action_id: int) -> None: ...

    def release_arm(self) -> None: ...


class DryRunAdapter:
    """打印计划，但不导入 SDK、不创建客户端。"""

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        print("  DRY-RUN Move(" f"vx={vx:+.3f}, vy={vy:+.3f}, vyaw={vyaw:+.3f})")

    def stop_move(self) -> None:
        print("  DRY-RUN StopMove()")

    def arm_action(self, action_id: int) -> None:
        print(f"  DRY-RUN ExecuteAction({action_id})")

    def release_arm(self) -> None:
        print("  DRY-RUN ExecuteAction(99)  # 释放手臂预设动作")


class G1SdkAdapter:
    """封装 G1 SDK；只能由显式执行路径创建。"""

    def __init__(self, interface: str, sdk_timeout: float = 5.0) -> None:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        except ImportError as exc:
            raise RuntimeError(
                "实机执行需要安装 unitree_sdk2py；离线演示请移除 --execute"
            ) from exc

        ChannelFactoryInitialize(0, interface)

        self._loco_client = LocoClient()
        self._loco_client.SetTimeout(sdk_timeout)
        self._loco_client.Init()

        try:
            self._arm_client = G1ArmActionClient()
            self._arm_client.SetTimeout(sdk_timeout)
            self._arm_client.Init()
        except Exception:
            # 运动客户端已经建立时，后续初始化失败也先明确发送零速度。
            self._loco_client.StopMove()
            raise

    def preflight(self, require_arm_actions: bool) -> int:
        """读取实际 FSM 状态；不满足已知动作条件时拒绝继续。"""

        code, fsm_id = self._loco_client.GetFsmId()
        _ensure_sdk_success("GetFsmId", code)
        if fsm_id is None:
            raise RuntimeError("未能读取 G1 的 FSM 状态")
        if fsm_id not in SUPPORTED_CONTROL_FSM_IDS:
            purpose = "手臂预设动作" if require_arm_actions else "本课程运动示例"
            raise RuntimeError(
                f"当前 FSM ID={fsm_id}，不满足{purpose}的安全执行条件；"
                f"允许值为 {sorted(SUPPORTED_CONTROL_FSM_IDS)}"
            )
        self.stop_move()
        return int(fsm_id)

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        result = self._loco_client.Move(vx, vy, vyaw)
        _ensure_sdk_success("Move", result)

    def stop_move(self) -> None:
        result = self._loco_client.StopMove()
        _ensure_sdk_success("StopMove", result)

    def arm_action(self, action_id: int) -> None:
        result = self._arm_client.ExecuteAction(action_id)
        _ensure_sdk_success(f"ExecuteAction({action_id})", result)

    def release_arm(self) -> None:
        result = self._arm_client.ExecuteAction(99)
        _ensure_sdk_success("ExecuteAction(99)", result)


def _ensure_sdk_success(operation: str, result: object) -> None:
    """兼容返回 None 或整数错误码的 SDK 版本。"""

    if isinstance(result, int) and result != 0:
        raise RuntimeError(f"{operation} 执行失败，SDK 错误码 {result}")


def execute_plan(
    plan: Sequence[PlanStep],
    adapter: G1Adapter,
    limits: SafetyLimits | None = None,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """执行已校验计划；异常、超时或中断时都显式停止并释放手臂。"""

    limits = limits or SafetyLimits()
    validate_plan(plan, limits)
    started = clock()
    try:
        for index, step in enumerate(plan, start=1):
            elapsed = clock() - started
            if elapsed > limits.max_plan_duration:
                raise PlanTimeoutError("演示计划超时，已进入停止流程")

            if isinstance(step, MoveStep):
                print(
                    f"步骤 {index}: {step.name}，"
                    f"vx={step.vx:+.2f}, vy={step.vy:+.2f}, "
                    f"vyaw={step.vyaw:+.2f}，持续 {step.duration:.1f}s"
                )
                adapter.move(step.vx, step.vy, step.vyaw)
                _sleep_with_deadline(
                    step.duration, started, limits.max_plan_duration, clock, sleep
                )
                adapter.stop_move()
            elif isinstance(step, ArmActionStep):
                action_id = ARM_ACTIONS[step.action]
                print(
                    f"步骤 {index}: 手臂动作 {step.action}，"
                    f"白名单 ID={action_id}，持续 {step.duration:.1f}s"
                )
                adapter.arm_action(action_id)
                _sleep_with_deadline(
                    step.duration, started, limits.max_plan_duration, clock, sleep
                )
                adapter.release_arm()
            else:
                print(f"步骤 {index}: {step.name}")
                adapter.stop_move()
    finally:
        # StopMove 与释放动作分别尝试，避免前一个异常阻断后一个安全动作。
        try:
            adapter.stop_move()
        finally:
            adapter.release_arm()


def _sleep_with_deadline(
    duration: float,
    started: float,
    max_duration: float,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    if clock() - started + duration > max_duration:
        raise PlanTimeoutError("下一动作将超过演示时限，已拒绝执行")
    sleep(duration)
    if clock() - started > max_duration:
        raise PlanTimeoutError("演示计划超时，已进入停止流程")


def safety_checklist() -> str:
    return """实机启动前检查：
1. 机器人按设备流程进入稳定站立和可编程控制状态。
2. 地面平整、防滑，机器人四周至少留出 2 米净空。
3. 急停方式已验证，现场监护人员始终握持遥控器。
4. 电量、网络和关节状态正常，机器人没有挂载未知负载。
5. 所有人站在机器人运动范围之外；首次执行建议使用保护架。"""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--action",
        choices=("move-forward", "move-left", "turn-left", "wave", "handshake", "demo"),
        default="demo",
        help="从安全白名单选择一个演示计划",
    )
    parser.add_argument("--interface", help="连接 G1 的网卡名称，如 eth0")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许连接 G1 并发送动作；默认仅 dry-run",
    )
    parser.add_argument(
        "--acknowledge-safety",
        metavar="READY",
        help="逐项完成启动检查后，实机模式必须填写 READY",
    )
    parser.add_argument("--sdk-timeout", type=float, default=5.0)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not 1.0 <= args.sdk_timeout <= 10.0:
        raise ValueError("sdk-timeout 必须在 1~10 秒之间")
    if args.execute:
        if not args.interface:
            raise ValueError("实机模式必须提供 --interface")
        if args.acknowledge_safety != "READY":
            raise ValueError("实机执行前必须提供 --acknowledge-safety READY")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
        plan = build_plan(args.action)
        expected_duration = validate_plan(plan)
    except (ValueError, PlanValidationError) as exc:
        parser.error(str(exc))

    print(safety_checklist())
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"\n运行模式：{mode}；计划预计用时 {expected_duration:.1f}s")
    adapter: G1Adapter
    if args.execute:
        adapter = G1SdkAdapter(args.interface, args.sdk_timeout)
        require_arm_actions = any(isinstance(step, ArmActionStep) for step in plan)
        try:
            fsm_id = adapter.preflight(require_arm_actions)
            print(f"实机预检通过：FSM ID={fsm_id}，已先发送 StopMove()。")
        except Exception:
            try:
                adapter.stop_move()
            finally:
                adapter.release_arm()
            raise
    else:
        adapter = DryRunAdapter()

    sleep: Callable[[float], None]
    if args.execute:
        sleep = time.sleep
    else:
        sleep = lambda duration: print(
            f"  DRY-RUN 等待 {duration:.1f}s（已跳过实际等待）"
        )

    try:
        execute_plan(plan, adapter, sleep=sleep)
    except KeyboardInterrupt:
        print("收到中断，已进入 finally 停止流程。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

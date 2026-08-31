#!/usr/bin/env python3
"""Lesson 07：利用前/左/右距离构建单命令源安全控制器。

默认 ``--dry-run`` 只回放内置距离数据，不导入宇树 SDK。``--observe`` 会
读取 ``rt/utlidar/range_info``，但绝不创建 SportClient；只有显式传入
``--execute``，程序才会把状态机生成的唯一一条命令交给 SportClient。

控制链路为：距离消息 -> 有效性检查 -> 中值/EMA -> 滞回状态机 -> 唯一命令源。
状态含义：CLEAR 低速前进、SLOW 减速、STOP 制动、TURN 原地选择空旷侧转向。

实机前提与安全说明
------------------
1. 不同固件/配置的 range_info 字段语义可能不同。先用 ``--observe`` 人工遮挡
   前、左、右方向，确认 x/y/z 的映射和单位确实是米，再考虑执行模式。
2. 测试区域四周至少留出 2 m，移除玻璃、镜面、细杆、台阶和移动人员；遥控器
   与急停全程可用，建议给机器人设置更低的系统限速。
3. 本例不是完整导航系统：它没有地图、定位、路径规划和动态目标预测。
4. 雷达失效、数据长时间无效或程序退出时采取 StopMove，但这仍不能替代
   现场监护、机械隔离、遥控急停与厂商安全机制。
"""

from __future__ import annotations

import argparse
import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Deque, Optional, Protocol, Tuple


class LidarSafetyState(str, Enum):
    """雷达安全控制状态。"""

    CLEAR = "CLEAR"
    SLOW = "SLOW"
    STOP = "STOP"
    TURN = "TURN"


class MotionClient(Protocol):
    def Move(self, vx: float, vy: float, vyaw: float) -> object: ...

    def StopMove(self) -> object: ...


@dataclass(frozen=True)
class LidarSafetyConfig:
    """距离单位为米，速度单位为 m/s 与 rad/s。"""

    stop_enter_m: float = 0.45
    stop_exit_m: float = 0.65
    slow_enter_m: float = 1.10
    slow_exit_m: float = 1.35
    side_turn_min_m: float = 0.80
    side_margin_m: float = 0.20
    clear_speed_mps: float = 0.20
    slow_speed_mps: float = 0.08
    turn_speed_radps: float = 0.30
    turn_delay_s: float = 0.60
    transition_hold_s: float = 0.25
    median_window: int = 5
    ema_alpha: float = 0.45
    min_valid_m: float = 0.05
    max_valid_m: float = 20.0
    invalid_stop_after: int = 3

    def __post_init__(self) -> None:
        if not (
            0.0 < self.stop_enter_m < self.stop_exit_m < self.slow_enter_m
            < self.slow_exit_m
        ):
            raise ValueError(
                "应满足 0 < stop_enter < stop_exit < slow_enter < slow_exit"
            )
        if self.side_turn_min_m <= self.stop_enter_m:
            raise ValueError("side_turn_min_m 应大于 stop_enter_m")
        if self.side_margin_m < 0.0:
            raise ValueError("side_margin_m 不能为负数")
        if self.clear_speed_mps < 0.0 or self.slow_speed_mps < 0.0:
            raise ValueError("前进速度不能为负数")
        if self.slow_speed_mps > self.clear_speed_mps:
            raise ValueError("slow_speed_mps 不应大于 clear_speed_mps")
        if self.turn_speed_radps <= 0.0:
            raise ValueError("turn_speed_radps 必须大于 0")
        if self.turn_delay_s < 0.0 or self.transition_hold_s < 0.0:
            raise ValueError("持续时间不能为负数")
        if self.median_window < 1:
            raise ValueError("median_window 至少为 1")
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha 必须位于 (0, 1]")
        if not 0.0 < self.min_valid_m < self.max_valid_m:
            raise ValueError("距离有效范围设置错误")
        if self.invalid_stop_after < 1:
            raise ValueError("invalid_stop_after 至少为 1")


@dataclass(frozen=True)
class MotionCommand:
    """状态机的唯一运动输出。kind 为 MOVE 或 STOP。"""

    kind: str
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0


@dataclass(frozen=True)
class LidarDecision:
    """一帧距离数据对应的控制结果。"""

    timestamp: float
    state: LidarSafetyState
    changed: bool
    front_m: Optional[float]
    left_m: Optional[float]
    right_m: Optional[float]
    front_valid: bool
    command: MotionCommand
    reason: str


def sanitize_distance(
    value: Optional[float], min_valid_m: float = 0.05, max_valid_m: float = 20.0
) -> Optional[float]:
    """过滤 None、NaN、无穷值、零值和量程外数据。"""

    if value is None:
        return None
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance):
        return None
    if not min_valid_m <= distance <= max_valid_m:
        return None
    return distance


class DistanceFilter:
    """距离中值滤波 + EMA，并记录连续无效帧数。"""

    def __init__(
        self,
        window_size: int,
        alpha: float,
        min_valid_m: float,
        max_valid_m: float,
    ) -> None:
        self._samples: Deque[float] = deque(maxlen=window_size)
        self._alpha = alpha
        self._min = min_valid_m
        self._max = max_valid_m
        self.value: Optional[float] = None
        self.invalid_count = 0

    def update(self, sample: Optional[float]) -> Tuple[Optional[float], bool]:
        clean = sanitize_distance(sample, self._min, self._max)
        if clean is None:
            self.invalid_count += 1
            return self.value, False

        self.invalid_count = 0
        self._samples.append(clean)
        median = float(statistics.median(self._samples))
        if self.value is None:
            self.value = median
        else:
            self.value += self._alpha * (median - self.value)
        return self.value, True


class LidarSafetyController:
    """纯 Python 雷达状态机，不导入 SDK，也不直接发送运动指令。"""

    def __init__(
        self, config: LidarSafetyConfig = LidarSafetyConfig()
    ) -> None:
        self.config = config
        args = (
            config.median_window,
            config.ema_alpha,
            config.min_valid_m,
            config.max_valid_m,
        )
        self._front_filter = DistanceFilter(*args)
        self._left_filter = DistanceFilter(*args)
        self._right_filter = DistanceFilter(*args)
        self.state = LidarSafetyState.CLEAR
        self._state_since: Optional[float] = None
        self._candidate: Optional[LidarSafetyState] = None
        self._candidate_since: Optional[float] = None
        self._turn_sign = 0

    def _choose_turn(
        self, left_m: Optional[float], right_m: Optional[float]
    ) -> int:
        """左侧更空旷返回 +1，右侧更空旷返回 -1，无可靠方向返回 0。"""

        c = self.config
        left_ok = left_m is not None and left_m >= c.side_turn_min_m
        right_ok = right_m is not None and right_m >= c.side_turn_min_m

        if self._turn_sign > 0 and left_ok:
            if not right_ok or left_m + c.side_margin_m >= right_m:  # type: ignore[operator]
                return 1
        if self._turn_sign < 0 and right_ok:
            if not left_ok or right_m + c.side_margin_m >= left_m:  # type: ignore[operator]
                return -1

        if left_ok and not right_ok:
            return 1
        if right_ok and not left_ok:
            return -1
        if left_ok and right_ok:
            assert left_m is not None and right_m is not None
            difference = float(left_m) - float(right_m)
            if difference >= c.side_margin_m:
                return 1
            if difference <= -c.side_margin_m:
                return -1
        return 0

    def _target_state(
        self,
        front_m: Optional[float],
        left_m: Optional[float],
        right_m: Optional[float],
        timestamp: float,
    ) -> LidarSafetyState:
        c = self.config
        if front_m is None:
            return LidarSafetyState.STOP

        if self.state is LidarSafetyState.STOP:
            if front_m >= c.stop_exit_m:
                return (
                    LidarSafetyState.CLEAR
                    if front_m >= c.slow_exit_m
                    else LidarSafetyState.SLOW
                )
            stopped_for = timestamp - (
                timestamp if self._state_since is None else self._state_since
            )
            if (
                front_m <= c.stop_enter_m
                and stopped_for >= c.turn_delay_s
                and self._choose_turn(left_m, right_m) != 0
            ):
                return LidarSafetyState.TURN
            return LidarSafetyState.STOP

        if self.state is LidarSafetyState.TURN:
            if front_m >= c.stop_exit_m:
                return (
                    LidarSafetyState.CLEAR
                    if front_m >= c.slow_exit_m
                    else LidarSafetyState.SLOW
                )
            if self._choose_turn(left_m, right_m) == 0:
                return LidarSafetyState.STOP
            return LidarSafetyState.TURN

        if front_m <= c.stop_enter_m:
            return LidarSafetyState.STOP

        if self.state is LidarSafetyState.CLEAR:
            return (
                LidarSafetyState.SLOW
                if front_m <= c.slow_enter_m
                else LidarSafetyState.CLEAR
            )

        # SLOW 状态只有超过更大的退出阈值才恢复 CLEAR。
        return (
            LidarSafetyState.CLEAR
            if front_m >= c.slow_exit_m
            else LidarSafetyState.SLOW
        )

    def _advance(
        self, target: LidarSafetyState, timestamp: float
    ) -> Tuple[LidarSafetyState, bool]:
        if self._state_since is None:
            self._state_since = timestamp

        if target is self.state:
            self._candidate = None
            self._candidate_since = None
            return self.state, False

        # 距离过近或雷达失效时立即 STOP；解除/转向需稳定一段时间。
        if target is LidarSafetyState.STOP:
            self.state = target
            self._state_since = timestamp
            self._candidate = None
            self._candidate_since = None
            self._turn_sign = 0
            return self.state, True

        if target is not self._candidate:
            self._candidate = target
            self._candidate_since = timestamp

        candidate_since = (
            timestamp if self._candidate_since is None else self._candidate_since
        )
        if timestamp - candidate_since + 1e-12 < self.config.transition_hold_s:
            return self.state, False

        self.state = target
        self._state_since = timestamp
        self._candidate = None
        self._candidate_since = None
        return self.state, True

    def _command_for_state(
        self, left_m: Optional[float], right_m: Optional[float]
    ) -> MotionCommand:
        c = self.config
        if self.state is LidarSafetyState.CLEAR:
            return MotionCommand("MOVE", vx=c.clear_speed_mps)
        if self.state is LidarSafetyState.SLOW:
            return MotionCommand("MOVE", vx=c.slow_speed_mps)
        if self.state is LidarSafetyState.STOP:
            return MotionCommand("STOP")

        sign = self._choose_turn(left_m, right_m)
        if sign == 0:
            return MotionCommand("STOP")
        self._turn_sign = sign
        return MotionCommand("MOVE", vyaw=sign * c.turn_speed_radps)

    def update(
        self,
        front_m: Optional[float],
        left_m: Optional[float],
        right_m: Optional[float],
        timestamp: Optional[float] = None,
    ) -> LidarDecision:
        """处理一帧原始距离，返回状态和唯一运动命令。"""

        now = time.monotonic() if timestamp is None else float(timestamp)
        front, front_valid_now = self._front_filter.update(front_m)
        left, _left_valid_now = self._left_filter.update(left_m)
        right, _right_valid_now = self._right_filter.update(right_m)

        front_reliable = (
            front is not None
            and self._front_filter.invalid_count < self.config.invalid_stop_after
        )
        front_for_state = front if front_reliable else None
        left_for_state = (
            left
            if left is not None
            and self._left_filter.invalid_count < self.config.invalid_stop_after
            else None
        )
        right_for_state = (
            right
            if right is not None
            and self._right_filter.invalid_count < self.config.invalid_stop_after
            else None
        )
        target = self._target_state(
            front_for_state, left_for_state, right_for_state, now
        )
        state, changed = self._advance(target, now)

        if state is not LidarSafetyState.TURN:
            self._turn_sign = 0
        command = self._command_for_state(left_for_state, right_for_state)

        if not front_reliable:
            reason = (
                f"前向距离连续无效 {self._front_filter.invalid_count} 帧，保护停车"
            )
        elif state is LidarSafetyState.TURN:
            direction = "左" if command.vyaw > 0.0 else "右"
            reason = f"前方受阻，{direction}侧空间更大，原地转向"
        else:
            reason = f"前向滤波距离 {front:.2f} m，目标状态 {target.value}"

        return LidarDecision(
            timestamp=now,
            state=state,
            changed=changed,
            front_m=front,
            left_m=left,
            right_m=right,
            front_valid=front_valid_now and front_reliable,
            command=command,
            reason=reason,
        )


class LatestRanges:
    """DDS 回调与控制循环之间的线程安全单帧缓存。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Optional[Tuple[float, float, float]] = None
        self._updated_at: Optional[float] = None

    def set(self, front: float, left: float, right: float) -> None:
        with self._lock:
            self._value = (float(front), float(left), float(right))
            self._updated_at = time.monotonic()

    def get(
        self, stale_after_s: float
    ) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            if self._value is None or self._updated_at is None:
                return None
            if time.monotonic() - self._updated_at > stale_after_s:
                return None
            return self._value


class SdkMotionSink:
    """全程序唯一的 SDK 运动命令出口。"""

    def __init__(self, sport_client: MotionClient, resend_s: float = 0.25) -> None:
        self._sport = sport_client
        self._resend_s = resend_s
        self._last: Optional[MotionCommand] = None
        self._last_sent_at = -math.inf

    def send(self, command: MotionCommand, timestamp: float) -> None:
        if command == self._last and timestamp - self._last_sent_at < self._resend_s:
            return
        if command.kind == "STOP":
            self._sport.StopMove()
        elif command.kind == "MOVE":
            self._sport.Move(command.vx, command.vy, command.vyaw)
        else:
            raise ValueError(f"未知命令类型：{command.kind}")
        self._last = command
        self._last_sent_at = timestamp

    def safe_shutdown(self) -> None:
        self._sport.StopMove()


def _format_range(value: Optional[float]) -> str:
    return " -- " if value is None else f"{value:4.2f}"


def _print_decision(decision: LidarDecision) -> None:
    command = decision.command
    command_text = (
        "StopMove()"
        if command.kind == "STOP"
        else f"Move({command.vx:.2f}, {command.vy:.2f}, {command.vyaw:.2f})"
    )
    mark = "状态切换" if decision.changed else "保持"
    print(
        f"[{decision.timestamp:8.2f}] {decision.state.value:5s} {mark:4s} | "
        f"F/L/R={_format_range(decision.front_m)}/"
        f"{_format_range(decision.left_m)}/{_format_range(decision.right_m)} m | "
        f"{command_text:25s} | {decision.reason}"
    )


def run_dry_demo(config: LidarSafetyConfig) -> None:
    """回放“畅通—减速—停车—左转—恢复”的离线数据。"""

    controller = LidarSafetyController(config)
    samples = []
    timestamp = 0.0
    # 段落依次展示：CLEAR、SLOW、STOP、向左 TURN、恢复、传感器失效。
    for count, front, left, right in (
        (6, 2.0, 1.5, 1.4),
        (7, 0.9, 1.5, 1.4),
        (12, 0.3, 1.6, 0.6),
        (7, 0.8, 1.5, 0.8),
        (8, 1.6, 1.5, 1.4),
        (3, math.nan, 1.5, 1.4),
    ):
        for _ in range(count):
            samples.append((timestamp, front, left, right))
            timestamp += 0.2
    print("离线演示：只计算命令，不导入 SDK，也不会驱动机器人。")
    for timestamp, front, left, right in samples:
        _print_decision(controller.update(front, left, right, timestamp))


def _initialize_channel(interface: str, domain: int) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if interface:
        ChannelFactoryInitialize(domain, interface)
    else:
        ChannelFactoryInitialize(domain)


def run_live(args: argparse.Namespace, execute: bool) -> None:
    """订阅距离并运行状态机；execute=False 时严格不创建运动客户端。"""

    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PointStamped_
    except ImportError as exc:
        raise SystemExit(
            "未找到 unitree_sdk2py。请先安装课程 SDK，或使用默认 --dry-run。"
        ) from exc

    _initialize_channel(args.interface, args.domain)
    latest = LatestRanges()

    def on_range(message: Any) -> None:
        try:
            latest.set(message.point.x, message.point.y, message.point.z)
        except (AttributeError, TypeError, ValueError):
            pass

    subscriber = ChannelSubscriber("rt/utlidar/range_info", PointStamped_)
    subscriber.Init(on_range, 10)

    sink: Optional[SdkMotionSink] = None
    if execute:
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        sport = SportClient()
        sport.SetTimeout(3.0)
        sport.Init()
        sink = SdkMotionSink(sport)

    controller = LidarSafetyController()
    start = time.monotonic()
    mode_name = "执行控制" if execute else "只读观察"
    print(f"实机模式：{mode_name}；Ctrl+C 结束。")
    if execute:
        print(
            "已启用 Move/StopMove：确认场地清空、字段映射正确、遥控急停可用。"
        )

    try:
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            values = latest.get(args.stale_after)
            if values is None:
                decision = controller.update(None, None, None)
            else:
                decision = controller.update(*values)
            _print_decision(decision)
            if sink is not None:
                sink.send(decision.command, decision.timestamp)
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\n收到中断，准备停车退出。")
    finally:
        if sink is not None:
            sink.safe_shutdown()
        close = getattr(subscriber, "Close", None)
        if callable(close):
            close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lesson 07：雷达距离滤波、滞回避障与单命令源控制"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="回放内置距离，不导入 SDK（默认）",
    )
    mode.add_argument(
        "--observe",
        action="store_true",
        help="只订阅实机雷达，不创建 SportClient",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="显式允许状态机通过唯一命令源调用 Move/StopMove",
    )
    parser.add_argument("--interface", default="ens37", help="机器人网卡名")
    parser.add_argument("--domain", type=int, default=0, help="DDS 域编号")
    parser.add_argument("--period", type=float, default=0.10, help="控制周期（秒）")
    parser.add_argument(
        "--stale-after",
        type=float,
        default=0.35,
        help="超过该时间未收到新数据即视为失效（秒）",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行时长；0 表示持续运行",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.period <= 0.0 or args.stale_after <= 0.0:
        raise SystemExit("--period 和 --stale-after 必须大于 0")
    if args.execute:
        run_live(args, execute=True)
    elif args.observe:
        run_live(args, execute=False)
    else:
        run_dry_demo(LidarSafetyConfig())


if __name__ == "__main__":
    main()

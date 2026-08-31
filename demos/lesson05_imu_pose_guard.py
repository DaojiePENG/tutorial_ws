#!/usr/bin/env python3
"""Lesson 05：利用 IMU 姿态实现带滞回的安全保护。

默认运行方式是 ``--dry-run``：程序只回放内置数据，不导入宇树 SDK，
也不会向机器人发送任何指令。``--observe`` 只订阅实机姿态并打印判断结果；
只有显式指定 ``--execute`` 时，程序才会创建 VuiClient/SportClient，并在
STOP 状态调用 ``StopMove()``（LED 可用 ``--no-led`` 关闭）。

实机前提与安全说明
------------------
1. 先在离线模式核对阈值，再用 ``--observe`` 验证 R/P 方向、单位和零偏。
2. 清空机器人四周至少 2 m，遥控器和急停保持可用，建议两人配合测试。
3. 本例是教学用的附加保护，不能替代遥控急停、限速、支撑架或现场监护。
4. IMU 瞬时冲击、通信中断和地面打滑都可能造成误判；执行前应从低速开始。
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
from typing import Any, Deque, Iterable, Optional, Protocol, Sequence, Tuple


class PoseSafetyState(str, Enum):
    """姿态保护状态。"""

    NORMAL = "NORMAL"
    WARN = "WARN"
    STOP = "STOP"


class StopClient(Protocol):
    def StopMove(self) -> object: ...


class BrightnessClient(Protocol):
    def SetBrightness(self, brightness: int) -> object: ...


@dataclass(frozen=True)
class PoseGuardConfig:
    """姿态保护参数，角度单位均为度。"""

    warn_enter_deg: float = 12.0
    warn_exit_deg: float = 8.0
    stop_enter_deg: float = 25.0
    stop_exit_deg: float = 18.0
    enter_hold_s: float = 0.35
    exit_hold_s: float = 0.80
    median_window: int = 5
    ema_alpha: float = 0.40
    invalid_stop_after: int = 5

    def __post_init__(self) -> None:
        if not (0.0 <= self.warn_exit_deg < self.warn_enter_deg):
            raise ValueError("应满足 0 <= warn_exit_deg < warn_enter_deg")
        if not (
            self.warn_enter_deg < self.stop_exit_deg < self.stop_enter_deg
        ):
            raise ValueError(
                "应满足 warn_enter_deg < stop_exit_deg < stop_enter_deg"
            )
        if self.enter_hold_s < 0.0 or self.exit_hold_s < 0.0:
            raise ValueError("持续时间不能为负数")
        if self.median_window < 1:
            raise ValueError("median_window 至少为 1")
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha 必须位于 (0, 1]")
        if self.invalid_stop_after < 1:
            raise ValueError("invalid_stop_after 至少为 1")


@dataclass(frozen=True)
class PoseDecision:
    """一帧姿态数据对应的判断结果。"""

    timestamp: float
    state: PoseSafetyState
    changed: bool
    sensor_valid: bool
    raw_roll_deg: Optional[float]
    raw_pitch_deg: Optional[float]
    filtered_roll_deg: float
    filtered_pitch_deg: float
    tilt_deg: float
    reason: str


class MedianEmaFilter:
    """先做滑动中值，再做指数低通，兼顾去毛刺和响应速度。"""

    def __init__(self, window_size: int = 5, alpha: float = 0.4) -> None:
        if window_size < 1:
            raise ValueError("window_size 至少为 1")
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha 必须位于 (0, 1]")
        self._samples: Deque[float] = deque(maxlen=window_size)
        self._alpha = alpha
        self._value: Optional[float] = None

    @property
    def value(self) -> Optional[float]:
        return self._value

    def update(self, sample: float) -> float:
        self._samples.append(float(sample))
        median = float(statistics.median(self._samples))
        if self._value is None:
            self._value = median
        else:
            self._value += self._alpha * (median - self._value)
        return self._value


def radians_to_degrees(rpy: Sequence[float]) -> Tuple[float, float, float]:
    """将 SDK 的 [roll, pitch, yaw] 弧度值转换为角度。"""

    if len(rpy) < 3:
        raise ValueError("rpy 至少包含 roll、pitch、yaw 三个元素")
    return tuple(math.degrees(float(v)) for v in rpy[:3])  # type: ignore[return-value]


def _valid_angle(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value) and abs(value) <= 180.0


class ImuPoseGuard:
    """不依赖 SDK 的 IMU 姿态保护状态机。

    ``update`` 的输入单位为度。风险增大和风险解除都必须持续一段时间，
    同时进入阈值与退出阈值不同，从而避免机器人在边界附近反复切换状态。
    """

    _RANK = {
        PoseSafetyState.NORMAL: 0,
        PoseSafetyState.WARN: 1,
        PoseSafetyState.STOP: 2,
    }

    def __init__(self, config: PoseGuardConfig = PoseGuardConfig()) -> None:
        self.config = config
        self.state = PoseSafetyState.NORMAL
        self._roll_filter = MedianEmaFilter(
            config.median_window, config.ema_alpha
        )
        self._pitch_filter = MedianEmaFilter(
            config.median_window, config.ema_alpha
        )
        self._candidate: Optional[PoseSafetyState] = None
        self._candidate_since: Optional[float] = None
        self._invalid_count = 0

    def _target_state(self, tilt_deg: float) -> PoseSafetyState:
        c = self.config
        if self.state is PoseSafetyState.NORMAL:
            if tilt_deg >= c.stop_enter_deg:
                return PoseSafetyState.STOP
            if tilt_deg >= c.warn_enter_deg:
                return PoseSafetyState.WARN
            return PoseSafetyState.NORMAL

        if self.state is PoseSafetyState.WARN:
            if tilt_deg >= c.stop_enter_deg:
                return PoseSafetyState.STOP
            if tilt_deg <= c.warn_exit_deg:
                return PoseSafetyState.NORMAL
            return PoseSafetyState.WARN

        # STOP 解除时先回到 WARN；只有姿态充分恢复才回到 NORMAL。
        if tilt_deg > c.stop_exit_deg:
            return PoseSafetyState.STOP
        if tilt_deg <= c.warn_exit_deg:
            return PoseSafetyState.NORMAL
        return PoseSafetyState.WARN

    def _advance(
        self,
        target: PoseSafetyState,
        timestamp: float,
        force: bool = False,
    ) -> Tuple[PoseSafetyState, bool]:
        if target is self.state:
            self._candidate = None
            self._candidate_since = None
            return self.state, False

        if force:
            self.state = target
            self._candidate = None
            self._candidate_since = None
            return self.state, True

        if target is not self._candidate:
            self._candidate = target
            self._candidate_since = timestamp

        increasing = self._RANK[target] > self._RANK[self.state]
        hold_s = self.config.enter_hold_s if increasing else self.config.exit_hold_s
        candidate_since = (
            timestamp if self._candidate_since is None else self._candidate_since
        )
        candidate_age = timestamp - candidate_since
        if candidate_age + 1e-12 < hold_s:
            return self.state, False

        self.state = target
        self._candidate = None
        self._candidate_since = None
        return self.state, True

    def update(
        self,
        roll_deg: Optional[float],
        pitch_deg: Optional[float],
        timestamp: Optional[float] = None,
    ) -> PoseDecision:
        """输入一帧 Roll/Pitch（度），返回滤波后的安全状态。"""

        now = time.monotonic() if timestamp is None else float(timestamp)
        valid = _valid_angle(roll_deg) and _valid_angle(pitch_deg)
        changed = False

        if valid:
            assert roll_deg is not None and pitch_deg is not None
            self._invalid_count = 0
            filtered_roll = self._roll_filter.update(float(roll_deg))
            filtered_pitch = self._pitch_filter.update(float(pitch_deg))
            tilt = max(abs(filtered_roll), abs(filtered_pitch))
            target = self._target_state(tilt)
            state, changed = self._advance(target, now)
            reason = (
                f"滤波后最大倾角 {tilt:.1f}°，目标状态 {target.value}"
            )
        else:
            self._invalid_count += 1
            filtered_roll = self._roll_filter.value or 0.0
            filtered_pitch = self._pitch_filter.value or 0.0
            tilt = max(abs(filtered_roll), abs(filtered_pitch))
            if self._invalid_count >= self.config.invalid_stop_after:
                state, changed = self._advance(
                    PoseSafetyState.STOP, now, force=True
                )
                reason = f"连续 {self._invalid_count} 帧 IMU 数据无效，进入保护"
            else:
                state = self.state
                reason = (
                    f"IMU 数据无效（{self._invalid_count}/"
                    f"{self.config.invalid_stop_after}），暂时保持当前状态"
                )

        return PoseDecision(
            timestamp=now,
            state=state,
            changed=changed,
            sensor_valid=valid,
            raw_roll_deg=(
                float(roll_deg) if roll_deg is not None and _valid_angle(roll_deg) else None
            ),
            raw_pitch_deg=(
                float(pitch_deg)
                if pitch_deg is not None and _valid_angle(pitch_deg)
                else None
            ),
            filtered_roll_deg=filtered_roll,
            filtered_pitch_deg=filtered_pitch,
            tilt_deg=tilt,
            reason=reason,
        )


class LatestRpy:
    """DDS 回调与主循环之间的线程安全单帧缓存。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: Optional[Tuple[float, float, float]] = None
        self._updated_at: Optional[float] = None

    def set(self, rpy: Iterable[float]) -> None:
        values = tuple(float(v) for v in rpy)
        if len(values) < 3:
            return
        with self._lock:
            self._value = values[:3]  # type: ignore[assignment]
            self._updated_at = time.monotonic()

    def get(
        self, stale_after_s: float
    ) -> Optional[Tuple[float, float, float]]:
        with self._lock:
            if self._updated_at is None:
                return None
            if time.monotonic() - self._updated_at > stale_after_s:
                return None
            return self._value


class SdkPoseActuator:
    """本文件中唯一允许调用 LED 和 StopMove 的对象。"""

    _BRIGHTNESS = {
        PoseSafetyState.NORMAL: 0,
        PoseSafetyState.WARN: 5,
        PoseSafetyState.STOP: 10,
    }

    def __init__(
        self,
        sport_client: StopClient,
        vui_client: Optional[BrightnessClient],
        stop_enabled: bool = True,
        heartbeat_s: float = 0.5,
    ) -> None:
        self._sport = sport_client
        self._vui = vui_client
        self._stop_enabled = stop_enabled
        self._heartbeat_s = heartbeat_s
        self._last_state: Optional[PoseSafetyState] = None
        self._last_stop_at = -math.inf

    def apply(self, decision: PoseDecision) -> None:
        state_changed = decision.state is not self._last_state
        if state_changed and self._vui is not None:
            self._vui.SetBrightness(self._BRIGHTNESS[decision.state])

        if (
            decision.state is PoseSafetyState.STOP
            and self._stop_enabled
            and (
                state_changed
                or decision.timestamp - self._last_stop_at >= self._heartbeat_s
            )
        ):
            self._sport.StopMove()
            self._last_stop_at = decision.timestamp

        self._last_state = decision.state

    def safe_shutdown(self) -> None:
        if self._stop_enabled:
            self._sport.StopMove()


def _print_decision(decision: PoseDecision) -> None:
    validity = "有效" if decision.sensor_valid else "无效"
    mark = "状态切换" if decision.changed else "保持"
    print(
        f"[{decision.timestamp:8.2f}] {decision.state.value:6s} {mark:4s} | "
        f"R={decision.filtered_roll_deg:6.1f}° "
        f"P={decision.filtered_pitch_deg:6.1f}° | 数据{validity} | "
        f"{decision.reason}"
    )


def run_dry_demo(config: PoseGuardConfig) -> None:
    """回放一段“正常—倾斜—恢复”的离线数据。"""

    guard = ImuPoseGuard(config)
    samples = []
    timestamp = 0.0
    # 段落依次展示：正常、单帧毛刺、持续警告、持续停车、分级恢复。
    for count, roll, pitch in (
        (5, 2.0, 0.0),
        (1, 40.0, 0.0),
        (5, 3.0, 1.0),
        (7, 16.0, 3.0),
        (10, 30.0, 6.0),
        (8, 15.0, 4.0),
        (15, 4.0, 1.0),
    ):
        for _ in range(count):
            samples.append((timestamp, roll, pitch))
            timestamp += 0.2
    print("离线演示：不会导入 SDK，也不会发送任何实机指令。")
    for timestamp, roll, pitch in samples:
        _print_decision(guard.update(roll, pitch, timestamp))


def _initialize_channel(interface: str, domain: int) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if interface:
        ChannelFactoryInitialize(domain, interface)
    else:
        ChannelFactoryInitialize(domain)


def run_live(args: argparse.Namespace, execute: bool) -> None:
    """订阅实机姿态；execute=False 时严格保持只读。"""

    try:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    except ImportError as exc:
        raise SystemExit(
            "未找到 unitree_sdk2py。请先安装课程 SDK，或使用默认 --dry-run。"
        ) from exc

    _initialize_channel(args.interface, args.domain)
    latest = LatestRpy()

    def on_state(message: Any) -> None:
        try:
            latest.set(message.imu_state.rpy)
        except (AttributeError, TypeError, ValueError):
            pass

    subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    subscriber.Init(on_state, 10)

    actuator: Optional[SdkPoseActuator] = None
    if execute:
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        sport = SportClient()
        sport.SetTimeout(3.0)
        sport.Init()

        vui = None
        if not args.no_led:
            from unitree_sdk2py.go2.vui.vui_client import VuiClient

            vui = VuiClient()
            vui.SetTimeout(3.0)
            vui.Init()
        actuator = SdkPoseActuator(
            sport, vui, stop_enabled=not args.no_stop
        )
        # 执行模式启动时先清除可能残留的运动指令，再等待有效 IMU 数据。
        actuator.safe_shutdown()

    guard = ImuPoseGuard()
    start = time.monotonic()
    mode_name = "执行保护" if execute else "只读观察"
    print(f"实机模式：{mode_name}；Ctrl+C 结束。")
    if execute:
        print("已启用实机输出：STOP 状态会调用 StopMove()。")

    try:
        while args.duration <= 0 or time.monotonic() - start < args.duration:
            rpy = latest.get(args.stale_after)
            if rpy is None:
                decision = guard.update(None, None)
            else:
                roll, pitch, _yaw = radians_to_degrees(rpy)
                decision = guard.update(roll, pitch)
            _print_decision(decision)
            if actuator is not None:
                actuator.apply(decision)
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\n收到中断，准备退出。")
    finally:
        if actuator is not None:
            actuator.safe_shutdown()
        close = getattr(subscriber, "Close", None)
        if callable(close):
            close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lesson 05：IMU 姿态滤波、滞回判断与 StopMove 保护"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="回放内置数据，不导入 SDK（默认）",
    )
    mode.add_argument(
        "--observe",
        action="store_true",
        help="只订阅实机姿态，不创建运动或灯光客户端",
    )
    mode.add_argument(
        "--execute",
        action="store_true",
        help="显式允许 LED 输出，并在 STOP 状态调用 StopMove()",
    )
    parser.add_argument("--interface", default="ens37", help="机器人网卡名")
    parser.add_argument("--domain", type=int, default=0, help="DDS 域编号")
    parser.add_argument("--period", type=float, default=0.10, help="判断周期（秒）")
    parser.add_argument(
        "--stale-after",
        type=float,
        default=0.35,
        help="超过该时间未收到新姿态即视为失效（秒）",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="运行时长；0 表示持续运行"
    )
    parser.add_argument("--no-led", action="store_true", help="执行模式不控制 LED")
    parser.add_argument(
        "--no-stop", action="store_true", help="执行模式只显示/亮灯，不调用 StopMove"
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
        run_dry_demo(PoseGuardConfig())


if __name__ == "__main__":
    main()

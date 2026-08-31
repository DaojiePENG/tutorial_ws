#!/usr/bin/env python3
"""Lesson 06: Go2 视觉闭环控制安全示例。

默认运行合成检测数据，只打印决策，不连接机器人、不保存图像。只有同时提供
``--source go2 --execute --acknowledge-safety READY`` 时才会发送运动指令。

闭环逻辑只依赖 Python 标准库，便于离线讲解和单元测试。OpenCV、NumPy 与
Unitree SDK 都在实时模式下延迟导入。
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional, Protocol, Sequence


class VisionState(str, Enum):
    """视觉闭环的四种可观察状态。"""

    SEARCH = "SEARCH"
    TRACK = "TRACK"
    STOP = "STOP"
    LOST = "LOST"


@dataclass(frozen=True)
class Detection:
    """单个目标检测结果，bbox 采用 (x1, y1, x2, y2) 像素坐标。"""

    bbox: tuple[float, float, float, float]
    confidence: float
    label: str = "target"

    def __post_init__(self) -> None:
        x1, y1, x2, y2 = self.bbox
        if x2 <= x1 or y2 <= y1:
            raise ValueError("bbox 必须满足 x2>x1 且 y2>y1")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence 必须在 0~1 之间")

    def area_ratio(self, frame_width: int, frame_height: int) -> float:
        frame_area = max(1.0, float(frame_width * frame_height))
        x1, y1, x2, y2 = self.bbox
        return max(0.0, (x2 - x1) * (y2 - y1) / frame_area)

    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


@dataclass(frozen=True)
class MotionCommand:
    """发送给底盘的速度指令。"""

    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0

    @property
    def is_stop(self) -> bool:
        return abs(self.vx) < 1e-9 and abs(self.vy) < 1e-9 and abs(self.vyaw) < 1e-9


@dataclass(frozen=True)
class Decision:
    state: VisionState
    command: MotionCommand
    reason: str
    detection: Optional[Detection] = None


@dataclass(frozen=True)
class ControllerConfig:
    confidence_threshold: float = 0.65
    confirm_frames: int = 3
    lost_frames: int = 3
    desired_area_ratio: float = 0.08
    stop_area_ratio: float = 0.18
    center_deadband: float = 0.08
    association_distance: float = 0.30
    max_forward_speed: float = 0.15
    max_yaw_speed: float = 0.35
    forward_gain: float = 2.0
    yaw_gain: float = 0.45

    def __post_init__(self) -> None:
        if self.confirm_frames < 1 or self.lost_frames < 1:
            raise ValueError("连续帧阈值必须至少为 1")
        if not 0.0 < self.desired_area_ratio < self.stop_area_ratio < 1.0:
            raise ValueError("面积比例必须满足 0 < desired < stop < 1")
        if self.max_forward_speed <= 0.0 or self.max_yaw_speed <= 0.0:
            raise ValueError("速度上限必须大于 0")


class VisionClosedLoopController:
    """将检测框转换为安全、受限的运动决策。"""

    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()
        self.state = VisionState.SEARCH
        self._seen_streak = 0
        self._miss_streak = 0
        self._last_detection: Optional[Detection] = None
        self._has_confirmed_target = False

    def reset(self) -> None:
        self.state = VisionState.SEARCH
        self._seen_streak = 0
        self._miss_streak = 0
        self._last_detection = None
        self._has_confirmed_target = False

    def update(
        self,
        detections: Sequence[Detection],
        frame_width: int,
        frame_height: int,
    ) -> Decision:
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError("图像宽高必须大于 0")

        target = self._select_target(detections)
        if target is None:
            return self._handle_missing_target()

        self._miss_streak = 0
        if self._is_same_target(target, frame_width, frame_height):
            self._seen_streak += 1
        else:
            self._seen_streak = 1
        self._last_detection = target

        area_ratio = target.area_ratio(frame_width, frame_height)
        if area_ratio >= self.config.stop_area_ratio:
            self.state = VisionState.STOP
            self._has_confirmed_target = True
            return Decision(
                self.state,
                MotionCommand(),
                f"目标过近，面积占比 {area_ratio:.3f} 达到安全阈值",
                target,
            )

        if self._seen_streak < self.config.confirm_frames:
            self.state = VisionState.SEARCH
            return Decision(
                self.state,
                MotionCommand(),
                f"等待连续帧确认 {self._seen_streak}/{self.config.confirm_frames}",
                target,
            )

        self._has_confirmed_target = True
        self.state = VisionState.TRACK
        command = self._tracking_command(target, frame_width, frame_height)
        return Decision(
            self.state,
            command,
            f"目标已确认，面积占比 {area_ratio:.3f}",
            target,
        )

    def _select_target(self, detections: Sequence[Detection]) -> Optional[Detection]:
        candidates = [
            detection
            for detection in detections
            if detection.confidence >= self.config.confidence_threshold
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda detection: detection.confidence)

    def _is_same_target(
        self,
        detection: Detection,
        frame_width: int,
        frame_height: int,
    ) -> bool:
        if self._last_detection is None:
            return False
        old_x, old_y = self._last_detection.center()
        new_x, new_y = detection.center()
        normalized_distance = math.hypot(
            (new_x - old_x) / frame_width,
            (new_y - old_y) / frame_height,
        )
        return normalized_distance <= self.config.association_distance

    def _handle_missing_target(self) -> Decision:
        self._seen_streak = 0
        self._last_detection = None
        if not self._has_confirmed_target:
            self.state = VisionState.SEARCH
            return Decision(self.state, MotionCommand(), "未发现可信目标")

        self._miss_streak += 1
        if self._miss_streak >= self.config.lost_frames:
            self.state = VisionState.LOST
            self._has_confirmed_target = False
            return Decision(self.state, MotionCommand(), "目标持续丢失，保持停止")

        self.state = VisionState.STOP
        return Decision(
            self.state,
            MotionCommand(),
            f"目标短暂丢失 {self._miss_streak}/{self.config.lost_frames}，立即停止",
        )

    def _tracking_command(
        self,
        detection: Detection,
        frame_width: int,
        frame_height: int,
    ) -> MotionCommand:
        center_x, _ = detection.center()
        horizontal_error = (center_x - frame_width / 2.0) / (frame_width / 2.0)
        if abs(horizontal_error) <= self.config.center_deadband:
            horizontal_error = 0.0

        # 图像右侧目标对应机器人右转，因此 yaw 符号取反。
        vyaw = _clamp(
            -self.config.yaw_gain * horizontal_error,
            -self.config.max_yaw_speed,
            self.config.max_yaw_speed,
        )
        area_error = max(
            0.0,
            self.config.desired_area_ratio
            - detection.area_ratio(frame_width, frame_height),
        )
        alignment_scale = max(0.0, 1.0 - abs(horizontal_error))
        vx = _clamp(
            self.config.forward_gain * area_error * alignment_scale,
            0.0,
            self.config.max_forward_speed,
        )
        return MotionCommand(vx=vx, vy=0.0, vyaw=vyaw)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class Actuator(Protocol):
    def apply(self, command: MotionCommand) -> None: ...

    def stop(self) -> None: ...


class DryRunActuator:
    """只打印决策，不导入 SDK，也不连接机器人。"""

    def apply(self, command: MotionCommand) -> None:
        print(
            "  DRY-RUN 指令 "
            f"vx={command.vx:+.3f} m/s, vy={command.vy:+.3f} m/s, "
            f"vyaw={command.vyaw:+.3f} rad/s"
        )

    def stop(self) -> None:
        print("  DRY-RUN 停止：未向实机发送任何指令")


class Go2SportActuator:
    """Go2 执行器；实例只能在显式执行模式下创建。"""

    def __init__(self, sport_client: object) -> None:
        self._client = sport_client

    def apply(self, command: MotionCommand) -> None:
        if command.is_stop:
            self.stop()
        else:
            self._client.Move(command.vx, command.vy, command.vyaw)

    def stop(self) -> None:
        self._client.StopMove()


class ColorTargetDetector:
    """用彩色标志物产生 bbox 和置信度，适合现场教学。"""

    def __init__(
        self, target_color: str = "red", min_area_ratio: float = 0.003
    ) -> None:
        if target_color not in {"red", "blue"}:
            raise ValueError("target_color 只支持 red 或 blue")
        self.target_color = target_color
        self.min_area_ratio = min_area_ratio

    def detect(self, frame: object) -> list[Detection]:
        import cv2  # 延迟导入：离线状态机不依赖 OpenCV
        import numpy as np

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if self.target_color == "red":
            mask_a = cv2.inRange(hsv, np.array([0, 100, 80]), np.array([10, 255, 255]))
            mask_b = cv2.inRange(
                hsv, np.array([170, 100, 80]), np.array([180, 255, 255])
            )
            mask = cv2.bitwise_or(mask_a, mask_b)
        else:
            mask = cv2.inRange(hsv, np.array([95, 100, 70]), np.array([135, 255, 255]))

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []

        frame_height, frame_width = frame.shape[:2]
        frame_area = float(frame_width * frame_height)
        contour = max(contours, key=cv2.contourArea)
        contour_area = cv2.contourArea(contour)
        if contour_area / frame_area < self.min_area_ratio:
            return []

        x, y, width, height = cv2.boundingRect(contour)
        rectangle_area = max(1.0, float(width * height))
        fill_ratio = contour_area / rectangle_area
        size_score = min(1.0, contour_area / (frame_area * 0.04))
        confidence = _clamp(0.55 + 0.25 * fill_ratio + 0.20 * size_score, 0.0, 1.0)
        return [Detection((x, y, x + width, y + height), confidence, self.target_color)]


def synthetic_frames() -> Iterable[tuple[int, int, list[Detection]]]:
    """生成一段可复现的离线状态序列。"""

    width, height = 640, 480
    sequence: list[list[Detection]] = [
        [],
        [Detection((470, 170, 550, 280), 0.82)],
        [Detection((455, 165, 545, 285), 0.86)],
        [Detection((430, 155, 535, 290), 0.89)],
        [Detection((310, 135, 455, 320), 0.91)],
        [Detection((185, 80, 500, 410), 0.94)],
        [],
        [],
        [],
    ]
    for detections in sequence:
        yield width, height, detections


def run_synthetic(controller: VisionClosedLoopController, actuator: Actuator) -> None:
    print("离线演示：合成目标从画面右侧进入、靠近，随后丢失。")
    try:
        for index, (width, height, detections) in enumerate(
            synthetic_frames(), start=1
        ):
            decision = controller.update(detections, width, height)
            print(f"帧 {index:02d} | {decision.state.value:6s} | {decision.reason}")
            actuator.apply(decision.command)
    finally:
        actuator.stop()


def initialize_go2(
    interface: str, execute: bool
) -> tuple[object, Optional[Go2SportActuator]]:
    """延迟导入并初始化 Go2 视频客户端；按需初始化运动客户端。"""

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient
    except ImportError as exc:
        raise RuntimeError(
            "实时模式需要安装 unitree_sdk2py；离线演示请使用默认参数"
        ) from exc

    ChannelFactoryInitialize(0, interface)
    video_client = VideoClient()
    video_client.SetTimeout(3.0)
    video_client.Init()

    actuator: Optional[Go2SportActuator] = None
    if execute:
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        sport_client = SportClient()
        sport_client.SetTimeout(3.0)
        sport_client.Init()
        actuator = Go2SportActuator(sport_client)
        actuator.stop()
    return video_client, actuator


def run_go2(
    controller: VisionClosedLoopController,
    video_client: object,
    actuator: Actuator,
    detector: ColorTargetDetector,
    max_runtime: float,
    display: bool,
) -> None:
    import cv2  # 延迟导入
    import numpy as np

    started = time.monotonic()
    frame_index = 0
    try:
        while time.monotonic() - started < max_runtime:
            code, data = video_client.GetImageSample()
            if code != 0:
                raise RuntimeError(f"获取 Go2 图像失败，错误码 {code}")
            frame = cv2.imdecode(
                np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                actuator.apply(MotionCommand())
                continue

            frame_index += 1
            detections = detector.detect(frame)
            height, width = frame.shape[:2]
            decision = controller.update(detections, width, height)
            print(
                f"帧 {frame_index:05d} | {decision.state.value:6s} | {decision.reason}"
            )
            actuator.apply(decision.command)

            if display:
                for detection in detections:
                    x1, y1, x2, y2 = (int(value) for value in detection.bbox)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(
                    frame,
                    decision.state.value,
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("Go2 Vision Closed Loop", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    finally:
        actuator.stop()
        if display:
            cv2.destroyAllWindows()
        print("已发送停止指令；画面仅在内存中处理，未保存任何帧。")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        choices=("synthetic", "go2"),
        default="synthetic",
        help="synthetic 为离线数据；go2 为实时摄像头",
    )
    parser.add_argument("--interface", help="连接 Go2 的网卡名称，如 eth0")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许发送运动指令；默认仅 dry-run",
    )
    parser.add_argument(
        "--acknowledge-safety",
        metavar="READY",
        help="实机执行前必须显式填写 READY",
    )
    parser.add_argument("--target-color", choices=("red", "blue"), default="red")
    parser.add_argument("--max-runtime", type=float, default=30.0)
    parser.add_argument("--display", action="store_true", help="显示实时画面，不保存")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.max_runtime <= 0.0 or args.max_runtime > 120.0:
        raise ValueError("max-runtime 必须在 0~120 秒之间")
    if args.source == "go2" and not args.interface:
        raise ValueError("Go2 实时模式必须提供 --interface")
    if args.execute:
        if args.source != "go2":
            raise ValueError("禁止用合成数据控制实机；--execute 必须配合 --source go2")
        if args.acknowledge_safety != "READY":
            raise ValueError("实机执行前必须提供 --acknowledge-safety READY")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    controller = VisionClosedLoopController()
    if args.source == "synthetic":
        run_synthetic(controller, DryRunActuator())
        return 0

    video_client, real_actuator = initialize_go2(args.interface, args.execute)
    actuator: Actuator = (
        real_actuator if real_actuator is not None else DryRunActuator()
    )
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"实时视觉模式：{mode}；隐私默认开启，不保存视频帧。")
    detector = ColorTargetDetector(args.target_color)
    run_go2(
        controller, video_client, actuator, detector, args.max_runtime, args.display
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

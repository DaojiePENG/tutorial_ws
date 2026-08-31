"""Lesson 07 雷达安全控制器的 SDK 无关测试。"""

from __future__ import annotations

import math
import unittest

from tutorial_ws.demos.lesson07_lidar_safety_controller import (
    LidarSafetyConfig,
    LidarSafetyController,
    LidarSafetyState,
    MotionCommand,
    SdkMotionSink,
    sanitize_distance,
)


def fast_config(**changes: object) -> LidarSafetyConfig:
    values = {
        "stop_enter_m": 0.45,
        "stop_exit_m": 0.65,
        "slow_enter_m": 1.10,
        "slow_exit_m": 1.35,
        "side_turn_min_m": 0.80,
        "side_margin_m": 0.20,
        "clear_speed_mps": 0.20,
        "slow_speed_mps": 0.08,
        "turn_speed_radps": 0.30,
        "turn_delay_s": 0.40,
        "transition_hold_s": 0.20,
        "median_window": 1,
        "ema_alpha": 1.0,
        "min_valid_m": 0.05,
        "max_valid_m": 20.0,
        "invalid_stop_after": 3,
    }
    values.update(changes)
    return LidarSafetyConfig(**values)


class LidarSafetyControllerTests(unittest.TestCase):
    def test_distance_validation(self) -> None:
        for invalid in (None, math.nan, math.inf, -1.0, 0.0, 30.0, "bad"):
            self.assertIsNone(sanitize_distance(invalid))
        self.assertEqual(sanitize_distance(1.25), 1.25)

    def test_clear_slow_stop_turn_and_recovery(self) -> None:
        controller = LidarSafetyController(fast_config())
        self.assertEqual(
            controller.update(2.0, 1.0, 1.0, 0.0).state,
            LidarSafetyState.CLEAR,
        )

        # 进入 SLOW 需要稳定 0.2 秒。
        self.assertEqual(
            controller.update(1.0, 1.0, 1.0, 0.1).state,
            LidarSafetyState.CLEAR,
        )
        slowed = controller.update(1.0, 1.0, 1.0, 0.31)
        self.assertEqual(slowed.state, LidarSafetyState.SLOW)
        self.assertAlmostEqual(slowed.command.vx, 0.08)

        # 近障碍立即停车。
        stopped = controller.update(0.40, 1.50, 0.60, 0.40)
        self.assertEqual(stopped.state, LidarSafetyState.STOP)
        self.assertEqual(stopped.command.kind, "STOP")

        # 先停车 0.4 秒，再确认左侧持续空旷 0.2 秒，才进入 TURN。
        self.assertEqual(
            controller.update(0.40, 1.50, 0.60, 0.81).state,
            LidarSafetyState.STOP,
        )
        turning = controller.update(0.40, 1.50, 0.60, 1.02)
        self.assertEqual(turning.state, LidarSafetyState.TURN)
        self.assertGreater(turning.command.vyaw, 0.0)

        # 前方超过 STOP 退出阈值后先回到 SLOW，避免直接高速前进。
        self.assertEqual(
            controller.update(0.80, 1.50, 0.60, 1.10).state,
            LidarSafetyState.TURN,
        )
        recovered = controller.update(0.80, 1.50, 0.60, 1.31)
        self.assertEqual(recovered.state, LidarSafetyState.SLOW)

        # SLOW 的退出阈值高于进入阈值，1.2 m 时仍保持慢速。
        self.assertEqual(
            controller.update(1.20, 1.50, 0.60, 1.40).state,
            LidarSafetyState.SLOW,
        )
        controller.update(1.50, 1.50, 0.60, 1.50)
        clear = controller.update(1.50, 1.50, 0.60, 1.71)
        self.assertEqual(clear.state, LidarSafetyState.CLEAR)

    def test_three_invalid_front_samples_force_stop(self) -> None:
        controller = LidarSafetyController(fast_config())
        controller.update(2.0, 1.0, 1.0, 0.0)
        self.assertEqual(
            controller.update(math.nan, 1.0, 1.0, 0.1).state,
            LidarSafetyState.CLEAR,
        )
        self.assertEqual(
            controller.update(None, 1.0, 1.0, 0.2).state,
            LidarSafetyState.CLEAR,
        )
        decision = controller.update(math.inf, 1.0, 1.0, 0.3)
        self.assertEqual(decision.state, LidarSafetyState.STOP)
        self.assertEqual(decision.command.kind, "STOP")
        self.assertFalse(decision.front_valid)

    def test_median_filter_rejects_single_close_spike(self) -> None:
        controller = LidarSafetyController(
            fast_config(median_window=3, transition_hold_s=0.0)
        )
        controller.update(2.0, 1.0, 1.0, 0.0)
        controller.update(2.0, 1.0, 1.0, 0.1)
        decision = controller.update(0.20, 1.0, 1.0, 0.2)
        self.assertEqual(decision.front_m, 2.0)
        self.assertEqual(decision.state, LidarSafetyState.CLEAR)

    def test_invalid_side_data_cannot_authorize_turn(self) -> None:
        controller = LidarSafetyController(fast_config(transition_hold_s=0.0))
        controller.update(0.3, 1.5, 0.6, 0.0)
        # 连续无效帧使左侧缓存失效，右侧又太近，因此只能保持 STOP。
        controller.update(0.3, None, 0.6, 0.2)
        controller.update(0.3, None, 0.6, 0.4)
        decision = controller.update(0.3, None, 0.6, 0.6)
        self.assertEqual(decision.state, LidarSafetyState.STOP)
        self.assertEqual(decision.command.kind, "STOP")


class FakeSportClient:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def Move(self, vx: float, vy: float, vyaw: float) -> None:
        self.calls.append(("Move", vx, vy, vyaw))

    def StopMove(self) -> None:
        self.calls.append(("StopMove",))


class MotionSinkTests(unittest.TestCase):
    def test_sink_is_the_single_dispatch_point(self) -> None:
        fake = FakeSportClient()
        sink = SdkMotionSink(fake, resend_s=0.25)
        move = MotionCommand("MOVE", vx=0.1, vyaw=0.2)

        sink.send(move, 0.0)
        sink.send(move, 0.1)  # 限频，不重复发送同一命令。
        sink.send(MotionCommand("STOP"), 0.2)
        sink.safe_shutdown()

        self.assertEqual(
            fake.calls,
            [
                ("Move", 0.1, 0.0, 0.2),
                ("StopMove",),
                ("StopMove",),
            ],
        )


if __name__ == "__main__":
    unittest.main()

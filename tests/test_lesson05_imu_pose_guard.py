"""Lesson 05 IMU 姿态保护的 SDK 无关测试。"""

from __future__ import annotations

import math
import unittest

from tutorial_ws.demos.lesson05_imu_pose_guard import (
    ImuPoseGuard,
    PoseGuardConfig,
    PoseSafetyState,
    radians_to_degrees,
)


def fast_config(**changes: object) -> PoseGuardConfig:
    values = {
        "warn_enter_deg": 10.0,
        "warn_exit_deg": 7.0,
        "stop_enter_deg": 25.0,
        "stop_exit_deg": 18.0,
        "enter_hold_s": 0.5,
        "exit_hold_s": 0.8,
        "median_window": 1,
        "ema_alpha": 1.0,
        "invalid_stop_after": 3,
    }
    values.update(changes)
    return PoseGuardConfig(**values)


class ImuPoseGuardTests(unittest.TestCase):
    def test_sustained_tilt_hysteresis_and_hold_time(self) -> None:
        guard = ImuPoseGuard(fast_config())

        self.assertEqual(guard.update(0.0, 0.0, 0.0).state, PoseSafetyState.NORMAL)
        self.assertEqual(guard.update(12.0, 0.0, 0.1).state, PoseSafetyState.NORMAL)
        warn = guard.update(12.0, 0.0, 0.61)
        self.assertEqual(warn.state, PoseSafetyState.WARN)
        self.assertTrue(warn.changed)

        self.assertEqual(guard.update(30.0, 0.0, 0.7).state, PoseSafetyState.WARN)
        stopped = guard.update(30.0, 0.0, 1.21)
        self.assertEqual(stopped.state, PoseSafetyState.STOP)

        # 20° 已低于进入 STOP 的阈值，但尚未低于退出阈值 18°。
        self.assertEqual(guard.update(20.0, 0.0, 1.3).state, PoseSafetyState.STOP)
        self.assertEqual(guard.update(16.0, 0.0, 1.4).state, PoseSafetyState.STOP)
        self.assertEqual(guard.update(16.0, 0.0, 2.21).state, PoseSafetyState.WARN)

        self.assertEqual(guard.update(5.0, 0.0, 2.3).state, PoseSafetyState.WARN)
        self.assertEqual(guard.update(5.0, 0.0, 3.11).state, PoseSafetyState.NORMAL)

    def test_median_filter_rejects_one_frame_spike(self) -> None:
        guard = ImuPoseGuard(
            fast_config(median_window=3, ema_alpha=1.0, enter_hold_s=0.0)
        )
        guard.update(0.0, 0.0, 0.0)
        guard.update(0.0, 0.0, 0.1)
        decision = guard.update(60.0, 0.0, 0.2)
        self.assertEqual(decision.filtered_roll_deg, 0.0)
        self.assertEqual(decision.state, PoseSafetyState.NORMAL)

    def test_pitch_can_trigger_stop(self) -> None:
        guard = ImuPoseGuard(fast_config(enter_hold_s=0.0))
        decision = guard.update(0.0, -30.0, 0.0)
        self.assertEqual(decision.state, PoseSafetyState.STOP)
        self.assertEqual(decision.tilt_deg, 30.0)

    def test_repeated_invalid_samples_fail_safe_to_stop(self) -> None:
        guard = ImuPoseGuard(fast_config(enter_hold_s=0.0))
        guard.update(0.0, 0.0, 0.0)
        self.assertEqual(guard.update(math.nan, 0.0, 0.1).state, PoseSafetyState.NORMAL)
        self.assertEqual(guard.update(None, 0.0, 0.2).state, PoseSafetyState.NORMAL)
        decision = guard.update(None, None, 0.3)
        self.assertEqual(decision.state, PoseSafetyState.STOP)
        self.assertFalse(decision.sensor_valid)

    def test_radians_to_degrees(self) -> None:
        roll, pitch, yaw = radians_to_degrees((math.pi / 2, -math.pi, 0.0))
        self.assertAlmostEqual(roll, 90.0)
        self.assertAlmostEqual(pitch, -180.0)
        self.assertAlmostEqual(yaw, 0.0)


if __name__ == "__main__":
    unittest.main()

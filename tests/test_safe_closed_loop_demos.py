"""Lesson 06/08 安全演示的纯离线测试。"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载模块：{path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


vision = load_module(
    "lesson06_vision_closed_loop_demo",
    ROOT / "demos" / "lesson06_vision_closed_loop_demo.py",
)
g1 = load_module(
    "lesson08_g1_motion_arm_demo",
    ROOT / "demos" / "lesson08_g1_motion_arm_demo.py",
)


class VisionControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        config = vision.ControllerConfig(confirm_frames=3, lost_frames=2)
        self.controller = vision.VisionClosedLoopController(config)
        self.target = vision.Detection((470, 190, 540, 280), 0.90)

    def test_target_requires_consecutive_confirmation(self) -> None:
        first = self.controller.update([self.target], 640, 480)
        second = self.controller.update([self.target], 640, 480)
        third = self.controller.update([self.target], 640, 480)

        self.assertEqual(first.state, vision.VisionState.SEARCH)
        self.assertEqual(second.state, vision.VisionState.SEARCH)
        self.assertEqual(third.state, vision.VisionState.TRACK)
        self.assertTrue(first.command.is_stop)
        self.assertLess(third.command.vyaw, 0.0)
        self.assertGreaterEqual(third.command.vx, 0.0)

    def test_low_confidence_detection_never_moves(self) -> None:
        weak = vision.Detection((200, 150, 350, 330), 0.40)
        for _ in range(5):
            decision = self.controller.update([weak], 640, 480)
            self.assertEqual(decision.state, vision.VisionState.SEARCH)
            self.assertTrue(decision.command.is_stop)

    def test_large_bbox_forces_immediate_stop(self) -> None:
        close_target = vision.Detection((150, 100, 500, 400), 0.92)
        decision = self.controller.update([close_target], 640, 480)
        self.assertEqual(decision.state, vision.VisionState.STOP)
        self.assertTrue(decision.command.is_stop)

    def test_confirmed_target_loss_stops_then_becomes_lost(self) -> None:
        for _ in range(3):
            self.controller.update([self.target], 640, 480)
        first_miss = self.controller.update([], 640, 480)
        second_miss = self.controller.update([], 640, 480)

        self.assertEqual(first_miss.state, vision.VisionState.STOP)
        self.assertEqual(second_miss.state, vision.VisionState.LOST)
        self.assertTrue(first_miss.command.is_stop)
        self.assertTrue(second_miss.command.is_stop)

    def test_jump_to_different_bbox_restarts_confirmation(self) -> None:
        self.controller.update([self.target], 640, 480)
        self.controller.update([self.target], 640, 480)
        jumped = vision.Detection((20, 20, 80, 90), 0.95)
        decision = self.controller.update([jumped], 640, 480)

        self.assertEqual(decision.state, vision.VisionState.SEARCH)
        self.assertIn("1/3", decision.reason)

    def test_tracking_command_respects_velocity_limits(self) -> None:
        config = self.controller.config
        for _ in range(3):
            decision = self.controller.update([self.target], 640, 480)
        self.assertLessEqual(abs(decision.command.vx), config.max_forward_speed)
        self.assertLessEqual(abs(decision.command.vyaw), config.max_yaw_speed)
        self.assertEqual(decision.command.vy, 0.0)

    def test_execute_rejects_synthetic_source(self) -> None:
        parser = vision.build_parser()
        args = parser.parse_args(["--execute", "--acknowledge-safety", "READY"])
        with self.assertRaises(ValueError):
            vision.validate_args(args)


class RecordingAdapter:
    def __init__(self, fail_on_arm: bool = False) -> None:
        self.calls: list[tuple] = []
        self.fail_on_arm = fail_on_arm

    def move(self, vx: float, vy: float, vyaw: float) -> None:
        self.calls.append(("move", vx, vy, vyaw))

    def stop_move(self) -> None:
        self.calls.append(("stop",))

    def arm_action(self, action_id: int) -> None:
        self.calls.append(("arm", action_id))
        if self.fail_on_arm:
            raise RuntimeError("模拟动作失败")

    def release_arm(self) -> None:
        self.calls.append(("release",))


class G1DemoTests(unittest.TestCase):
    def test_every_named_plan_passes_safety_validation(self) -> None:
        for action in (
            "move-forward",
            "move-left",
            "turn-left",
            "hands-up",
            "wave",
            "handshake",
            "demo",
        ):
            with self.subTest(action=action):
                self.assertGreater(g1.validate_plan(g1.build_plan(action)), 0.0)

    def test_unknown_arm_action_is_rejected(self) -> None:
        plan = [g1.ArmActionStep("unverified-action", 1.0)]
        with self.assertRaises(g1.PlanValidationError):
            g1.validate_plan(plan)

    def test_unsafe_move_is_rejected(self) -> None:
        plan = [g1.MoveStep(0.30, 0.0, 0.0, 0.5)]
        with self.assertRaises(g1.PlanValidationError):
            g1.validate_plan(plan)

    def test_supported_control_states_are_explicit(self) -> None:
        self.assertEqual(g1.SUPPORTED_CONTROL_FSM_IDS, {500, 501, 801})

    def test_plan_stops_and_releases_after_success(self) -> None:
        adapter = RecordingAdapter()
        g1.execute_plan(
            g1.build_plan("move-forward"),
            adapter,
            sleep=lambda _: None,
        )
        self.assertIn(("move", 0.08, 0.0, 0.0), adapter.calls)
        self.assertEqual(adapter.calls[-2:], [("stop",), ("release",)])

    def test_finally_stops_and_releases_after_action_failure(self) -> None:
        adapter = RecordingAdapter(fail_on_arm=True)
        with self.assertRaises(RuntimeError):
            g1.execute_plan(g1.build_plan("wave"), adapter, sleep=lambda _: None)
        self.assertEqual(adapter.calls[-2:], [("stop",), ("release",)])

    def test_execute_mode_requires_interface_and_acknowledgement(self) -> None:
        parser = g1.build_parser()
        args = parser.parse_args(["--execute", "--action", "wave"])
        with self.assertRaises(ValueError):
            g1.validate_args(args)


if __name__ == "__main__":
    unittest.main()

# 安全闭环演示测试说明

本目录验证 Lesson 05–08 示例中的纯 Python 判断逻辑和安全门控。测试不会连接机器人，不需要 Unitree SDK，也不会发送运动指令。

## 运行全部测试

从项目根目录运行：

```bash
cd /home/daojie/Tutorial_All
python3 -m unittest discover -s tutorial_ws/tests -v
```

如果不希望测试产生 `__pycache__`，可使用：

```bash
PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s tutorial_ws/tests -v
```

## 运行单个测试文件

```bash
python3 -m unittest tutorial_ws.tests.test_lesson05_imu_pose_guard -v
python3 -m unittest tutorial_ws.tests.test_lesson07_lidar_safety_controller -v
python3 -m unittest tutorial_ws.tests.test_safe_closed_loop_demos -v
```

`test_safe_closed_loop_demos.py` 同时覆盖 Lesson 06 视觉控制器和 Lesson 08 G1 动作计划。

## 运行单个测试类或方法

只运行 Lesson 05 姿态保护测试类：

```bash
python3 -m unittest \
  tutorial_ws.tests.test_lesson05_imu_pose_guard.ImuPoseGuardTests -v
```

只运行一项雷达失效保护测试：

```bash
python3 -m unittest \
  tutorial_ws.tests.test_lesson07_lidar_safety_controller.LidarSafetyControllerTests.test_three_invalid_front_samples_force_stop \
  -v
```

只运行视觉速度限幅测试：

```bash
python3 -m unittest \
  tutorial_ws.tests.test_safe_closed_loop_demos.VisionControllerTests.test_tracking_command_respects_velocity_limits \
  -v
```

只运行 G1 异常清理测试：

```bash
python3 -m unittest \
  tutorial_ws.tests.test_safe_closed_loop_demos.G1DemoTests.test_finally_stops_and_releases_after_action_failure \
  -v
```

## 覆盖范围

### `test_lesson05_imu_pose_guard.py`

- 持续倾斜达到最小时间后才从 `NORMAL` 进入 `WARN/STOP`。
- WARN/STOP 的进入与退出阈值存在滞回，避免临界点抖动。
- 中值滤波能抑制单帧姿态尖峰。
- Roll 或 Pitch 任一方向均可触发保护。
- 连续无效 IMU 数据采用失效安全策略进入 `STOP`。
- RPY 弧度到角度的转换正确。

### `test_lesson07_lidar_safety_controller.py`

- None、NaN、无穷值、零值和量程外距离会被过滤。
- `CLEAR → SLOW → STOP → TURN → SLOW → CLEAR` 的状态切换、持续时间和滞回正确。
- 连续无效前向距离会触发 `STOP`。
- 中值滤波能抑制单帧过近毛刺。
- 侧向距离失效时不能授权转向。
- `SdkMotionSink` 是唯一命令分发点，并对重复指令限频。

### `test_safe_closed_loop_demos.py`：Lesson 06

- 目标必须连续出现指定帧数才允许进入 `TRACK`。
- 低置信度目标不能产生运动。
- 目标框过大时立即进入 `STOP`。
- 已确认目标短暂丢失先停止，持续丢失进入 `LOST`。
- 目标框发生大幅跳变时重新确认目标身份。
- 视觉速度命令不超过配置上限，且侧向速度为零。
- 合成数据与 `--execute` 的组合会被拒绝。

### `test_safe_closed_loop_demos.py`：Lesson 08

- 每个命名动作计划都能通过默认安全上限校验。
- 未知手臂动作和超限移动会被拒绝。
- 允许的 FSM ID 集合是明确、可测试的。
- 正常完成后会停车并释放手臂。
- 手臂动作异常时，`finally` 仍会停车并释放手臂。
- 实机执行缺少网卡或安全确认时会被拒绝。

## 测试不覆盖的内容

离线测试不能证明实机上的以下事项：

- 网卡、DDS 域、话题名称和字段映射是否与当前固件一致；
- IMU、摄像头和雷达的真实精度、时延、丢包和失效模式；
- SDK 动作 ID、FSM ID 与具体机器人版本是否兼容；
- 实际地面、负载、光照、反光物体和人员运动带来的风险；
- 急停、厂商安全机制和机械保护是否工作正常。

因此，通过单元测试只是进入只读实机验证的前提，不能直接视为允许执行动作。


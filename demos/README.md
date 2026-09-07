# Lesson 05–08 安全闭环演示说明

本目录提供四个可以直接运行的教学示例，内容从“读传感器”逐步过渡到“根据反馈生成受限动作”：

| 课程 | 机器人 | 反馈输入 | 核心问题 | 默认行为 |
| --- | --- | --- | --- | --- |
| Lesson 05 | Go2 | IMU Roll/Pitch | 姿态异常时如何预警和停车 | 回放内置数据，不导入 SDK |
| Lesson 06 | Go2 | 摄像头目标框 | 如何让机器人对准并接近目标 | 回放合成检测，不导入 SDK |
| Lesson 07 | Go2 | 雷达前/左/右距离 | 如何减速、停车和选择转向 | 回放内置距离，不导入 SDK |
| Lesson 08 | G1 | 动作计划与 FSM 预检 | 如何安全执行低速移动和手臂预设动作 | 打印计划，不导入 SDK |

所有示例都遵守同一个原则：**不带参数直接运行时，不会向机器人发送动作指令**。状态判断、滤波、限幅和计划校验尽量写成不依赖 Unitree SDK 的纯 Python 代码，因此可先在普通电脑上讲解和测试。

> 这些程序是课程演示和附加保护，不是经过安全认证的控制系统。它们不能替代厂商安全机制、遥控急停、限速、机械隔离、保护架和现场监护。

## 运行位置与依赖

以下命令均假定当前目录为项目根目录：

```bash
cd /home/daojie/Tutorial_All
```

四个示例的默认离线模式只需要 Python 3。实机模式还需要与机器人和固件匹配的 `unitree_sdk2py`：

- Lesson 05/07：需要 DDS 通信、Go2 状态或雷达消息类型；执行模式还需要 `SportClient`。
- Lesson 06：实时图像需要 Unitree `VideoClient`、OpenCV 和 NumPy；执行模式还需要 `SportClient`。
- Lesson 08：执行模式需要 G1 `LocoClient` 和 `G1ArmActionClient`。

SDK、网卡名、DDS 域和机器人网络配置应以现场设备为准。不要为了“验证是否连通”直接使用执行模式，先按“离线 → 只读/实时 dry-run → 低速执行”的顺序验证。

## 通用实机安全边界

在运行任何带 `--execute` 的命令前，至少完成以下检查：

1. 机器人电量、关节、网络和急停状态正常，遥控器在监护人员手中。
2. 地面平整、防滑，四周至少保留 2 m 净空，移除台阶、玻璃、镜面、细杆和移动人员。
3. 确认网卡名、数据单位、坐标方向和话题字段含义；不同固件或配置不能直接套用假设。
4. 首次测试使用最低速度、最短时长；G1 首次动作建议配合保护架。
5. 运行人员能随时按下急停；不要依赖程序的 `finally` 或 `StopMove()` 作为唯一安全手段。

## Lesson 05：IMU 姿态保护

文件：[`lesson05_imu_pose_guard.py`](lesson05_imu_pose_guard.py)

### 做什么

程序订阅 Go2 运动状态，读取 IMU 的 Roll/Pitch，用滤波、滞回和最小持续时间判断姿态风险。它展示了一个最小的反馈闭环：

```text
IMU R/P → 有效性检查 → 滑动中值 → EMA 低通 → 滞回状态机 → LED / StopMove
```

### 输入与输出

- 实机输入话题：`rt/sportmodestate`。
- 输入字段：`SportModeState_.imu_state.rpy`，SDK 原始单位为弧度，程序转换为度。
- 状态判断量：`max(abs(filtered_roll), abs(filtered_pitch))`。
- 控制台输出：时间、`NORMAL/WARN/STOP`、滤波后 Roll/Pitch、数据是否有效以及切换原因。
- `--execute` 输出：状态变化时可设置 VUI LED；`STOP` 时周期性调用 `SportClient.StopMove()`。

### 滤波和状态机

默认先取 5 帧滑动中值，再用 `alpha=0.40` 的 EMA 低通，减少单帧冲击造成的误判。

| 当前风险 | 进入条件 | 退出条件 | 默认输出 |
| --- | --- | --- | --- |
| `NORMAL` | 最大倾角持续达到 12° | — | LED 亮度 0，不发运动指令 |
| `WARN` | 最大倾角持续达到 12° | 最大倾角持续降到 8° | LED 亮度 5 |
| `STOP` | 最大倾角持续达到 25° | 最大倾角持续降到 18°以下，先退回较低风险状态 | LED 亮度 10，并调用 `StopMove()` |

风险增加默认需保持 0.35 s，风险解除默认需保持 0.80 s。进入阈值和退出阈值不同，避免机器人在临界角度附近来回切换。连续 5 帧无效数据会立即进入 `STOP`；超过 0.35 s 未收到新姿态，也会在控制循环中被视为无效数据。

### 常用命令

离线回放，安全默认值：

```bash
python3 tutorial_ws/demos/lesson05_imu_pose_guard.py
```

显式写出 dry-run，效果相同：

```bash
python3 tutorial_ws/demos/lesson05_imu_pose_guard.py --dry-run
```

只读观察实机 IMU，不创建运动和灯光客户端：

```bash
python3 tutorial_ws/demos/lesson05_imu_pose_guard.py \
  --observe --interface ens37 --domain 0 --duration 30
```

确认方向、单位和阈值后，才允许启用保护输出：

```bash
python3 tutorial_ws/demos/lesson05_imu_pose_guard.py \
  --execute --interface ens37 --domain 0 --duration 30
```

`--no-led` 可关闭 LED，`--no-stop` 会关闭 `StopMove()`，只保留观察或灯光提示。`--no-stop` 会削弱保护能力，不应用于依赖本程序停车的测试。

### 实机前提

先用 `--observe` 手动轻微改变姿态，确认 Roll/Pitch 的正负方向、弧度转换和零偏。执行模式启动时会先调用一次 `StopMove()`；退出或异常时也会再次停车。IMU 冲击、地面打滑和通信中断仍可能造成误判，因此本例只能作为附加保护。

## Lesson 06：视觉目标闭环

文件：[`lesson06_vision_closed_loop_demo.py`](lesson06_vision_closed_loop_demo.py)

### 做什么

程序使用红色或蓝色标志物的检测框演示目标跟随。控制器只依赖检测框、置信度和画面尺寸；OpenCV、NumPy 和 SDK 仅在实时模式延迟导入。

```text
图像 → HSV 颜色分割 → 目标框 → 连续帧确认 → 对准/接近 → Move 或停止
```

默认合成序列模拟目标从画面右侧进入、逐渐靠近、最后丢失。实时模式通过 `VideoClient.GetImageSample()` 读取 Go2 图像，帧只在内存中处理，默认不显示也不保存。

### 输入与输出

- 离线输入：程序内置的 `Detection(bbox, confidence, label)` 序列。
- 实时输入：Go2 摄像头 JPEG 数据，经 OpenCV 解码和红/蓝 HSV 阈值分割后生成目标框。
- 控制台输出：帧号、`SEARCH/TRACK/STOP/LOST`、判断原因和 dry-run 速度。
- 执行输出：`SportClient.Move(vx, 0, vyaw)`；停止命令转换为 `StopMove()`。
- `--display`：只显示实时检测画面，不保存帧；按 `Esc` 可退出。

### 状态机和安全边界

| 状态 | 条件 | 运动行为 |
| --- | --- | --- |
| `SEARCH` | 没有高置信度目标，或目标尚未连续确认 | 停止 |
| `TRACK` | 同一目标连续出现 3 帧 | 受限前进并调整偏航 |
| `STOP` | 目标面积占比达到 0.18，或已确认目标短暂丢失 | 立即停止 |
| `LOST` | 已确认目标连续丢失 3 帧 | 保持停止并清除跟踪状态 |

默认只接受置信度不低于 0.65 的目标；期望面积占比为 0.08；画面中心死区为归一化偏差 0.08。输出被限制在 `vx <= 0.15 m/s`、`abs(vyaw) <= 0.35 rad/s`，且不生成侧向速度。目标跳变过大时会重新进行连续帧确认。

执行模式必须同时满足以下三个条件：

1. `--source go2`，禁止用合成数据驱动实机；
2. 提供真实网卡 `--interface ...`；
3. 明确填写 `--acknowledge-safety READY`。

单次实时运行默认最多 30 s，`--max-runtime` 允许范围为 0–120 s。图像获取异常、程序退出或键盘中断均进入停止流程。

### 常用命令

默认合成数据 dry-run：

```bash
python3 tutorial_ws/demos/lesson06_vision_closed_loop_demo.py
```

读取 Go2 实时画面但只打印速度，不创建运动客户端：

```bash
python3 tutorial_ws/demos/lesson06_vision_closed_loop_demo.py \
  --source go2 --interface ens37 --target-color red --max-runtime 30
```

在上一步确认检测稳定后显示实时画面：

```bash
python3 tutorial_ws/demos/lesson06_vision_closed_loop_demo.py \
  --source go2 --interface ens37 --target-color blue \
  --max-runtime 30 --display
```

完成全部安全检查后才允许实机执行：

```bash
python3 tutorial_ws/demos/lesson06_vision_closed_loop_demo.py \
  --source go2 --interface ens37 --target-color red \
  --max-runtime 30 --execute --acknowledge-safety READY
```

### 实机前提

使用尺寸足够、颜色饱和且背景中不易混淆的红色或蓝色标志物。先在不同距离、光照和背景下运行实时 dry-run，观察置信度、面积占比和丢失状态。颜色分割并不是通用目标检测，强反光、相似颜色、遮挡和曝光变化都可能造成误检。

## Lesson 07：雷达安全避障

文件：[`lesson07_lidar_safety_controller.py`](lesson07_lidar_safety_controller.py)

### 做什么

程序把前、左、右三个方向的简化距离转换为一个受限运动命令，展示感知安全层如何包住前进命令：

```text
F/L/R 距离 → 无效值过滤 → 中值/EMA → 滞回状态机 → 单一命令出口
```

`LidarSafetyController` 只计算状态和 `MotionCommand`，不接触 SDK；`SdkMotionSink` 是执行模式中唯一调用 `Move/StopMove` 的位置，避免多个回调同时发令。

### 输入与输出

- 实机输入话题：`rt/utlidar/range_info`。
- 示例字段映射：`point.x = front`、`point.y = left`、`point.z = right`，单位假定为米。
- 控制台输出：时间、`CLEAR/SLOW/STOP/TURN`、滤波后的 F/L/R 距离、命令和原因。
- 执行输出：低速 `Move(vx, 0, 0)`、原地 `Move(0, 0, vyaw)` 或 `StopMove()`。

### 滤波、状态机和默认阈值

距离小于 0.05 m、大于 20 m、`None`、NaN 和无穷值会被过滤。有效距离先做 5 帧中值，再用 `alpha=0.45` 的 EMA 平滑。连续 3 帧前向数据无效，或超过 0.35 s 没有新消息，会进入保护停车。

| 状态 | 默认进入/退出条件 | 输出 |
| --- | --- | --- |
| `CLEAR` | 前方高于慢速区；从 `SLOW` 恢复需达到 1.35 m | `vx = 0.20 m/s` |
| `SLOW` | 前方降到 1.10 m；保持到超过 1.35 m | `vx = 0.08 m/s` |
| `STOP` | 前方降到 0.45 m，或数据失效 | `StopMove()` |
| `TURN` | 停车至少 0.60 s 后，一侧距离至少 0.80 m 且方向明确 | `abs(vyaw) = 0.30 rad/s` |

前方恢复到 0.65 m 才允许退出停车；非紧急状态切换还需稳定 0.25 s。左右距离差至少 0.20 m 才选更空旷一侧；方向不可靠时保持 `STOP`。进入 `STOP` 不等待普通切换时间。

### 常用命令

离线回放完整的 `CLEAR → SLOW → STOP → TURN → 恢复 → 数据失效` 序列：

```bash
python3 tutorial_ws/demos/lesson07_lidar_safety_controller.py
```

只读订阅实机雷达，不创建 `SportClient`：

```bash
python3 tutorial_ws/demos/lesson07_lidar_safety_controller.py \
  --observe --interface ens37 --domain 0 --duration 30
```

确认字段映射、量程和现场障碍后才允许执行：

```bash
python3 tutorial_ws/demos/lesson07_lidar_safety_controller.py \
  --execute --interface ens37 --domain 0 --duration 30
```

### 实机前提

不同固件或配置中的 `range_info` 字段含义可能不同。必须先用 `--observe` 分别遮挡前、左、右方向，确认 x/y/z 映射、单位、刷新率和失效值。该程序只有局部距离反应，没有定位、地图、路径规划、台阶识别或动态目标预测，不能当作完整导航系统。

## Lesson 08：G1 低速移动与手臂动作

文件：[`lesson08_g1_motion_arm_demo.py`](lesson08_g1_motion_arm_demo.py)

### 做什么

程序将 G1 演示拆成受限的有限动作计划。默认只打印计划；执行模式才创建 `LocoClient` 和 `G1ArmActionClient`。它不是传感器反馈状态机，而是“白名单计划 → 预检 → 逐步执行 → 无条件停止和释放”的安全执行链。

### 输入与输出

- 输入：命令行白名单 `--action`，包括 `move-forward`、`move-left`、`turn-left`、`wave`、`handshake` 和组合 `demo`。
- `demo` 计划：低速前进、停止、举手、挥手、握手、停止。
- 手臂白名单 ID：举手 15、挥手 25、握手 27；动作结束用 ID 99 释放预设动作。
- dry-run 输出：逐步打印 `Move`、`StopMove`、`ExecuteAction` 和跳过的等待时间。
- 执行输出：通过 G1 SDK 发送对应底盘与手臂命令。

### 计划校验和安全边界

| 项目 | 默认上限 |
| --- | ---: |
| `abs(vx)` | 0.10 m/s |
| `abs(vy)` | 0.08 m/s |
| `abs(vyaw)` | 0.15 rad/s |
| 单次移动 | 1.0 s |
| 单次手臂动作 | 8.0 s |
| 整体计划 | 20.0 s |

执行前会读取 G1 FSM ID，只接受代码中明确列出的 `{500, 501, 801}`。客户端建立或预检失败时先尝试停车；无论计划成功、异常、超时或中断，`finally` 都分别尝试 `StopMove()` 和释放手臂，避免一个操作失败后阻断另一个安全动作。

执行模式必须同时提供网卡和 `--acknowledge-safety READY`。SDK 超时参数只允许 1–10 s。

### 常用命令

默认组合计划 dry-run：

```bash
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py
```

分别检查移动或手臂计划：

```bash
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py --action move-forward
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py --action turn-left
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py --action wave
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py --action handshake
```

完成屏幕打印的启动检查后才允许实机执行：

```bash
python3 tutorial_ws/demos/lesson08_g1_motion_arm_demo.py \
  --action demo --interface ens37 \
  --execute --acknowledge-safety READY
```

### 实机前提

G1 必须按设备流程进入稳定站立和可编程控制状态，FSM ID 也必须满足程序预检。确认动作 ID 与当前 SDK/固件一致，周围人员退出手臂和身体扫掠范围。首次执行建议使用保护架，并先分别验证单个移动和单个手臂动作，再运行组合计划。

## 离线测试

完整测试命令、单项运行方法和覆盖范围见 [`../tests/README.md`](../tests/README.md)。最常用命令为：

```bash
python3 -m unittest discover -s tutorial_ws/tests -v
```


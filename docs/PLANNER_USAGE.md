# Spot 规则控制

从项目根目录构建并运行。默认机器人为 Spot，默认控制方式为 Rules；规则控制不需要模型权重或 LibTorch。

```bash
cmake -S . -B build -DUSE_TORCH=OFF
cmake --build build -j 6
./build/RobotSim
# 等同于：
./build/RobotSim robot/boston_dynamics_spot/scene.xml
```

Linux 或未安装 CycloneDDS 时，配置时增加 `-DUSE_DDS=OFF`。无需 DDS 也能使用窗口按钮和键盘控制。

## 操作

启动后等待约 2 秒归位。

| 操作 | 按钮 / 快捷键 |
| --- | --- |
| 前进 | Forward / W |
| 后退 | Backward / S |
| 左转、右转 | Turn Left / A、Turn Right / D |
| 停止并站稳 | Stop / X |
| 暂停、继续 | Pause / Run、空格 |
| 回到初始站立姿态 | Reset / Backspace |
| 规则 / 模型控制 | Rules / Policy |

`Rule gait speed` 下的 Scale 范围为 0.1–10.0，数值不是米/秒。0.5–10.0 调节步幅，半步时长保持 350 ms；低于 0.5 时，保留至少 4 cm 的前后足端行程，通过降低步频降速，避免极小步幅被接触形变和关节跟踪误差抵消。0.1 倍速的半步时长为 1750 ms。抬脚高度随速度在 2.1–3.5 cm 范围变化。

`Ground friction` 下的 Scale 范围为 0.05–2.0，是所有 geom 切向摩擦的倍率（默认 1.0 为模型原值）。MuJoCo 接触摩擦取双方 geom 的最大值，只调地板会被足底摩擦（0.8）覆盖，因此滑块整体等比缩放；调小可复现打滑场景，每个物理步生效。该值同时实时传入规划器，用于摩擦自适应步态：低摩擦下自动减小步幅封顶值，目标速度超过封顶时保持步幅、提高步频（支撑相最短 200 ms），超出可达范围的速度会自然饱和而不是摔倒，详见 `docs/SPEED_LIMIT_ANALYSIS.md`。

物理步长固定为 1 ms，速度变更在下一步轨迹生成时生效。停止会先完成当前迈步，再收回站立姿态，因此低速下停止响应也会更慢。

静止起步（含倾斜保护强制停止后的再次起步）有加速过程：实际速度从 1.0 倍按每秒 2.0 的速率爬升到滑块目标值，而不是一启动就按设定速度运动。例如目标 10 倍速时约 4.5 s 才爬满；行进中拖动滑块同样按该速率平滑过渡。

暂停时物理状态、归位进度和步态相位均停止推进。Reset 在物理线程内恢复 `home` 关键帧，清空规划器、跌倒保护和轨迹状态，并将运动命令恢复为站立。

Policy 保留现有模型推理入口，需要匹配的机器人策略和 `-DUSE_TORCH=ON`。本次修复和验证范围是 Spot 的规则控制；Go1/Go2 不使用 Spot 的规则参数。

## DDS 控制（可选）

```bash
./build/tools/dds_control_client basic
```

输入 `forward`、`backward`、`turn_left`、`turn_right` 或 `stand`。规则步态采用足端轨迹、逆运动学和姿态/航向反馈，不依赖学习模型。

## 验证

```bash
ctest --test-dir build --output-on-failure -j 3
# 独立运行 30 秒前进测试：
./build/HeadlessSim robot/boston_dynamics_spot/scene.xml forward 1 30
```

HeadlessSim 参数为 `场景路径 测试名称 速度倍率 秒数`；测试名称支持 `forward`、`backward`、`left`、`right`、`stand`、`stop`、`reset`、`speed`、`transitions`。时间范围为 15–300 秒。测试使用真实 MuJoCo 动力学，检查运动方向、姿态、关节范围、非有限数值和停止后的漂移；同时检查关节目标速度峰值 < 6 rad/s、加速度峰值 < 300 rad/s²、加速度 RMS < 35 rad/s²，失败返回非零退出码。`transitions` 连续执行前进、后退、左转、右转、前进、停止。

PlannerTest 检查足端跨相位连续性、支撑相末端姿态扰动、切换方向后的对角腿交替、控制接管时目标不跳变，以及暂停、时间回退、重置、不同调用频率、跌倒锁存和非法输入。

步态平滑性修正的测量与限制见 [步态检查记录](GAIT_REVIEW.md)。

验证范围为仓库默认平地场景的上述动作。跳跃、蹲伏和复杂地形不在本次验证范围内。

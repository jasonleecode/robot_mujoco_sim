# Planner模块集成说明

## 重构完成

`src/planner.h` 和 `src/planner.cpp` 现在作为对planner模块的封装接口，使用planner模块的Robot类和运动学模型来控制机器人。

## 主要改变

### 1. 使用planner模块的Robot类

- **之前**：直接使用简单的三角函数计算关节角度
- **现在**：使用planner模块的`Robot`类和运动学模型进行精确计算

### 2. 基于足端轨迹的步态生成

新的实现流程：
1. 定义标准站立位置的关节角度和足端位置
2. 基于步态相位计算足端的3D轨迹偏移
3. 使用planner模块的逆运动学（`jointAngleCompute`）计算关节角度
4. 自动应用关节限制（通过`enforceJointLim`）

### 3. 接口保持不变

- `planner.h`的公共接口保持不变
- `main.cpp`无需修改
- 完全向后兼容

## 代码结构

### planner.h
- 封装了planner模块的Robot类
- 保持了原有的公共接口（`SpotPlanner`类）
- 内部使用planner模块的功能

### planner.cpp
- 使用`Robot`类提供运动学模型
- 使用`Leg::getKinematics()->jointAngleCompute()`进行逆运动学求解
- 基于足端轨迹生成步态

## 关键实现细节

### 腿索引映射

由于SpotPlanner和Robot类的腿索引顺序不同，使用了映射：
- **SpotPlanner顺序**：0=FL, 1=FR, 2=RL, 3=RR
- **Robot legs数组顺序**：0=FR, 1=FL, 2=RR, 3=RL

### 足端轨迹计算

```cpp
// 基于步态相位计算足端偏移
Eigen::Vector3d foot_offset(0, 0, 0);
foot_offset.x() = stride_x * cos(2π * phase);  // 前后摆动
foot_offset.z() = lift_h * sin(π * phase / 0.5); // 抬腿
foot_offset.y() = turn_factor * side_sign * 0.02; // 转向

// 使用逆运动学计算关节角度
Eigen::Vector3d new_angles = leg->getKinematics()->jointAngleCompute(
    current_angles, foot_offset
);
```

### 标准站立位置

在构造函数中初始化标准站立位置：
1. 设置标准关节角度到Robot模型
2. 获取标准位置的足端位置
3. 用于后续步态计算的参考

## 优势

1. **精确的运动学模型**：使用真实的机器人参数和运动学
2. **自动关节限制**：planner模块自动处理关节角度限制
3. **可扩展性**：可以轻松添加更复杂的步态模式
4. **模块化设计**：planner模块作为独立库，易于维护和扩展

## 使用方式

使用方式与之前完全相同：

```cpp
SpotPlanner planner;
planner.reset();
planner.setMode(control::BasicMotion::kForward);

// 在主循环中
RobotState robot_state;
robot.getState(robot_state);
planner.update(robot_state);

std::vector<double> qref;
planner.getJointTargets(qref);
// 应用qref到机器人控制
```

## 注意事项

1. planner模块的Robot类使用Go2机器人的参数（LEN_BASE, LEN_HIP等）
2. 如果需要适配Spot机器人，可能需要调整这些参数
3. 当前实现使用简单的足端轨迹，可以进一步优化

## 下一步改进方向

1. **参数适配**：根据Spot机器人的实际参数调整planner模块的参数
2. **更复杂的步态**：实现更多步态模式（walk, gallop等）
3. **状态反馈**：使用实际机器人状态进行步态调整
4. **优化步态参数**：根据实际测试调整步幅、频率等参数


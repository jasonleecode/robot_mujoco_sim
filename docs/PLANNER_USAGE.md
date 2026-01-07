# Planner模块使用说明

## 如何让机器人向前运动

### 当前实现方式

1. **通过DDS发送控制命令**：
   - 使用工具发送DDS消息，设置：
     - `mode = kBasic` (CommandMode::kBasic = 1)
     - `action = kForward` (BasicMotion::kForward = 0)

2. **代码执行流程**：
   ```
   main.cpp接收到DDS命令
   ↓
   检测到mode=kBasic且action=kForward
   ↓
   planner.setMode(kForward)  // 设置planner为前进模式
   planner_drive = true        // 启用planner控制
   ↓
   主循环中：
   robot.getState(robot_state)    // 获取当前机器人状态
   planner.update(robot_state)    // 更新planner（计算步态相位）
   planner.getJointTargets(qref)  // 获取目标关节角度
   ↓
   将qref应用到shared_control.values
   ↓
   物理线程中应用控制量到机器人
   ```

### 代码位置

- **命令接收**：`src/main.cpp` 第183-209行
- **Planner更新**：`src/main.cpp` 第211-218行
- **步态生成**：`src/planner.cpp` 的 `getJointTargets()` 方法

### 当前步态实现

当前的`SpotPlanner`使用简单的三角函数生成trot步态（对角步态）：
- **步态频率**：1.5 Hz
- **步幅**：0.2弧度（可通过speed_multiplier_调整）
- **抬腿高度**：0.4弧度（可通过speed_multiplier_调整）
- **相位偏移**：[0.0, 0.5, 0.5, 0.0] 对应 [FL, FR, HL, HR]

### 使用Planner模块改进

planner模块提供了`Robot`类和运动学模型，可以用于更精确的控制：

#### 1. 使用运动学模型计算精确的足端轨迹

```cpp
#include "Quadruped/Robot.h"

class SpotPlanner {
    Robot robot_model_;  // planner模块的Robot类
    
    void getJointTargets(std::vector<double>& qref) {
        // 使用Robot类的运动学模型
        // 1. 定义足端轨迹（基于步态相位）
        // 2. 使用逆运动学计算关节角度
        // 3. 设置到robot_model_并获取角度
    }
};
```

#### 2. 使用planner模块的优势

- **精确的运动学模型**：使用真实的机器人参数
- **逆运动学求解**：可以基于足端位置计算关节角度
- **雅可比矩阵**：可以用于速度控制和力控制
- **关节限制检查**：自动处理关节角度限制

#### 3. 下一步改进方向

1. **集成Robot类**：在SpotPlanner中使用planner模块的Robot类
2. **基于足端轨迹的步态**：定义足端的3D轨迹，然后使用逆运动学计算关节角度
3. **更复杂的步态**：可以使用planner模块实现更多步态模式

### 注意事项

- 当前实现使用的是简单的关节空间控制（直接设置关节角度）
- planner模块的Robot类使用的是Go2机器人的参数，可能和Spot机器人有差异
- 如需使用planner模块的完整功能，需要适配机器人的参数（腿长、关节限制等）


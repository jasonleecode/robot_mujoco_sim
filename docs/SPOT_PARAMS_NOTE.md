# Spot机器人参数调整说明

## 当前实现状态

### ✅ 已完成

1. **Spot参数提取**：从`model/boston_dynamics_spot/spot.xml`提取了所有参数，保存在`src/spot_robot_params.h`
2. **使用实际机器人状态**：在`getJointTargets()`中使用实际机器人状态（`RobotState`）作为逆运动学的起点
3. **标准站立位置**：使用从spot.xml提取的标准站立角度（0, 1.04, -1.8）

### ⚠️ 当前限制

**Planner模块参数限制**：
- planner模块的Robot类使用硬编码宏定义（`planner/include/Quadruped/Joint.h`）
- 这些参数是为Go2机器人设计的，无法在运行时修改
- 参数差异较大（见参数对照表）

## 参数对照表

| 参数 | Go2 (planner模块) | Spot (实际) | 差异 |
|------|-------------------|-------------|------|
| LEN_BASE | 0.3868m | 0.5957m | +54% |
| LEN_HIP | 0.0955m | 0.055m | -42% |
| LEN_THIGH | 0.2130m | 0.32m | +50% |
| LEN_CALF | 0.23m | 0.3365m | +46% |

## 如何真正使用Spot参数

如果要让planner模块使用Spot参数，需要修改`planner/include/Quadruped/Joint.h`：

```cpp
// 原Go2参数
#define LEN_BASE 0.3868
#define LEN_HIP 0.0955
#define LEN_THIGH 0.2130
#define LEN_CALF 0.23

// 改为Spot参数
#define LEN_BASE 0.5957
#define LEN_HIP 0.055
#define LEN_THIGH 0.32
#define LEN_CALF 0.3365
```

**注意事项**：
- 这会影响所有使用planner模块的代码
- 如果项目中还有其他地方使用planner模块（用于其他机器人），需要谨慎

## 当前实现的优势

虽然planner模块使用Go2参数，但当前实现仍有优势：

1. **使用实际状态**：基于实际机器人状态计算，能够适应实际运动
2. **逆运动学求解**：使用planner模块的逆运动学算法
3. **自动关节限制**：通过planner模块自动应用关节限制
4. **实时调整**：每一步都基于当前实际状态计算下一步

## 推荐方案

### 方案A：修改planner模块参数（如果只用于Spot）

如果planner模块仅用于Spot机器人，可以直接修改宏定义。

### 方案B：保持当前实现（如果planner模块可能用于其他机器人）

当前实现虽然使用Go2参数，但通过使用实际状态进行步态调整，能够部分补偿参数差异。

### 方案C：创建Spot专用的运动学模块（长期方案）

创建一个独立的Spot运动学模块，不依赖planner模块的Robot类。

## 测试建议

1. 测试基本步态（前进、转向）
2. 观察运动是否稳定
3. 如果运动异常，考虑修改planner模块的参数


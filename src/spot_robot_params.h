#pragma once

/**
 * @file spot_robot_params.h
 * @brief Spot机器人参数定义（从model/boston_dynamics_spot/spot.xml提取）
 * 
 * 这些参数从MuJoCo模型文件中提取，用于planner模块的运动学计算
 */

namespace spot_params {

// 身体尺寸（从spot.xml提取）
// 前腿位置: pos="0.29785 0.055 0" (fl_hip) 和 pos="0.29785 -0.055 0" (fr_hip)
// 后腿位置: pos="-0.29785 0.055 0" (hl_hip) 和 pos="-0.29785 -0.055 0" (hr_hip)
// 注意：这些参数名称避免与planner模块的宏定义冲突（LEN_BASE等）
constexpr double SPOT_LEN_BASE = 0.5957;      // 身体长度（前后方向，0.29785 * 2）
constexpr double SPOT_WIDTH_BASE = 0.11;      // 身体宽度（左右方向，0.055 * 2）

// 腿的尺寸（从spot.xml的body结构提取）
// upper leg长度：从pos="0 0.1108 0"到pos="0.025 0 -0.32"，长度约为0.32米
// lower leg长度：从foot的pos="0 0 -0.3365"，长度约为0.3365米
// 注意：planner模块的LEN_HIP定义可能不同，这里使用0.055（从body到hip的Y方向距离）
constexpr double SPOT_LEN_HIP = 0.055;        // Hip偏移（从body中心到hip的Y方向距离）
constexpr double SPOT_LEN_THIGH = 0.32;       // 大腿长度（upper leg）
constexpr double SPOT_LEN_CALF = 0.3365;      // 小腿长度（lower leg + foot offset）

// 标准站立位置的关节角度（从spot.xml的keyframe "home"提取）
// qpos: "0 1.04 -1.8" 对应每个腿的 [hx, hy, kn]
constexpr double STAND_HX = 0.0;         // Hip X轴角度（侧摆）
constexpr double STAND_HY = 1.04;        // Hip Y轴角度（前后摆）
constexpr double STAND_KN = -1.8;        // Knee角度

// 关节角度范围（从spot.xml的joint range提取）
// hx: range="-0.785398 0.785398" (±45度)
// hy: range="-0.898845 2.29511" (约-51.5度到131.5度)
// kn: range="-2.7929 -0.254402" (约-160度到-14.6度)
constexpr double HX_MIN = -0.785398;
constexpr double HX_MAX = 0.785398;
constexpr double HY_MIN = -0.898845;
constexpr double HY_MAX = 2.29511;
constexpr double KN_MIN = -2.7929;
constexpr double KN_MAX = -0.254402;

} // namespace spot_params


// Deterministic integration check using the real Spot model and MuJoCo physics.
// Usage: HeadlessSim [scene.xml] [forward|backward|left|right|stand|stop|reset|speed|transitions] [scale] [seconds] [friction_scale]
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "planner.h"
#include "robot_config.h"

namespace {
constexpr double dt = 0.001;
double yaw(const mjData* d) {
  const auto* q = d->qpos + 3;
  return std::atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]));
}
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
void home(const mjModel* m, mjData* d) {
  int key = mj_name2id(m, mjOBJ_KEY, "home");
  if (key >= 0) mj_resetDataKeyframe(m, d, key);
  else mj_resetData(m, d);
  mj_forward(m, d);
}
}

int main(int argc, char** argv) {
  try {
    const std::string path = argc > 1 ? argv[1] : "robot/boston_dynamics_spot/scene.xml";
    const std::string scenario = argc > 2 ? argv[2] : "forward";
    const double scale = argc > 3 ? std::stod(argv[3]) : 1.0;
    const double seconds = argc > 4 ? std::stod(argv[4]) : 20.0;
    const double friction_scale = argc > 5 ? std::stod(argv[5]) : 1.0;
    require(std::isfinite(scale) && scale >= 0.1 && scale <= 10, "scale must be in [0.1, 10]");
    require(std::isfinite(seconds) && seconds >= 15 && seconds <= 300, "seconds must be in [15, 300]");
    require(std::isfinite(friction_scale) && friction_scale >= 0.01 && friction_scale <= 2.0,
            "friction_scale must be in [0.01, 2]");
    const std::vector<std::string> scenarios = {"forward", "backward", "left", "right", "stand", "stop", "reset", "speed", "transitions"};
    require(std::find(scenarios.begin(), scenarios.end(), scenario) != scenarios.end(), "unknown scenario");
    if (const char* plugins = std::getenv("MUJOCO_PLUGIN_DIR")) {
      mj_loadAllPluginLibraries(plugins, nullptr);
    } else if (const char* user = std::getenv("HOME")) {
      auto plugins = std::string(user) + "/.mujoco/mujoco-3.6.0/bin/mujoco_plugin";
      if (std::filesystem::exists(plugins)) mj_loadAllPluginLibraries(plugins.c_str(), nullptr);
    }
    char error[1000] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    require(m != nullptr, error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    auto cfg = detectRobotConfig(m.get(), path);
    require(cfg.supports_rule_gait && cfg.uses_position_ctrl, "this test requires the 12-actuator Spot position-control model");
    m->opt.timestep = dt;
    int floor = mj_name2id(m.get(), mjOBJ_GEOM, "floor");
    if (floor >= 0) m->geom_friction[3*floor] = 1.0;
    // 与 RobotSim 的 Ground friction 滑块一致：等比缩放所有 geom 的切向摩擦
    for (int i = 0; i < m->ngeom; ++i) m->geom_friction[3*i] *= friction_scale;
    home(m.get(), d.get());
    SpotPlanner planner;
    planner.setControlFrequency(dt);
    planner.setSpeedScale(scale);
    std::vector<double> ctrl = cfg.stand_angles;
    bool initialized = false;
    double min_height = 10, max_tilt = 0, heading = 0, previous_yaw = yaw(d.get());
    double start_x = 0, start_y = 0, tail_x = 0, tail_y = 0;
    double max_jump = 0;
    double max_speed = 0, max_accel = 0, accel_squared = 0;
    double min_walk_height = 10, max_walk_height = 0;
    std::vector<double> previous_velocity(m->nu, 0.0);
    int target_samples = 0;
    long long accel_samples = 0;
    for (int step = 0; step < static_cast<int>(seconds / dt); ++step) {
      const double elapsed = step * dt;
      if (scenario == "reset" && step == 8000) {
        home(m.get(), d.get());
        planner.reset();
        initialized = false;
        ctrl = cfg.stand_angles;
        target_samples = 0;
        heading = 0;
        previous_yaw = yaw(d.get());
      }
      RobotState state;
      state.time = d->time;
      state.qpos.assign(d->qpos, d->qpos + m->nq);
      state.qvel.assign(d->qvel, d->qvel + m->nv);
      state.imu_quat.assign(d->qpos + 3, d->qpos + 7);
      state.imu_gyro.assign(d->qvel + 3, d->qvel + 6);
      if (state.time >= 2.5) {
        if (!initialized) {
          planner.setCurrentState(state, ctrl);
          initialized = true;
          start_x = d->qpos[0]; start_y = d->qpos[1];
        }
        control::BasicMotion command = control::BasicMotion::kForward;
        if (scenario == "backward") command = control::BasicMotion::kBackward;
        if (scenario == "left") command = control::BasicMotion::kTurnLeft;
        if (scenario == "right") command = control::BasicMotion::kTurnRight;
        if (scenario == "stand" || (scenario == "stop" && elapsed >= 7.0)) command = control::BasicMotion::kStand;
        if (scenario == "transitions") {
          if (elapsed >= 5.1) command = control::BasicMotion::kBackward;
          if (elapsed >= 7.3) command = control::BasicMotion::kTurnLeft;
          if (elapsed >= 9.6) command = control::BasicMotion::kTurnRight;
          if (elapsed >= 11.8) command = control::BasicMotion::kForward;
          if (elapsed >= seconds-5.0) command = control::BasicMotion::kStand;
        }
        if (planner.mode() != command) planner.setMode(command);
        if (scenario == "speed") planner.setSpeedScale(elapsed < 5 ? 0.1 : elapsed < 9 ? 2.0 : 1.0);
        auto previous = ctrl;
        planner.update(state);
        planner.getJointTargets(ctrl);
        for (int i=0; i<m->nu; ++i) max_jump = std::max(max_jump, std::abs(ctrl[i]-previous[i]));
        for (int i=0; i<m->nu; ++i) {
          double velocity = (ctrl[i]-previous[i])/dt;
          // Initialization synchronizes to measured joints; measure trajectories
          // after that handoff, including start/stop/speed/phase transitions.
          if (target_samples >= 1) max_speed = std::max(max_speed, std::abs(velocity));
          if (target_samples >= 2) {
            double acceleration = (velocity-previous_velocity[i])/dt;
            max_accel = std::max(max_accel, std::abs(acceleration));
            accel_squared += acceleration*acceleration;
            ++accel_samples;
          }
          previous_velocity[i] = velocity;
        }
        ++target_samples;
      }
      require(ctrl.size() == static_cast<size_t>(m->nu), "wrong actuator target count");
      for (int i = 0; i < m->nu; ++i) {
        require(std::isfinite(ctrl[i]), "non-finite joint target");
        require(ctrl[i] >= cfg.joint_limits_low[i] && ctrl[i] <= cfg.joint_limits_high[i], "joint target outside model limits");
        d->ctrl[i] = ctrl[i];
      }
      mj_step(m.get(), d.get());
      for (int i=0; i<m->nq; ++i) require(std::isfinite(d->qpos[i]), "non-finite physics state");
      const double projection = 1-2*(d->qpos[4]*d->qpos[4]+d->qpos[5]*d->qpos[5]);
      double tilt = std::acos(std::clamp(projection, -1.0, 1.0));
      min_height = std::min(min_height, d->qpos[2]);
      max_tilt = std::max(max_tilt, tilt);
      if (state.time >= 4.0) {
        min_walk_height = std::min(min_walk_height, d->qpos[2]);
        max_walk_height = std::max(max_walk_height, d->qpos[2]);
      }
      require(!planner.isFallen() && tilt < 0.384 && d->qpos[2] > 0.32, "unstable posture at t=" + std::to_string(elapsed));
      double current_yaw = yaw(d.get());
      heading += std::atan2(std::sin(current_yaw-previous_yaw), std::cos(current_yaw-previous_yaw));
      previous_yaw = current_yaw;
      if (step == static_cast<int>((seconds-2.0)/dt)) { tail_x=d->qpos[0]; tail_y=d->qpos[1]; }
    }
    double dx=d->qpos[0]-start_x, dy=d->qpos[1]-start_y;
    double drift=std::hypot(d->qpos[0]-tail_x, d->qpos[1]-tail_y);
    printf("trajectory dx=%.4f dy=%.4f yaw=%.4f\n",dx,dy,heading);
    printf("smoothness peak_target_speed=%.3frad/s peak_target_accel=%.1frad/s^2 rms_target_accel=%.2frad/s^2 height_range=%.4fm\n",
      max_speed, max_accel, std::sqrt(accel_squared/std::max(1LL, accel_samples)), max_walk_height-min_walk_height);
    require(max_speed < 6.0, "joint target velocity spike");
    require(max_accel < 300.0, "joint target acceleration spike");
    require(std::sqrt(accel_squared/std::max(1LL, accel_samples)) < 35.0, "excessive joint target acceleration RMS");
    if (scenario=="forward" || scenario=="reset" || scenario=="speed" || scenario=="stop") require(dx > 0.15, "did not travel forward");
    if (scenario=="backward") require(dx < -0.15, "did not travel backward");
    if (scenario=="left") require(heading > 0.3, "did not turn left");
    if (scenario=="right") require(heading < -0.3, "did not turn right");
    if (scenario=="stand" || scenario=="stop" || scenario=="transitions") require(drift < 0.02, "did not settle after stop");
    for (int i=0; i<mjNWARNING; ++i) require(d->warning[i].number == 0, "MuJoCo reported a physics warning");
    printf("PASS %s scale=%.2f duration=%.1fs dx=%.3fm dy=%.3fm yaw=%.3frad min_height=%.3fm max_tilt=%.2fdeg final_2s_drift=%.4fm max_target_step=%.4frad\n",
      scenario.c_str(), scale, seconds, dx, dy, heading, min_height, max_tilt*180/3.141592653589793, drift, max_jump);
    return 0;
  } catch (const std::exception& error) {
    fprintf(stderr, "FAIL: %s\n", error.what());
    return 1;
  }
}

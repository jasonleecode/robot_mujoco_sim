// Headless physics simulation for diagnosing forward gait behavior.
// Loads MuJoCo model, runs SpotPlanner, prints body trajectory.
#include <mujoco/mujoco.h>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cmath>

#include "planner.h"
#include "ControlActions.hpp"
#include "robot_config.h"

template <typename T>
T lerp(T a, T b, double t) { return a + (b - a) * t; }

int main(int argc, char** argv) {
    const char* model_path = (argc > 1) ? argv[1]
                                        : "robot/boston_dynamics_spot/scene.xml";

    // Load OBJ/STL decoder plugins so mesh assets can be parsed
    mj_loadAllPluginLibraries(
        (std::string(getenv("HOME") ? getenv("HOME") : "") +
         "/.mujoco/mujoco-3.6.0/bin/mujoco_plugin").c_str(), nullptr);

    char err[1000];
    mjModel* m = mj_loadXML(model_path, nullptr, err, sizeof(err));
    if (!m) { fprintf(stderr, "Load error: %s\n", err); return 1; }

    mjData* d = mj_makeData(m);

    // Load home keyframe
    int key_id = mj_name2id(m, mjOBJ_KEY, "home");
    if (key_id >= 0) mj_resetDataKeyframe(m, d, key_id);
    else mj_resetData(m, d);

    // Set floor friction to 1.0 (same as RobotSim.cpp)
    int floor_geom = mj_name2id(m, mjOBJ_GEOM, "floor");
    if (floor_geom >= 0) m->geom_friction[floor_geom * 3] = 1.0;

    RobotConfig cfg = detectRobotConfig(m, model_path);
    int nu = m->nu;

    SpotPlanner planner;
    planner.setControlFrequency(0.001);

    const double kDt = 0.001;
    m->opt.timestep = kDt;

    // Homing phase
    std::vector<double> spawn_q(nu), ctrl(nu);
    for (int i = 0; i < nu; ++i) spawn_q[i] = d->qpos[7 + i];
    const auto& stand = cfg.stand_angles;

    const double homing_duration = 2.0;
    double sim_time = 0.0;

    printf("Starting headless simulation...\n");
    printf("T=%.3f  body_x=%.4f  body_z=%.4f  (homing)\n", sim_time, d->qpos[0], d->qpos[2]);

    bool homing_done = false;
    bool forward_triggered = false;
    bool planner_active = false;

    for (int step = 0; step < 10000; ++step) {
        sim_time = step * kDt;

        RobotState state;
        state.time = sim_time;
        state.qpos.assign(d->qpos, d->qpos + m->nq);
        state.qvel.assign(d->qvel, d->qvel + m->nv);

        // IMU from qpos quaternion
        state.imu_quat = {d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]};
        state.imu_gyro = {d->qvel[3], d->qvel[4], d->qvel[5]};

        if (!homing_done) {
            double t = sim_time / homing_duration;
            if (t < 1.0) {
                double s = t * t * (3 - 2 * t);
                for (int i = 0; i < nu; ++i) ctrl[i] = lerp(spawn_q[i], stand[i], s);
            } else {
                homing_done = true;
                planner.setCurrentState(state);
                printf("T=%.3f  Homing complete. body_x=%.4f body_z=%.4f\n",
                       sim_time, d->qpos[0], d->qpos[2]);
            }
        }

        if (homing_done) {
            // Trigger forward after 0.5s
            if (!forward_triggered && sim_time >= homing_duration + 0.5) {
                forward_triggered = true;
                planner_active = true;
                planner.setCurrentState(state);
                planner.setMode(control::BasicMotion::kForward);
                printf("T=%.3f  Forward triggered. body_x=%.4f\n", sim_time, d->qpos[0]);
            }

            if (planner_active) {
                if (planner.mode() != control::BasicMotion::kForward)
                    planner.setMode(control::BasicMotion::kForward);
                planner.update(state);
                planner.getJointTargets(ctrl);
            } else {
                ctrl = stand;
            }
        }

        // Apply control
        for (int i = 0; i < nu; ++i) d->ctrl[i] = ctrl[i];
        mj_step(m, d);

        // Print body position every 100ms
        if (step % 100 == 0 && step >= 2000) {
            double qw=d->qpos[3], qx=d->qpos[4], qy=d->qpos[5], qz=d->qpos[6];
            double roll  = atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)) * 57.3;
            double sinp  = 2*(qw*qy-qz*qx);
            double pitch = (fabs(sinp)>=1) ? copysign(90,sinp) : asin(sinp)*57.3;
            double fl_hy_act = d->qpos[7+1];
            double fr_hy_act = d->qpos[7+4];
            double rl_hy_act = d->qpos[7+7];
            double rr_hy_act = d->qpos[7+10];
            printf("T=%.3f  bx=%.4f by=%.4f bz=%.4f  R=%.1f P=%.1f"
                   "  hy_act(FL,FR,RL,RR)=(%.3f,%.3f,%.3f,%.3f)\n",
                   sim_time, d->qpos[0], d->qpos[1], d->qpos[2], roll, pitch,
                   fl_hy_act, fr_hy_act, rl_hy_act, rr_hy_act);
        }
    }

    printf("\n=== Final State ===\n");
    printf("body_x=%.4f  body_y=%.4f  body_z=%.4f\n", d->qpos[0], d->qpos[1], d->qpos[2]);
    double bx_start = 0.0;  // body starts at x=0
    double total_time = (10000 - 2000) * kDt;  // time after forward trigger started (approx)
    printf("Approx velocity: %.4f m/s (over last %.1fs)\n",
           (d->qpos[0] - bx_start) / 10.0, 10.0);

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}

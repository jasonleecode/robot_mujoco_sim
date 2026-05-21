// Headless planner test: verifies joint target directions during forward gait.
// Build: g++ -std=c++17 -o /tmp/planner_test src/planner_test.cpp
//        (not suitable as a standalone build – use CMake target below)
#include "planner.h"
#include "ControlActions.hpp"
#include <cstdio>
#include <vector>
#include <cmath>

int main() {
    SpotPlanner planner;
    planner.setControlFrequency(0.001);

    // Build a standing state: body at (0,0,0.46), identity orientation, all joints at (0,1.04,-1.8)
    RobotState state;
    state.time = 0.0;
    // qpos: x,y,z, qw,qx,qy,qz, then 12 joint angles
    state.qpos.resize(19, 0.0);
    state.qpos[0] = 0.0;  // body x
    state.qpos[1] = 0.0;  // body y
    state.qpos[2] = 0.46; // body z
    state.qpos[3] = 1.0;  // qw (upright)
    state.qpos[4] = 0.0;  state.qpos[5] = 0.0;  state.qpos[6] = 0.0;
    // joints: FL, FR, RL, RR each (hx, hy, kn)
    for (int i = 0; i < 12; ++i) {
        int j = i % 3;
        if (j == 0) state.qpos[7+i] = 0.0;
        else if (j == 1) state.qpos[7+i] = 1.04;
        else state.qpos[7+i] = -1.8;
    }
    // qvel: 6 body dof + 12 joint velocities
    state.qvel.resize(18, 0.0);
    // imu: [w,x,y,z]
    state.imu_quat = {1.0, 0.0, 0.0, 0.0};

    planner.setCurrentState(state);

    // Print initial qTarg
    std::vector<double> qref(12);
    planner.getJointTargets(qref);
    printf("Initial qTarg (FL,FR,RL,RR) hy: %.4f %.4f %.4f %.4f\n",
           qref[1], qref[4], qref[7], qref[10]);

    // Trigger forward motion
    planner.setMode(control::BasicMotion::kForward);

    // Run for 500 steps (500ms)
    double prev_fl_hy = qref[1];
    double prev_fr_hy = qref[4];
    double prev_rl_hy = qref[7];
    double prev_rr_hy = qref[10];

    int last_print = -1;
    for (int step = 0; step < 800; ++step) {
        state.time = step * 0.001;
        planner.update(state);
        planner.getJointTargets(qref);

        // Print every 50 steps
        if (step % 50 == 0 || step < 5) {
            printf("t=%.3f  FL_hy=%.4f(d=%.4f)  FR_hy=%.4f(d=%.4f)"
                   "  RL_hy=%.4f(d=%.4f)  RR_hy=%.4f(d=%.4f)\n",
                   state.time,
                   qref[1], qref[1]-prev_fl_hy,
                   qref[4], qref[4]-prev_fr_hy,
                   qref[7], qref[7]-prev_rl_hy,
                   qref[10], qref[10]-prev_rr_hy);
            prev_fl_hy = qref[1];
            prev_fr_hy = qref[4];
            prev_rl_hy = qref[7];
            prev_rr_hy = qref[10];
        }
    }

    printf("\nFinal qTarg:\n");
    printf("  FL: hx=%.4f hy=%.4f kn=%.4f\n", qref[0], qref[1], qref[2]);
    printf("  FR: hx=%.4f hy=%.4f kn=%.4f\n", qref[3], qref[4], qref[5]);
    printf("  RL: hx=%.4f hy=%.4f kn=%.4f\n", qref[6], qref[7], qref[8]);
    printf("  RR: hx=%.4f hy=%.4f kn=%.4f\n", qref[9], qref[10], qref[11]);
    return 0;
}

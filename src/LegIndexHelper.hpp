#pragma once

struct LegIndexHelper {
    // SpotPlanner leg index (FL=0, FR=1, RL=2, RR=3) -> MuJoCo qpos offset
    static int toMuJoCoOffset(int spot_leg) {
        const int offsets[4] = {0, 3, 6, 9}; // FL, FR, RL, RR
        return (spot_leg >= 0 && spot_leg < 4) ? offsets[spot_leg] : 0;
    }
    
    // SpotPlanner leg index -> Robot legs array index
    static int toRobotIndex(int spot_leg) {
        const int map[4] = {1, 0, 3, 2}; // FL->1, FR->0, RL->3, RR->2
        return (spot_leg >= 0 && spot_leg < 4) ? map[spot_leg] : 0;
    }
    
    // Convert full angle array from MuJoCo order to Robot order
    static std::vector<double> mujocoToRobotOrder(const std::vector<double>& mujoco_angles) {
        if (mujoco_angles.size() != 12) return mujoco_angles;
        
        std::vector<double> robot_angles(12);
        // MuJoCo: FL(0-2), FR(3-5), RL(6-8), RR(9-11)
        // Robot:  FR(0-2), FL(3-5), RR(6-8), RL(9-11)
        
        // FR <- MuJoCo FR
        std::copy(mujoco_angles.begin() + 3, mujoco_angles.begin() + 6, robot_angles.begin());
        // FL <- MuJoCo FL
        std::copy(mujoco_angles.begin(), mujoco_angles.begin() + 3, robot_angles.begin() + 3);
        // RR <- MuJoCo RR
        std::copy(mujoco_angles.begin() + 9, mujoco_angles.begin() + 12, robot_angles.begin() + 6);
        // RL <- MuJoCo RL
        std::copy(mujoco_angles.begin() + 6, mujoco_angles.begin() + 9, robot_angles.begin() + 9);
        
        return robot_angles;
    }
};
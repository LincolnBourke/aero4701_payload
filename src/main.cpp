#include "stewartPlatform.hpp"
#include "payload.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Quaternionf;

int main() {
    Payload payload; 
    payload.readTrajectory();
    payload.printTrajectory();

    // // test class 
    // StewartPlatform stewart_platform;

    // // Define rotation about the z axis
    // Eigen::AngleAxisf rot(-48.7 * M_PI / 180, Vector3f::UnitZ());
    // Quaternionf q(rot);

    // PlatformPose target_pose{Vector3f(0.0, 0.0, 15.0), q}; 
    // target_pose.orientation.normalize();

    // stewart_platform.moveTo(&target_pose);

    // std::array<float, NUM_SERVOS> servo_targets = stewart_platform.getServoTargets();

    // for (int i = 0; i < NUM_SERVOS; i++)
    //     std::cout << servo_targets[i] * 180 / M_PI << ", ";
    // std::cout << std::endl;
}
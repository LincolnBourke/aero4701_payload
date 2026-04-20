#include "stewartPlatform.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Quaternionf;

int main() {
    // test class 
    StewartPlatform stewart_platform;

    // Define rotation about the z axis
    Eigen::AngleAxisf rot(0 * M_PI / 180, Vector3f::UnitZ());
    Quaternionf q(rot);

    PlatformPose target_pose{Vector3f(0.0, 0.0, 10.0), q}; 
    target_pose.orientation.normalize();

    stewart_platform.moveTo(&target_pose);

    const std::array<float, NUM_SERVOS> servo_targets = stewart_platform.getServoTargets();

    for (int i = 0; i < NUM_SERVOS; i++)
        std::cout << servo_targets[i] << ", ";
    std::cout << std::endl;

    // Try to recover the platform pose from the leg lengths
    std::cout << "True position: " << target_pose.position.transpose() << std::endl;
    std::cout << "True rotation: " << -48.7 * M_PI / 180 << std::endl;
    bool success = stewart_platform.computePlatformPosition();
    std::cout << "Pose calculation successful: " << success << std::endl;

}
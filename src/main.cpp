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

    PlatformPose target_pose{Vector3f(0.0, 0.0, 1.0), q}; 
    target_pose.orientation.normalize();

    stewart_platform.moveTo(&target_pose);

    const std::array<float, NUM_SERVOS> servo_targets = stewart_platform.getServoTargets();

    std::cout << "Calculated servo targets:" << std::endl;
    for (int i = 0; i < NUM_SERVOS; i++)
        std::cout << servo_targets[i] << ", ";
    std::cout << std::endl << std::endl;

    // Try to recover the platform pose from the leg lengths
    bool success = stewart_platform.computePlatformPosition();
    PlatformPose calculated_pose = stewart_platform.getPlatformPose();
    std::cout << "Pose calculation successful: " << success << std::endl;
    std::cout << "Calculated position: " << calculated_pose.position.transpose() << std::endl;
    std::cout << "Calculated orientation (w, x, y, z): " 
        << calculated_pose.orientation.w() << ", "
        << calculated_pose.orientation.x() << ", "
        << calculated_pose.orientation.y() << ", "
        << calculated_pose.orientation.z() << 
        std::endl;

}
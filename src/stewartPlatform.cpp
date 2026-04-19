#include "stewartPlatform.hpp"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define ZERO_TOL 1e-6   // Tolerance for floating point zero checking
#define NR_TOL 1e-6     // Tolerance for Newton-Raphson solver 

// ***** Stewart Platform Geometry *****

// Distance between platform origin and the bottom of the platform 
#define PLATFORM_Z_OFFSET 3.60 // mm

// Distance between the base origin and the top of the servo brace 
#define BASE_Z_OFFSET 9.4248 // mm

#define LOWER_ARM_LENGTH 18.5 // mm
#define UPPER_ARM_LENGTH 27 // mm 

// Radii of the circles that the base / platform anchor points lie on
#define BASE_ANCHOR_RADIUS 32.7015 // mm
#define PLATFORM_ANCHOR_RADIUS 33.023 // mm

// Half the angular separation between anchor points on the same face
#define BASE_ANCHOR_ANGULAR_OFFSET 0.1581 // rad
#define PLATFORM_ANCHOR_ANGULAR_OFFSET 0.2877 // rad

using Eigen::Matrix3f;

StewartPlatform::StewartPlatform()
    : lower_arm_length(LOWER_ARM_LENGTH),
    upper_arm_length(UPPER_ARM_LENGTH),
    servo_angular_pos{3.0/2.0 * M_PI, 1.0/2.0 * M_PI, 1.0/6.0 * M_PI, 7.0/6.0 * M_PI, 5.0/6.0 * M_PI, 11.0/6.0 * M_PI},
    platform_pose_target{Vector3f::Zero(), Quaternionf::Identity()},
    platform_pose{Vector3f::Zero(), Quaternionf::Identity()},
    servo_targets{0},
    servos{ServoMotor()}
{
    // Calculate anchor point positions as vectors 
    for (int i = 0; i < NUM_SERVOS; i += 2)
    {
        // Angle between anchor points
        float centre_ang = M_PI / 3 * i; 

        // Base anchor angles
        float lower_base_ang = centre_ang - BASE_ANCHOR_ANGULAR_OFFSET;
        float upper_base_ang = centre_ang + BASE_ANCHOR_ANGULAR_OFFSET;

        // Platform anchor angles
        float lower_plat_ang = centre_ang - PLATFORM_ANCHOR_ANGULAR_OFFSET;
        float upper_plat_ang = centre_ang + PLATFORM_ANCHOR_ANGULAR_OFFSET;

        base_anchors[i] = BASE_ANCHOR_RADIUS * Vector3f(std::cos(lower_base_ang), std::sin(lower_base_ang), 0);
        base_anchors[i+1] = BASE_ANCHOR_RADIUS * Vector3f(std::cos(upper_base_ang), std::sin(upper_base_ang), 0);
        platform_anchors[i] = PLATFORM_ANCHOR_RADIUS * Vector3f(std::cos(lower_plat_ang), std::sin(lower_plat_ang), 0);
        platform_anchors[i+1] = PLATFORM_ANCHOR_RADIUS * Vector3f(std::cos(upper_plat_ang), std::sin(upper_plat_ang), 0);
        
        // std::cout << "Base Anchor " << i << ": " << base_anchors[i].transpose() << std::endl;
        // std::cout << "Base Anchor " << i+1 << ": " << base_anchors[i+1].transpose() << std::endl;
        // std::cout << "Platform Anchor " << i << ": " << platform_anchors[i].transpose() << std::endl;
        // std::cout << "Platform Anchor " << i+1 << ": " << platform_anchors[i+1].transpose() << std::endl;
    }
};

StewartPlatform::~StewartPlatform(){};

const std::array<float, NUM_SERVOS>& StewartPlatform::getServoTargets() const
{
    return servo_targets;
};

const PlatformPose& StewartPlatform::getPlatformPose() const
{
    return platform_pose;
}

bool StewartPlatform::moveTo(PlatformPose* target_pose)
{
    bool successful_calculation = true;

    // Check platform does not intersect servo brace 
    if (target_pose->position[2] < 0)
    {
        std::cout << "Error: platform target z position must be > 0." << std::endl;
        successful_calculation = false;
    }
    else 
    {
        // Update internal pose target
        platform_pose_target.position = target_pose->position;
        platform_pose_target.orientation = target_pose->orientation;
        platform_pose_target.position[2] += (BASE_Z_OFFSET + PLATFORM_Z_OFFSET);

        successful_calculation = computeServoTargets();
    }

    return successful_calculation;
};

// Implements the inverse kinematics for the Stewart platform
bool StewartPlatform::computeServoTargets()
{
    bool successful_calculation = true;

    // Compute angle for each servo
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        // Compute distance between base and platform arm anchor points
        Vector3f effective_arm = platform_pose_target.position + platform_pose_target.orientation * platform_anchors[i] - base_anchors[i]; 
        float effective_arm_length = effective_arm.norm();

        if (effective_arm_length > lower_arm_length + upper_arm_length)
        {
            std::cout << "Error: maximum effective arm length exceeded." << std::endl;
            successful_calculation = false;
            break;
        }        

        // Intermediate values for servo rotation
        float e = 2 * lower_arm_length * effective_arm[2];
        float f = 2 * lower_arm_length * (std::cos(servo_angular_pos[i]) * effective_arm[0] + std::sin(servo_angular_pos[i]) * effective_arm[1]);
        float g = std::pow(effective_arm_length, 2) - std::pow(upper_arm_length, 2) + std::pow(lower_arm_length, 2);
        float e2 = e * e;
        float f2 = f * f;

        float denominator = std::sqrt(e2 + f2);
        float ratio = g / denominator;

        // Error checking
        if (denominator < ZERO_TOL)  
        {
            std::cout << "Error: servo target calculation: denominator is zero." << std::endl;
            successful_calculation = false;
            break;
        }
        else if (ratio < -1.0 || ratio > 1.0)
        {
            std::cout << "Error: servo target calculation: arcsin argument out of bounds. ";
            std::cout << "Argument = " << ratio << std::endl;
            successful_calculation = false;
            break;
        }
        else if (std::abs(f) < ZERO_TOL && std::abs(e) < ZERO_TOL)
        {
            std::cout << "Error: servo target calculation: arctan2 argument(s) out of bounds." << std::endl;
            successful_calculation = false;
            break;
        }

        // compute servo rotation corresponding to effective arm length 
        servo_targets[i] = std::asin(ratio) - std::atan2(f, e);
    }

    return successful_calculation;
};

// Implements the forward kinematics for the Stewart platform 
bool StewartPlatform::computePlatformPosition()
{   
    // Get the roll, pitch, yaw of the last calculated platform pose.
    // (0, 1, 2) argument ordering extracts in order of [roll, pitch, yaw]
    Vector3f euler_angles = platform_pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    // Initial guess for the platform pose - stored as [x, y, z, roll, pitch, yaw]
    Eigen::Matrix<float, NUM_SERVOS, 1> pose_guess; 
    pose_guess << 
        platform_pose.position[0],
        platform_pose.position[1], 
        platform_pose.position[2], 
        euler_angles[0],
        euler_angles[1], 
        euler_angles[2];
    
    // Get the arm lengths from the servo angles 

    // Iteratively solve for the platform pose using the Newton-Raphson method 
    while (true)
    {
        // System of equations to solve is A<pose correction vector> = b
        Eigen::Matrix<float, NUM_SERVOS, NUM_SERVOS> A;
        Eigen::Matrix<float, NUM_SERVOS, 1> b;

        float c_roll = std::cos(euler_angles[0]);
        float s_roll = std::sin(euler_angles[0]);
        float c_pitch = std::cos(euler_angles[1]);
        float s_pitch = std::sin(euler_angles[1]);
        float c_yaw = std::cos(euler_angles[2]);
        float s_yaw = std::sin(euler_angles[2]);

        // Compute the base to platform rotation matrix 
        Matrix3f rot_base_to_platform;
        rot_base_to_platform <<
            c_roll * c_pitch, c_roll * s_pitch * s_yaw - s_roll * c_yaw, c_roll * s_pitch * c_yaw + s_roll * s_yaw,
            s_roll * c_pitch, s_roll * s_pitch * s_yaw + c_roll * c_yaw, s_roll * s_pitch * c_yaw - c_roll * s_yaw,
            -s_pitch, c_pitch * s_yaw, c_pitch * c_yaw;

        for (int i = 0; i < NUM_SERVOS; i++)
        {
            Vector3f platform_position(pose_guess[0], pose_guess[1], pose_guess[2]);

            // Position of the platform origin relative to the base anchor point
            Vector3f x = platform_position - base_anchors[i];
            
            // 
            Vector3f u = rot_base_to_platform * platform_anchors[i];

            // Compute the effective arm length for the current pose estimate
            Vector3f effective_arm = x + platform_anchors[i];
            float arm_length = effective_arm.norm();

            // Define the equation to solve that gives the effective arm length for one arm
            float f = std::pow(x(0) + u(0), 2) + std::pow(x(1) + u(1), 2) + std::pow(x(2) + u(2), 2) - std::pow(arm_length, 2); // = 0

            // Compute the derivative of f with respect to each element of the platform pose estimate
            A(i,0) = 2 * (x(0) + u(0));
            A(i,1) = 2 * (x(1) + u(1));
            A(i,2) = 2 * (x(2) + u(2));
            A(i,3) = 2 * (-x(0) * u(1) + x(1) * u(0));
            A(i,4) = 2 * ((-x(0) * c_roll + x(1) * s_roll) * u(2) - (platform_anchors[i](0) * c_pitch + platform_anchors[i](1) * s_pitch * s_yaw) * x(2));
            A(i,5) = 2 * platform_anchors[i](1) * (x(0) * rot_base_to_platform(1,3) + x(1) * rot_base_to_platform(2,3) + x(2) * rot_base_to_platform(3,3));

            b(i) = -f; 
        }

        // Check if sum(|b|) < tolerance to finalise solution
        float b_sum = 0.0f;
        for (int i = 0; i < NUM_SERVOS; i++)
            b_sum = b_sum + std::abs(b(i));

        if (b_sum < NR_TOL)
            break;
        
        // Update the platform pose estimate
        Eigen::Matrix<float, NUM_SERVOS, 1> pose_update = A.partialPivLu().solve(b);
        pose_guess = pose_guess + pose_update;

        // Check if the magnitude of the update elements were below a threshold
        float update_sum = 0.0f;
        for (int i = 0; i < NUM_SERVOS; i++)
            update_sum = update_sum + std::abs(pose_update(i));

        if (update_sum < NR_TOL)
            break;
    }

    // Update the internal representation of the pose 
    platform_pose.position = pose_guess.head(3);
    platform_pose.orientation = 
        Eigen::AngleAxisf(pose_guess(3), Vector3f::UnitX()) * 
        Eigen::AngleAxisf(pose_guess(4), Vector3f::UnitY()) * 
        Eigen::AngleAxisf(pose_guess(5), Vector3f::UnitZ());

    return true;
};
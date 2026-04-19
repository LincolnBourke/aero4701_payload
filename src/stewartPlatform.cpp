#include "stewartPlatform.hpp"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#define ZERO_TOL 1e-6           // Tolerance for floating point zero checking

// Vertical distance between platform / base origin and first point of contact 
// if a collision were to occur 
#define PLATFORM_Z_OFFSET 3.60  // in mm
#define BASE_Z_OFFSET 9.4248    // in mm

StewartPlatform::StewartPlatform()
{
    // Define geometry using CAD model
    lower_arm_length = 18.5;
    upper_arm_length = 27;
    
    servo_angular_pos = {3.0/2.0 * M_PI, 1.0/2.0 * M_PI, 1.0/6.0 * M_PI, 7.0/6.0 * M_PI, 5.0/6.0 * M_PI, 11.0/6.0 * M_PI};

    // Calculate anchor points
    float base_radius = 32.7015;
    float platform_radius = 33.023;
    for (int i = 0; i < NUM_SERVOS; i += 2)
    {
        // Angle between anchor points
        float centre_ang = M_PI / 3 * i; 

        // Base anchor angles
        float lower_base_ang = centre_ang - 0.1581;
        float upper_base_ang = centre_ang + 0.1581;

        // Platform anchor angles
        float lower_plat_ang = centre_ang - 0.2877;
        float upper_plat_ang = centre_ang + 0.2877;

        base_anchors[i] = base_radius * Vector3f(std::cos(lower_base_ang), std::sin(lower_base_ang), 0);
        base_anchors[i+1] = base_radius * Vector3f(std::cos(upper_base_ang), std::sin(upper_base_ang), 0);
        platform_anchors[i] = platform_radius * Vector3f(std::cos(lower_plat_ang), std::sin(lower_plat_ang), 0);
        platform_anchors[i+1] = platform_radius * Vector3f(std::cos(upper_plat_ang), std::sin(upper_plat_ang), 0);
        
        // std::cout << "Base Anchor " << i << ": " << base_anchors[i].transpose() << std::endl;
        // std::cout << "Base Anchor " << i+1 << ": " << base_anchors[i+1].transpose() << std::endl;
        // std::cout << "Platform Anchor " << i << ": " << platform_anchors[i].transpose() << std::endl;
        // std::cout << "Platform Anchor " << i+1 << ": " << platform_anchors[i+1].transpose() << std::endl;
    }

    // Initialise the true platform pose 
    // TODO: update with a calculation of the true pose
    platform_pose.position = Vector3f(0, 0, 0);
    platform_pose.orientation = Quaternionf(1, 0, 0, 0);
    
    // Initialise the target platform pose
    platform_pose_target.position = Vector3f(0, 0, 0);
    platform_pose_target.orientation = Quaternionf(1, 0, 0, 0);
    servo_targets = {0, 0, 0, 0, 0, 0};
};

StewartPlatform::~StewartPlatform(){};

std::array<float, NUM_SERVOS> StewartPlatform::getServoTargets()
{
    return servo_targets;
};


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
    

    return false;
};
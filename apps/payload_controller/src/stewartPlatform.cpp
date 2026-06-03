#include "stewartPlatform.hpp"
#include "servo_targets_t.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <random>

#define ZERO_TOL 1e-6   // Tolerance for floating point zero checking
#define NR_TOL 1e-5     // Tolerance for Newton-Raphson solver 

// ***** Stewart Platform Geometry *****

// Distance between platform origin and the bottom of the platform 
#define PLATFORM_Z_OFFSET 8.0 // 3.60 // mm

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
    servos{ServoMotor()},
    upper_servo_limits{M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2, M_PI / 2},
    lower_servo_limits{-M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4, -M_PI / 4}
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
    }

    if (!lcm.good())
    {
        std::cout << "[ERROR] StewartPlatform LCM object not good." << std::endl;
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

bool StewartPlatform::moveTo(const PlatformPose &target_pose)
{
    // Update internal pose target
    platform_pose_target.position = target_pose.position;
    platform_pose_target.orientation = target_pose.orientation;
    platform_pose_target.position[2] += (BASE_Z_OFFSET + PLATFORM_Z_OFFSET);

    // Attempt to calculate the servo targets for the pose target
    if (computeServoTargets(true) == false)
        return false;

    // Publish targets for servo controller to use
    publishServoTargets();

    return true;
};

bool StewartPlatform::getAnglesForMove(PlatformPose target_pose, std::array<float, NUM_SERVOS>* angles)
{
    bool successful_calculation = true;
    
    // Store the current pose and servo targets to restore them at the end 
    PlatformPose saved_pose_target = platform_pose_target;
    std::array<float, NUM_SERVOS> saved_servo_targets = servo_targets;

    // Check platform does not intersect servo brace 
    if (target_pose.position[2] < 0)
    {
        std::cout << "Error: platform target z position must be > 0." << std::endl;
        successful_calculation = false;
    }
    else 
    {
        // Set the new pose target
        platform_pose_target.position = target_pose.position;
        platform_pose_target.orientation = target_pose.orientation;
        platform_pose_target.position[2] += (BASE_Z_OFFSET + PLATFORM_Z_OFFSET);

        // Compute servo targets into the internal servo_targets array
        successful_calculation = computeServoTargets(true);

        // Copy out the computed angles if successful
        if (successful_calculation && angles != nullptr)
            *angles = servo_targets;
    }

    // Restore the previous pose and servo targets so internal state is unchanged
    platform_pose_target = saved_pose_target;
    servo_targets = saved_servo_targets;

    return successful_calculation;
}

// Implements the inverse kinematics for the Stewart platform
bool StewartPlatform::computeServoTargets(bool print_errors)
{
    bool successful_calculation = true;

    // Check platform does not intersect servo brace 
    if (platform_pose_target.position[2] < 0)
    {
        if (print_errors)
            std::cout << "Error: platform target z position must be >= 0." << std::endl;
        
        successful_calculation = false;
        return successful_calculation;
    }

    // Compute angle for each servo
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        // Compute distance between base and platform arm anchor points
        Vector3f effective_arm = platform_pose_target.position + platform_pose_target.orientation * platform_anchors[i] - base_anchors[i]; 
        float effective_arm_length = effective_arm.norm();

        if (effective_arm_length > lower_arm_length + upper_arm_length)
        {
            if (print_errors)
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
            if (print_errors)
                std::cout << "Error: servo target calculation: denominator is zero." << std::endl;
            
            successful_calculation = false;
            break;
        }
        else if (ratio < -1.0 || ratio > 1.0)
        {
            if (print_errors)
            {
                std::cout << "Error: servo target calculation: arcsin argument out of bounds. ";
                std::cout << "Argument = " << ratio << std::endl;
            }
            
            successful_calculation = false;
            break;
        }
        else if (std::abs(f) < ZERO_TOL && std::abs(e) < ZERO_TOL)
        {
            if (print_errors)
                std::cout << "Error: servo target calculation: arctan2 argument(s) out of bounds." << std::endl;
            
            successful_calculation = false;
            break;
        }

        // compute servo rotation corresponding to effective arm length 
        float servo_target = std::asin(ratio) - std::atan2(f, e);

        // Check if the servo angle is within the joint limits
        if (servo_target > upper_servo_limits[i] || servo_target < lower_servo_limits[i])
        {   
            if (print_errors)
                std::cout << "Error: joint limits exceeded." << std::endl; 

            successful_calculation = false; 
            break;
        }

        // 
        if (computeAngularSkew(i, servo_target) > (15.0f * M_PI / 180))
        {
            if (print_errors)
                std::cout << "Error: Angular skew exceeded." << std::endl;
            
            // successful_calculation = false; 
            // break;
        }

        servo_targets[i] = servo_target;
    }

    return successful_calculation;
};

float StewartPlatform::computeAngularSkew(int servo_num, float servo_angle)
{
    // Angles of the servo horns when viewed from above
    float c_above = std::cos(servo_angular_pos[servo_num]);
    float s_above = std::sin(servo_angular_pos[servo_num]);

    // Angles of the servo horns from the horizontal
    float c_hor = std::cos(servo_angle);
    float s_hor = std::sin(servo_angle);

    // Vector along the servo horn (starting from motor axis)
    Vector3f bottom_arm = lower_arm_length * Vector3f(c_above * c_hor, s_above * c_hor, s_hor);

    // Get the vector along the top arm
    Vector3f effective_arm = platform_pose_target.position + platform_pose_target.orientation * platform_anchors[servo_num] - base_anchors[servo_num];
    Vector3f top_arm = effective_arm - bottom_arm;

    // Compute the vector normal to the plane 
    Vector3f normal = Vector3f{c_above, s_above, 0};
    Eigen::AngleAxisf rotation(-M_PI/2, Eigen::Vector3f::UnitZ()); // Rotation about the z-axis
    normal = rotation * normal;

    // Angle between top arm and normal 
    float ang = std::asin(std::abs(top_arm.dot(normal)) / (top_arm.norm() * normal.norm()));

    return ang;
}

/* Implements the forward kinematics for the Stewart platform.
    Uses Newton-Raphson method and is sensitive to the initial condition.
    Uses the last target pose of the stewart platform as the initial condition.
*/  
bool StewartPlatform::computePlatformPose()
{   
    bool successful_calculation = false;

    // Get the roll, pitch, yaw of the last target platform pose.
    // (0, 1, 2) argument ordering extracts in order of [roll, pitch, yaw]
    Vector3f euler_angles = platform_pose_target.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    
    // Initial guess for the platform pose - stored as [x, y, z, roll, pitch, yaw]
    Eigen::Matrix<float, NUM_SERVOS, 1> pose_guess; 
    pose_guess << 
        platform_pose_target.position[0],
        platform_pose_target.position[1], 
        platform_pose_target.position[2], 
        euler_angles[0],
        euler_angles[1], 
        euler_angles[2];
    
    // Get the vector along each servo horn
    std::array<Vector3f, NUM_SERVOS> h; 
    for (int i = 0; i < NUM_SERVOS; i++) 
    {
        // Angles of the servo horns when viewed from above 
        float c_above = std::cos(servo_angular_pos[i]);
        float s_above = std::sin(servo_angular_pos[i]);
        
        // Angles of the servo horns from the horizontal 
        float c_hor = std::cos(servos[i].getAngle());
        float s_hor = std::sin(servos[i].getAngle());

        // Vector along the servo horn (starting from motor axis)
        h[i] = lower_arm_length * Vector3f(c_above * c_hor, s_above * c_hor, s_hor);
    }

    // Iteratively solve for the platform pose using the Newton-Raphson method
    // Stop at a maximum number of iterations or when the pose correction is sufficiently small
    int max_iterations = 500; 
    for (int iters = 0; iters < max_iterations; iters++)
    {
        // System of equations to solve is A<pose correction vector> = b
        Eigen::Matrix<float, NUM_SERVOS, NUM_SERVOS> A;
        Eigen::Matrix<float, NUM_SERVOS, 1> b;
        
        float c_roll = std::cos(pose_guess[3]);
        float s_roll = std::sin(pose_guess[3]);
        float c_pitch = std::cos(pose_guess[4]);
        float s_pitch = std::sin(pose_guess[4]);
        float c_yaw = std::cos(pose_guess[5]);
        float s_yaw = std::sin(pose_guess[5]);
        
        // Compute the base to platform rotation matrix 
        Matrix3f rot_base_to_platform;
        rot_base_to_platform <<
            c_roll * c_pitch, c_roll * s_pitch * s_yaw - s_roll * c_yaw, c_roll * s_pitch * c_yaw + s_roll * s_yaw,
            s_roll * c_pitch, s_roll * s_pitch * s_yaw + c_roll * c_yaw, s_roll * s_pitch * c_yaw - c_roll * s_yaw,
            -s_pitch, c_pitch * s_yaw, c_pitch * c_yaw;
        
        for (int i = 0; i < NUM_SERVOS; i++)
        {
            Vector3f platform_position(pose_guess[0], pose_guess[1], pose_guess[2]);

            // Position of the platform origin relative to the base anchor point (in base frame)
            Vector3f x = platform_position - base_anchors[i];
            
            // Position of the platform anchor point relative to the platform origin (in base frame)
            Vector3f p = rot_base_to_platform * platform_anchors[i];

            // Define the equation to solve - minimising gives the platform pose
            float f = (x + p - h[i]).squaredNorm() - upper_arm_length * upper_arm_length; // = 0
            
            float f_1 = x(0) + p(0) - h[i](0);
            float f_2 = x(1) + p(1) - h[i](1);
            float f_3 = x(2) + p(2) - h[i](2);

            // Compute the derivative of f with respect to each element of the platform pose estimate
            A(i,0) = 2 * (x(0) + p(0) - h[i](0));
            A(i,1) = 2 * (x(1) + p(1) - h[i](1));
            A(i,2) = 2 * (x(2) + p(2) - h[i](2));
            A(i,3) = 2 * (-x(0) * p(1) + x(1) * p(0) + h[i](0) * p(1) - h[i](1) * p(0));
            A(i,4) = 2 * (f_1 * c_roll * p(2) + f_2 * s_roll * p(2) + f_3 * -(platform_anchors[i](0) * c_pitch + platform_anchors[i](1) * s_pitch * s_yaw));
            A(i,5) = 2 * platform_anchors[i](1) * (f_1 * rot_base_to_platform(0,2) + f_2 * rot_base_to_platform(1,2) + f_3 * rot_base_to_platform(2,2));

            b(i) = -f; 
        }

        // Check if sum(|b|) < tolerance to finalise solution
        float b_sum = 0.0f;
        for (int i = 0; i < NUM_SERVOS; i++)
            b_sum = b_sum + std::abs(b(i));

        if (b_sum < NR_TOL)
        {
            successful_calculation = true;
            break;
        }

        // Update the platform pose estimate
        Eigen::Matrix<float, NUM_SERVOS, 1> pose_update = A.fullPivLu().solve(b);
        pose_guess = pose_guess + pose_update;

        // Break if the pose guess has become invalid
        if (!pose_guess.allFinite())
        {
            std::cout << "Error: platform pose calculation contains NaN." << std::endl;
            break;
        }

        // Check if the magnitude of the update elements were below a threshold
        float update_sum = 0.0f;
        for (int i = 0; i < NUM_SERVOS; i++)
            update_sum = update_sum + std::abs(pose_update(i));

        if (update_sum < NR_TOL)
        {
            successful_calculation = true;    
            break;
        }
    }

    // Update the internal representation of the pose 
    if (successful_calculation)
    {
        platform_pose.position = pose_guess.head(3);
        platform_pose.orientation = 
            Eigen::AngleAxisf(pose_guess(3), Vector3f::UnitZ()) * 
            Eigen::AngleAxisf(pose_guess(4), Vector3f::UnitY()) * 
            Eigen::AngleAxisf(pose_guess(5), Vector3f::UnitX());

        // Ensure the quaternion is normalised
        platform_pose.orientation.normalize();
    }

    return successful_calculation;
};

/*** Stewart Platform Analyser methods ****************************************/

void StewartPlatformAnalyser::generatePointCloud(std::string file_path)
{
    std::vector<PlatformPose> point_cloud;

    // Define the range of each position parameter to iterate over
    std::array<float, 2> p_x_range   = {-40.0f, 40.0f};
    std::array<float, 2> p_y_range   = {-40.0f, 40.0f};
    std::array<float, 2> p_z_range   = {0.0f, 50.0f};

    // Define the position steps to search over 
    float pos_step = 2;

    // Perform a grid search through the parameters 
    for (float x = p_x_range[0]; x <= p_x_range[1]; x += pos_step)
        for (float y = p_y_range[0]; y <= p_y_range[1]; y += pos_step)
            for (float z = p_z_range[0]; z <= p_z_range[1]; z += pos_step)
                {
                    // Construct a platform position from the parameters
                    platform_pose_target.position = Vector3f(x, y, z);
                    platform_pose_target.position[2] += (BASE_Z_OFFSET + PLATFORM_Z_OFFSET);
                    
                    // Search across the range of angles for a valid orientation
                    bool success = searchAcrossOrientations();
                    if (success)
                    {
                        platform_pose_target.position[2] -= (BASE_Z_OFFSET + PLATFORM_Z_OFFSET);
                        point_cloud.push_back(platform_pose_target);
                    }
                }
    
    savePointCloud(point_cloud, file_path);
}

bool StewartPlatformAnalyser::searchAcrossOrientations()
{
    // Define the range of each angle parameter to iterate over
    std::array<float, 2> roll_range  = {-M_PI / 4, M_PI / 4};
    std::array<float, 2> pitch_range = {-M_PI / 4, M_PI / 4};
    std::array<float, 2> yaw_range   = {-M_PI / 4, M_PI / 4};

    // Define the angle steps to search over 
    float ang_step = M_PI / 8;

    for (float roll = roll_range[0]; roll <= roll_range[1] + ZERO_TOL; roll += ang_step)
        for (float pitch = pitch_range[0]; pitch <= pitch_range[1] + ZERO_TOL; pitch += ang_step)
            for (float yaw = yaw_range[0]; yaw <= yaw_range[1] + ZERO_TOL; yaw += ang_step)
                {
                    // Construct a platform orientation from the parameters
                    Eigen::AngleAxisf rollAngle(roll,   Eigen::Vector3f::UnitX());
                    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
                    Eigen::AngleAxisf yawAngle(yaw,     Eigen::Vector3f::UnitZ());
                    
                    platform_pose_target.orientation = yawAngle * pitchAngle * rollAngle;
                    
                    // Check whether the platform pose can be achieved 
                    bool success = computeServoTargets(false);
                    if (success)
                    {
                        return true;
                    }
                }
    
    return false;
}

void StewartPlatformAnalyser::savePointCloud(std::vector<PlatformPose>& point_cloud, std::string file_path)
{
    std::ofstream file(file_path);

    if (!file.is_open())
    {
        std::cout << "Failed to open file for writing: " << file_path << std::endl;
        return;
    }

    for (const auto& pose : point_cloud)
    {
        file << pose.position.x() << ","
             << pose.position.y() << ","
             << pose.position.z() << ","
             << pose.orientation.x() << ","
             << pose.orientation.y() << ","
             << pose.orientation.z() << ","
             << pose.orientation.w() << "\n";
    }

    file.close();
}

// --- LCM publisher methods ---------------------------------------------------

void StewartPlatform::publishServoTargets()
{
    // Populate and publish the servo targets message
    payload_messages::servo_targets_t msg;

    for (int i = 0; i < NUM_SERVOS; i++)
    {
        msg.angles[i] = servo_targets[i];
    }

    // Print the channel and each servo angle being published
    // std::cout << "[INFO] Publishing to SERVO_TARGETS:";
    // for (int i = 0; i < NUM_SERVOS; i++)
    // {
    //     std::cout << " s" << i << "=" << msg.angles[i];
    // }
    // std::cout << std::endl;

    lcm.publish("SERVO_TARGETS", &msg);
}
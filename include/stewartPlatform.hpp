/*
    Defines a class for representing the Stewart platform. Supports calculation
    of motor angles required to achieve a desired platform pose. 

    Also defines PlatformPose to represent this pose. 
*/

#ifndef STEWARTPLATFORM_H
#define STEWARTPLATFORM_H

#include <array>
#include <Eigen/Dense>

#define NUM_SERVOS 6

using Eigen::Vector3f;
using Eigen::Quaternionf;

struct PlatformPose
{
    /*  Position of the platform origin relative to the base origin.
        Platform origin: centre of platform at lowest platform height. */
    Vector3f position;          

    /*  Orientation of the platform relative to the base. */
    Quaternionf orientation; 
};

class StewartPlatform
{
    private: 
        // Platform geometry 
        float lower_arm_length;                             // in mm     
        float upper_arm_length;                             // in mm      
        std::array<float, NUM_SERVOS> servo_angular_pos;    // Fixed angle of servo horns relative to the horizontal when viewed top-down, in radians
        std::array<Vector3f, NUM_SERVOS> base_anchors;      // Vector positions of the arm attachment points on the base relative to a centered origin, in mm
        std::array<Vector3f, NUM_SERVOS> platform_anchors;  // Platform attachment points, in mm

        // Latest platform pose target 
        PlatformPose platform_pose; 

        // Required servo angles for the latest platform pose   
        std::array<float, NUM_SERVOS> servo_targets;

        // Calculate the required servo angles to achieve the latest platform pose
        bool computeServoTargets();
    
    public: 
        StewartPlatform();
        ~StewartPlatform();

        // Move the platform to a target pose 
        bool moveTo(PlatformPose* target_pose);

        // Returns an array of the servo motor angles required to achieve the 
        // last legitimate pose
        std::array<float, NUM_SERVOS> getServoTargets();
};

#endif
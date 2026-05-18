/*
    Defines the top level class for the payload application. 
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stewartPlatform.hpp"
#include "lcmHandler.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>

class Payload
{
    private: 
        lcm::LCM lcm;
        LcmHandler lcm_handler;

        StewartPlatform platform;

        // Relative location of the trajectory csv 
        std::string trajectory_path; 

        // The trajectory for the Stewart platform to track
        std::vector<PlatformPose> trajectory;

        // The motor angles for the Stewart platform to track to follow the trajectory
        std::vector<std::array<float, NUM_SERVOS>> trajectory_angles;

        // Computes the Stewart platform trajectory to a set of motor angles
        // Populates the trajectory_angles member.
        // Return value indicates if all angles could be calculated.
        bool computeTrajectoryAngles();

        // Writes a vector of motor angles to the file 
        bool writeAnglesToFile(std::string file_path);

    public:
        Payload();
        ~Payload();

        // Run the main event loop 
        void run(); 

        // Read the trajectory from the file and store in memory
        // Return value indicates if the file was found 
        bool readTrajectory();

        // Display the trajectory to std::cout 
        void printTrajectory();

        // Generates a csv file of the motor angles corresponding to a trajectory 
        bool generateTrajectoryAnglesFile(std::string file_path);
};

#endif
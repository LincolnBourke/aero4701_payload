/*
    Defines the top level class for the payload application. 
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stewartPlatform.hpp"

#include <string>
#include <vector>

class Payload
{
    private: 
        // Relative location of the trajectory csv 
        std::string trajectory_path; 

        // The trajectory for the Stewart platform to track
        std::vector<PlatformPose> trajectory;

    public:
        Payload();
        ~Payload();

        // Read the trajectory from the file and store in memory
        // Return value indicates if the file was found 
        bool readTrajectory();

        // Display the trajectory to std::cout 
        void printTrajectory();
};

#endif
/*
    Defines the top level class for the payload application. 
*/

#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "stewartPlatform.hpp"
#include "lcmHandler.hpp"
#include "camera_command_t.hpp"
#include "run_result_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>

// Define the error processed by the ERROR state
typedef struct {
    std::string msg; // Error message 
} payload_error_t;

class Payload
{
    private: 
        // LCM interface 
        lcm::LCM lcm;
        LcmHandler lcm_handler;

        // Error processed by the ERROR state
        payload_error_t error;
        
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

        // Incrementally move the platform to the starting trajectory_angles position.
        // platform_deployed = false indicates this method should be called again.
        // Return value indicates if the platform could be deployed.
        bool deployPlatformStep(bool &platform_deployed);

        // Move the platform along the trajectory defined by trajectory_angles.
        // trajectory_complete = false indicates this method should be called again.
        // Return value indicates if the trajectory could be successfully followed.
        bool trackTrajectoryStep(bool &trajectory_complete);

        // Move the platform back to the home position.
        // Return value indicates if the platform could be retracted successfully.
        bool retractPlatform();

        // Block until a save_complete message is received from the camera node.
        // Return value indicates if the save was successful.
        bool waitForSaveComplete();
        
        // --- LCM publisher methods -------------------------------------------
        void publishCameraCommand(int8_t command_id);
        void publishRunResult(int8_t return_id);

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
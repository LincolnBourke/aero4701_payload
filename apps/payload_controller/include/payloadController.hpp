/*
    Defines the top level class for the payload application. 
*/

#ifndef PAYLOAD_CONTROLLER_H
#define PAYLOAD_CONTROLLER_H

#include "stewartPlatform.hpp"
#include "lcmHandler.hpp"
#include "camera_command_t.hpp"
#include "run_result_t.hpp"
#include "payload_cont_to_cam_msg_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

// Define a trajectory
typedef struct {
    std::vector<PlatformPose> poses;
    std::vector<std::array<float, NUM_SERVOS>> angles;
    std::vector<float> times;
} trajectory_t;

// Define the states for the core payload controller state machine
typedef enum State {
    IDLE,               // Waits for command to start an experiment
    READ_TRAJECTORY,    // Reads the trajectory file & converts to interpolated servo angles 
    CALIBRATE_SERVOS,   // Uses limit switches to calibrate servos
    CALIBRATE_CAMERA,   // Moves platform to let camera node calibrate
    DEPLOY,             // Deploys the docking port to the starting position
    RUNNING,            // Runs the experiment
    SAVE_RESULTS,       // Saves experiment data and tells the camera node to do the same
    TERMINATE_RUN,      // Moves the platform back to the home position 
    ERROR,              // Publishes an erroneous run result
    DEBUG,              // Focus camera and take a debug test image
} state_t;

// Define the error processed by the ERROR state
typedef struct {
    std::string msg; // Error message 
} payload_error_t;

class PayloadController
{
    private:
        // LCM interface
        lcm::LCM lcm;
        LcmHandler lcm_handler;

        // Error processed by the ERROR state
        payload_error_t error;

        StewartPlatform platform;

        // Relative path to the trajectory csv file
        std::string trajectory_path;

        // The fully interpolated trajectory for the platform to track
        trajectory_t trajectory;

        // The step number which trackTrajectoryStep is up to
        std::vector<float>::size_type trajectory_step;

        // The time for which the experiment has been running
        std::chrono::time_point<std::chrono::steady_clock> experiment_start_time;

        // Reads raw poses from the trajectory file into raw_poses.
        // Return value indicates if the file was found and read successfully.
        bool readRawPoses(std::vector<PlatformPose>& raw_poses);

        // Interpolates between raw_poses at TRAJECTORY_STRUCT_STEP intervals,
        // populating out.poses and out.times.
        // Return value indicates if interpolation was successful.
        bool interpolateTrajectory(const std::vector<PlatformPose>& raw_poses, trajectory_t& out, 
            float raw_pose_step, float trajectory_step);
        
        // Computes servo angles for every pose in traj, populating traj.angles.
        // Return value indicates if all angles could be calculated.
        bool computeTrajectoryAngles(trajectory_t& traj);

        // Generate a trajectory to move the platform from its current orientation and
        // x-y plane offset onto the z-axis with the platform flat
        // bool compute

        // Writes the servo angles in the trajectory struct to a file.
        // Return value indicates if the file could be written.
        bool writeAnglesToFile(std::string file_path);

        // Incrementally moves the platform to the starting position in trajectory.angles.
        // platform_deployed = false indicates this method should be called again.
        // Return value indicates if the platform could be deployed.
        bool deployPlatformStep(bool &platform_deployed);

        // Moves the platform along the trajectory defined by trajectory.
        // trajectory_complete = false indicates this method should be called again.
        // Return value indicates if the trajectory could be successfully followed.
        bool trackTrajectoryStep(bool &trajectory_complete);

        // Move the platform back to the home position.
        // Return value indicates if the platform could be retracted successfully.
        bool retractPlatform();

        // // Block until a save_complete message is received from the camera node.
        // // Return value indicates if the save was successful.
        // Replaced with waitForCamStatus, but leaving in case needs to be brough back
        // bool waitForSaveComplete();

        // Block until status message from camera 
        bool waitForCamStatus(int timeout_ms);

        // --- State methods ---------------------------------------------------
        state_t handleIdleState();
        state_t handleReadTrajectoryState();
        state_t handleCalibrateServosState();
        state_t handleCalibrateCameraState();
        state_t handleDeployState();
        state_t handleRunningState();
        state_t handleSaveResultsState();
        state_t handleTerminateRunState();
        state_t handleErrorState();
        state_t handleDebugState();

        // --- LCM publisher methods -------------------------------------------
        void publishCameraCommand(state_t state, bool debug_mode);
        void publishRunResult(int8_t return_id);

    public:
        PayloadController();
        ~PayloadController();

        // Run the main event loop
        void run();

        // Reads the trajectory file, interpolates to TRAJECTORY_STRUCT_STEP intervals,
        // and computes servo angles. Populates the trajectory member atomically.
        // Return value indicates if the trajectory was built successfully.
        bool buildTrajectory();

        // Display the trajectory to std::cout 
        void printTrajectory();

        // Generates a csv file of the motor angles corresponding to a trajectory 
        bool generateTrajectoryAnglesFile(std::string file_path);
};

#endif

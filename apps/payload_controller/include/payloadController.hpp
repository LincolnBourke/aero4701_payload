/*
    Defines the top level class for the payload application. 
*/

#ifndef PAYLOAD_CONTROLLER_H
#define PAYLOAD_CONTROLLER_H

#include "stewartPlatform.hpp"
#include "lcmHandler.hpp"
#include "camera_command_t.hpp"
#include "run_result_t.hpp"
#include "switch_state_t.hpp"
#include "payload_cont_to_cam_msg_t.hpp"

#include "payloadConfig.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <iomanip>
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
        // LCM interface for general commands
        lcm::LCM lcm;
        LcmHandler lcm_handler;

        // Tolerance accepted before moving to next target pose 
        float tol; 

        // Error processed by the ERROR state
        payload_error_t error;

        StewartPlatform platform;

        // Relative path to the trajectory csv file
        std::string trajectory_path;

        // The fully interpolated trajectory for the platform to track
        trajectory_t trajectory;

        // The step number which trackTrajectoryStep is up to
        std::vector<float>::size_type trajectory_step;

        // Two-phase deployment trajectory from calibrated rest to trajectory.poses[0]
        trajectory_t deploy_trajectory;

        // Current step in deploy_trajectory
        size_t deploy_step;

        // The time for which the experiment has been running
        std::chrono::time_point<std::chrono::steady_clock> experiment_start_time;

        // General-purpose timer (start with startTimer(), query with readTimer())
        std::chrono::time_point<std::chrono::steady_clock> timer_start;
        void startTimer();
        float readTimer(); // returns ms elapsed since startTimer()

        // Servo feedback across the length of the trajectory
        // Emptied at start of an experiment and populated by the end
        std::vector<std::array<float, NUM_SERVOS>> servo_feedback;
        std::chrono::time_point<std::chrono::steady_clock> last_feedback_sample_time;

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

        // Builds the two-phase deployment trajectory into deploy_trajectory.
        // Must be called after buildTrajectory() populates trajectory.poses.
        // Return value indicates if both phases could be constructed.
        bool buildDeployTrajectory();

        // Incrementally moves the platform one step along deploy_trajectory.
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

        // Block until the platform has reached the target pose with some tolerance 
        bool waitForPose(const long int timeout); 

        // --- Servo calibration helpers ---------------------------------------
        // Moves the platform from PLATFORM_REST_Z to CALIBRATION_START_Z.
        bool moveToCalibrationStart();

        // Lowers the platform from CALIBRATION_START_Z to CALIBRATION_END_Z,
        // polling limit switches. Sets switches_activated on return.
        bool descendUntilSwitchActivation(bool& switches_activated);

        // Reads servo angles at switch activation and sets calibration offsets.
        void applyCalibrationOffsets(bool switches_activated);

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

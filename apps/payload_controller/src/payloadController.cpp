#include "payloadController.hpp"
#include "commands.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#define TRAJECTORY_FILE_STEP 200 // ms, time between successive poses in the trajectory file
#define TRAJECTORY_STRUCT_STEP 50 // ms, time between successive poses in the trajectory struct
#define TRAJECTORY_FILE_PATH "data/docking_trajectory.csv"
#define RESULTS_FILEPATH        "data/test_obc_nominal/results.csv"

static const char* CH_CONT_TO_CAM = "PAYLOAD_CAM";   // controller --> camera
static const char* CH_CAM_TO_CONT = "CAM_PAYLOAD";   // camera --> controller

// Timeouts for waiting for camera to respond
static const int CAM_WAIT_TIMEOUT_CALIB_MS  = 60000;  // 1 min for calibration - need this to wait enough time for camera to calibrate (attempts 3 times) 
static const int CAM_WAIT_TIMEOUT_DEFAULT_MS = 10000;  // 10s timeout
static const int CAM_WAIT_TIMEOUT_SAVE_MS = 40000;  // 30s for processing + 10s buffer - need extra time to process results and save pose estimates

// Automatically enters debug mode, no flag set - could be a comms from OBC?
static const bool DEBUG_MODE = true;
#define CALIBRATION_END_POINT_STEP 5000 // ms
#define CALIBRATION_STRUCT_STEP 50 // ms
#define CALIBRATION_START_Z 5 // mm
#define CALIBRATION_END_Z 0 // mm
#define PLATFORM_REST_Z 1 // mm, assumed platform height at rest before calibration begins

// Angle of the servos when switches activated 
    // Obtained from CAD 
constexpr float PHYSICAL_ANGLE_AT_ACTIVATION = -41.58f * M_PI / 180.0f;

PayloadController::PayloadController()
    : lcm(), lcm_handler(), tol(5*M_PI/180), // tolerance of 5 degs  
     error(), platform(), trajectory_step(0), experiment_start_time()
{
    trajectory_path = TRAJECTORY_FILE_PATH;

    if (!lcm.good())
    {
        std::cout << "[ERROR] Payload LCM object not good." << std::endl;
    }

    // Subscribe lcm handler to messages
    lcm.subscribe("RUN_COMMAND", &LcmHandler::handleRunCommand, &lcm_handler);
    lcm.subscribe("SAVE_COMPLETE", &LcmHandler::handleSaveComplete, &lcm_handler);
    lcm.subscribe("LIMIT_SWITCH_STATES", &LcmHandler::handleSwitchStateMsg, &lcm_handler);
    lcm.subscribe("SERVO_STATE", &LcmHandler::handleServoStateMsg, &lcm_handler);

    // Clear LCM messages 
    while (lcm.handleTimeout(0) == 1); 
    
    // Added camera to controller channel
    lcm.subscribe(CH_CAM_TO_CONT, &LcmHandler::handleCamMsg, &lcm_handler);
};

PayloadController::~PayloadController(){};

// Runs the core state machine for the payload controller
void PayloadController::run()
{
    state_t state = IDLE;
    
    while (true)
    {
        switch (state)
        {
        case IDLE:
            state = handleIdleState();
            break;
        case READ_TRAJECTORY:
            state = handleReadTrajectoryState();
            break;
        case CALIBRATE_SERVOS:
            state = handleCalibrateServosState();    
            break;
        case CALIBRATE_CAMERA:
            state = handleCalibrateCameraState();
            break;
        case DEPLOY:
            state = handleDeployState();
            break;
        case RUNNING:
            state = handleRunningState();
            break;
        case SAVE_RESULTS:
            state = handleSaveResultsState();
            break;
        case TERMINATE_RUN:
            state = handleTerminateRunState();
            break;
        case ERROR:
            state = handleErrorState();
            break;
        case DEBUG:
            state = handleDebugState();
            break;
        default: 
            std::cout << "[ERROR] Payload controller entered invalid state." << std::endl;
        }
    }
}

// --- State logic -------------------------------------------------------------

state_t PayloadController::handleIdleState()
{
    int command_id; 

    // Check if a run command has been published
    lcm.handleTimeout(0);
    if (lcm_handler.checkRunCommand(command_id))
    {
        // Only move to setup when start command received
        if (command_id == Commands::RunId::RUN_CONTROLLER)
        {
            std::cout << "[INFO] Payload controller state set to READ_TRAJECTORY." << std::endl;
            return READ_TRAJECTORY;
        }
        // Only move to debug when command received
        if (command_id == Commands::RunId::RUN_DEBUG)
        {
            std::cout << "[INFO] Payload controller state set to DEBUG." << std::endl;
            return DEBUG;
        }
    }

    return IDLE;
}

state_t PayloadController::handleReadTrajectoryState()
{
    // Read the trajectory file, interpolate, and compute servo angles
    if (buildTrajectory() == false)
    {
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }
    std::cout << "[INFO] Trajectory file parsed." << std::endl;

    // TODO check if will need a sleep for python

    // Automatically transition to servo calibration when the trajectory can be read
    std::cout << "[INFO] Payload controller state set to CALIBRATE_SERVOS." << std::endl;
    return CALIBRATE_SERVOS;
}

state_t PayloadController::handleCalibrateServosState()
{
    // Max and min z values scanned during calibration
    const float max_z = CALIBRATION_START_Z; // mm
    const float min_z = CALIBRATION_END_Z; // mm

    // Phase 1: Move platform outward from rest position to calibration start (no switch checking)
    {
        PlatformPose rest_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()};
        PlatformPose outward_start_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()};
        rest_pose.position(2) = PLATFORM_REST_Z;
        outward_start_pose.position(2) = max_z;
        std::vector<PlatformPose> outward_end_points = {rest_pose, outward_start_pose};

        trajectory_t outward_trajectory;
        if (interpolateTrajectory(outward_end_points, outward_trajectory, CALIBRATION_END_POINT_STEP, CALIBRATION_STRUCT_STEP) == false)
        {
            error.msg = "Could not generate outward calibration trajectory.";
            std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
            return ERROR;
        }

        auto outward_start_time = std::chrono::steady_clock::now();
        size_t j = 0;
        while (j < outward_trajectory.times.size())
        {
            std::chrono::duration<double> time_temp = std::chrono::steady_clock::now() - outward_start_time;
            double outward_time = std::chrono::duration<double, std::milli>(time_temp).count();

            if (outward_time >= outward_trajectory.times[j])
            {
                if (platform.moveTo(outward_trajectory.poses[j]) == false)
                {
                    error.msg = "Could not move platform to starting pose during servo calibration.";
                    std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                    return ERROR;
                }
                j++;
            }
        }
    }

    // Phase 2: Lower platform from calibration start to end, checking for limit switch activation

    // Generate the start and end positions for the servo calibration
    PlatformPose start_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()};
    PlatformPose end_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()}; 
    start_pose.position(2) = max_z;
    end_pose.position(2) = min_z;
    std::vector<PlatformPose> end_points = {start_pose, end_pose};

    // Generate trajectory between positions
    trajectory_t calibration_trajectory;
    if (interpolateTrajectory(end_points, calibration_trajectory, CALIBRATION_END_POINT_STEP, CALIBRATION_STRUCT_STEP) == false)
    {
        error.msg = "Could not generate inward calibration trajectory.";
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Start timing calibration 
    calibration_start_time = std::chrono::steady_clock::now();

    // Lower platform along trajectory until limit switches activate 
    int switch_states[3] = {0}; 
    bool switches_activated = false; 

    size_t i = 0;
    while (true)
    {   
        // Exit when finished trajectory
        if (i == calibration_trajectory.times.size())
        {
            break;
        }

        // Get the current time since calibration began 
        std::chrono::duration<double> time_temp = std::chrono::steady_clock::now() - calibration_start_time;
        double calibration_time = std::chrono::duration<double, std::milli>(time_temp).count();
        
        // Move platform along trajectory
        if (calibration_time >= calibration_trajectory.times[i])
        {
            if ( platform.moveTo(calibration_trajectory.poses[i]) == false )
            {
                error.msg = "Could not move platform to starting pose during servo calibration.";
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                return ERROR;
            }

            // std::cout << "Time: " << calibration_trajectory.times[i] << "s" << std::endl;
            // std::cout << "Target position: " << calibration_trajectory.poses[i].position.transpose() << std::endl;
            i++;
        }        

        // Check if the limit switches have activated
        while (lcm.handleTimeout(0) == 1) {};

        // bool result;
        bool all_flag = false; // If all switches have been tripped 
        auto result = lcm_handler.checkSwitchState(switch_states, all_flag); 

        printf("[INFO] Checked switch state, result %d, states [%d, %d, %d]\n:",
            result, switch_states[0], switch_states[1], switch_states[2]); 
        std::cout << std::flush; 

        if ( all_flag ) 
        {
            // New message 
            std::cout << "[INFO] New switch state: [ " << switch_states[0]; 
            std::cout << ", " << switch_states[1]; 
            std::cout << ", " << switch_states[2]; 
            std::cout << "]" << std::endl; 

            // If all activated, stop 
            if ( all_flag )  
            {
                switches_activated = true; 
                break; 
            }
        }
    }
    
    // Set calibration offset for the Stewart platform
    while (lcm.handleTimeout(0) == 1); 
    if (switches_activated)
    {
        // At switch activation, physical servo angle is known to be -41.58 deg.
        // Offset = physical - commanded, applied in publishServoTargets().
        float servo_angs[6] = {0};
        std::array<float, NUM_SERVOS> offsets;

        lcm.handleTimeout(10);
        if ( lcm_handler.checkServoAngs(servo_angs) ) {

            // Find the calibration offsets 
            for (int i = 0; i < NUM_SERVOS; i++)
                // Offset in radians 
                offsets[i] = PHYSICAL_ANGLE_AT_ACTIVATION - servo_angs[i]; // * M_PI / 180.0f; 
        }
        else {
            std::cout << "[WARNING] Failed to get calibration angles" << std::endl << std::flush; 
        }

        // Set the offsets (defaults to zero)
        platform.setCalibrationOffsets(offsets); 
        std::cout << "[INFO] Calibration offsets set." << std::endl;
    }
    else 
    {
        std::cout << "[ERROR] Servo calibration procedure did not find state with all switches activated." << std::endl;
        // std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        // return ERROR;
    }

    // Automatically transition to camera calibration when the servos are calibrated
    std::cout << "[INFO] Payload controller state set to CALIBRATE_CAMERA." << std::endl;
    return CALIBRATE_CAMERA;
}

state_t PayloadController::handleCalibrateCameraState()
{
    // Start camera nodes
    publishCameraCommand(CALIBRATE_CAMERA, DEBUG_MODE);
    
    // Wait for camera to report complete (1 min timeout for calibration)
    if (!waitForCamStatus(CAM_WAIT_TIMEOUT_CALIB_MS))
    {
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Automatically deploy platform when camera has been calibrated
    std::cout << "[INFO] Payload controller state set to DEPLOY." << std::endl;
    return DEPLOY;
}

state_t PayloadController::handleDeployState()
{
    bool platform_deployed;
    int command_id;

    // Make an incremental step to move the platform to the starting position
    if (deployPlatformStep(platform_deployed) == false)
    {
        error.msg = "Could not deploy platform.";
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Check if a message has been published to stop the experiment early
    lcm.handleTimeout(0);
    if (lcm_handler.checkRunCommand(command_id))
    {
        if (command_id == Commands::RunId::STOP_CONTROLLER)
        {
            std::cout << "[INFO] Payload controller state set to TERMINATE_RUN." << std::endl;
            return TERMINATE_RUN;
        }
    }

    // Check if the platform is fully deployed 
    if (platform_deployed == true)
    {
        std::cout << "[INFO] Payload controller state set to RUNNING." << std::endl;
        
        // Publish deploy to camera 
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publishCameraCommand(DEPLOY, DEBUG_MODE);

        // Wait for camera to report complete
        if (!waitForCamStatus(CAM_WAIT_TIMEOUT_DEFAULT_MS))
        {
            std::cout << "[INFO] Payload controller state set to TERMINATE_RUN." << std::endl;
            return TERMINATE_RUN;
        }

        experiment_start_time = std::chrono::steady_clock::now(); // Start timing experiment
        trajectory_step = 0; // Track trajectory from the start
        
        return RUNNING; 
    }

    return DEPLOY;
}

state_t PayloadController::handleRunningState()
{
    bool trajectory_complete;
    int command_id;

    // Only publish to camera on first step
    if (trajectory_step == 0)
    {
        // Give camera python script a second to catch up
        std::cout << "[INFO] Publish RUNNING to camera" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        publishCameraCommand(RUNNING, DEBUG_MODE);
    }

    // Make an incremental step to move the platform along the trajectory
    if (trackTrajectoryStep(trajectory_complete) == false)
    {
        // error.msg = "Failure while tracking trajectory.";
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Check if a message has been published to stop the experiment early
    lcm.handleTimeout(0);
    if (lcm_handler.checkRunCommand(command_id))
    {
        if (command_id == Commands::RunId::STOP_CONTROLLER)
        {
            std::cout << "[INFO] Payload controller state set to TERMINATE_RUN." << std::endl;
            return TERMINATE_RUN;
        }
    }

    // Check if the trajectory is complete before moving to SAVE_RESULTS
    if (trajectory_complete == true)
    {
        std::cout << "[INFO] Payload controller state set to SAVE_RESULTS." << std::endl;
        return SAVE_RESULTS;
    }

    return RUNNING;
}

state_t PayloadController::handleSaveResultsState()
{
    // Need 1s buffer for camera python to be ready
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Create a results file and save the servo angles across the trajectory
    // TODO
    // Suggested from Ollie - use this as base for code to save into experiment_results, then I will append to same file in camera python
    // Extract the boot count number on compute module (hardcoded path)
    int boot_count = 0;
    std::ifstream count_file("/home/slave2/Documents/service_test/boot_count.txt");
    if (count_file.is_open())
    {
        count_file >> boot_count;
        count_file.close();
    }
    else
    {
        std::cout << "[WARN] Could not read boot_count.txt, defaulting to 0" << std::endl;
    }

    // Create path names based on current boot
    char boot_dir[64];
    std::snprintf(boot_dir, sizeof(boot_dir), "outputs/boot_%03d", boot_count);
    std::string results_dir = std::string(boot_dir) + "/experiment_results";
    std::filesystem::create_directories(results_dir);

    // Example of writing servo angles to csv
    std::ofstream results_file(results_dir + "/experiment_results.csv");
    if (!results_file.is_open())
    {
        std::cout << "[ERROR] Failed to open results file." << std::endl;
        return ERROR;
    }
    // results_file << "tx,ty,tz,rx,ry,rz\n"; 
    results_file << "0.0,0.0,0.0,0.0,0.0,0.0\n";
    results_file.close();
    
    // Publish SAVE_RESULTS state to camera
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publishCameraCommand(SAVE_RESULTS, DEBUG_MODE);

    // Wait for camera to report complete
    if (!waitForCamStatus(CAM_WAIT_TIMEOUT_SAVE_MS))
    {
        std::cout << "[INFO] State set to ERROR." << std::endl;
        return ERROR;
    }

    // Copy across experiment results file to path for obc
    std::string src = results_dir + "/experiment_results.csv";
    std::string dst = RESULTS_FILEPATH;
    std::filesystem::create_directories("data/test_obc_nominal");
    try
    {
        std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing);
        std::cout << "[INFO] Results copied to " << dst << std::endl;
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::cout << "[ERROR] Failed to copy results file: " << e.what() << std::endl;
        return ERROR;
    }

    // Let the OBC bridge know the experiment is complete and results file has been saved
    publishRunResult(Commands::RunResult::RUN_SUCCESS);

    std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
    return IDLE;
}

state_t PayloadController::handleTerminateRunState()
{
    if (retractPlatform() == false)
    {
        error.msg = "Failed to retract the platform automatically.";
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Publish TERMINATE_RUN state to camera
    // TODO check if will need sleep
    publishCameraCommand(TERMINATE_RUN, DEBUG_MODE);

    // Wait for camera to report complete
    if (!waitForCamStatus(CAM_WAIT_TIMEOUT_DEFAULT_MS))
    {
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Automatically move back to IDLE
    std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
    return IDLE;
}

state_t PayloadController::handleErrorState()
{
    // Publish ERROR state to camera 
    publishCameraCommand(ERROR, DEBUG_MODE);
    
    // TODO: determine if the error message should be used elsewhere
    std::cout << "[ERROR] " << error.msg << std::endl;

    // Let the OBC bridge know the experiment failed
    publishRunResult(Commands::RunResult::RUN_FAIL);

    // Automatically move back to IDLE
    std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
    return IDLE;
}

state_t PayloadController::handleDebugState()
{
    // Start camera node
    publishCameraCommand(DEBUG, DEBUG_MODE);
    
    // Wait for camera to report complete (1 min timeout for debug)
    if (!waitForCamStatus(CAM_WAIT_TIMEOUT_CALIB_MS))
    {
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        // Transmit to OBC error
        publishRunResult(Commands::RunResult::RUN_FAIL);
        return ERROR;
    }

    // Transmit to OBC on success
    publishRunResult(Commands::RunResult::RUN_SUCCESS); 

    // Send back to idle for next OBC command
    std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
    return IDLE;
}

// --- Trajectory tracking -----------------------------------------------------

bool PayloadController::deployPlatformStep(bool &platform_deployed)
{
    // TODO: current set to move past successfully
    platform_deployed = true;

    return true; 
}

bool PayloadController::trackTrajectoryStep(bool &trajectory_complete)
{
    trajectory_complete = false;

    // Check if there are no more poses remaining to track
    if (trajectory_step == trajectory.times.size())
    {
        trajectory_complete = true;
        return true;
    }
    
    // Get the current experiment time in ms
    std::chrono::duration<double> time_temp = std::chrono::steady_clock::now() - experiment_start_time;
    double experiment_time = std::chrono::duration<double, std::milli>(time_temp).count();

    // Move the platform to the next pose until we catch up to the current time
    while (experiment_time >= trajectory.times[trajectory_step] && trajectory_step < trajectory.times.size())
    {
        // Display the target pose for debugging
        // std::cout << "[INFO] Moving to target pose:" 
        //     << " position: " << trajectory.poses[trajectory_step].position.transpose() << ","
        //     << " orientation: [" << trajectory.poses[trajectory_step].orientation.x() << ", "
        //     << trajectory.poses[trajectory_step].orientation.y() << ", "
        //     << trajectory.poses[trajectory_step].orientation.z() << ", "
        //     << trajectory.poses[trajectory_step].orientation.w() << "]" << std::endl;
        
        if (platform.moveTo(trajectory.poses[trajectory_step]) == false)
        {
            // error.msg = "Could not move platform to target pose.";
            return false;
        }
        trajectory_step++;
    }    

    return true; 
}

bool PayloadController::retractPlatform()
{
    // TODO
    return false;
}

// --- File i/o ----------------------------------------------------------------

bool PayloadController::readRawPoses(std::vector<PlatformPose>& raw_poses)
{
    bool result = true;
    std::ifstream file(trajectory_path);

    // Check the file could be opened
    if (!file.is_open())
    {
        std::cout << "Trajectory file not found." << std::endl;
        result = false;
    }
    else
    {
        // Read each line of the file
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string p_x, p_y, p_z, roll, pitch, yaw;

            // Parse the six comma-separated values on each line
            if (std::getline(ss, p_x, ',') &&
                std::getline(ss, p_y, ',') &&
                std::getline(ss, p_z, ',') &&
                std::getline(ss, roll, ',') &&
                std::getline(ss, pitch, ',') &&
                std::getline(ss, yaw))
            {
                // Attitude values in the trajectory file are in degrees; convert to radians for Eigen
                Eigen::AngleAxisf roll_angle(std::stof(roll) * M_PI / 180,   Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitch_angle(std::stof(pitch) * M_PI / 180, Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yaw_angle(std::stof(yaw) * M_PI / 180,     Eigen::Vector3f::UnitZ());

                Eigen::Quaternionf q = yaw_angle * pitch_angle * roll_angle;

                PlatformPose pose {
                    Vector3f(std::stof(p_x), std::stof(p_y), std::stof(p_z)), q
                };

                raw_poses.push_back(pose);
            }
            else
            {
                std::cout << "Error reading trajectory file." << std::endl;
                result = false;
                break;
            }
        }

        file.close();
    }

    return result;
}

bool PayloadController::writeAnglesToFile(std::string file_path)
{
    bool result = true;
    std::ofstream file(file_path);

    // Check the file could be opened/created
    if (!file.is_open())
    {
        std::cout << "Error: could not open file for writing: " << file_path << std::endl;
        result = false;
    }
    else
    {
        // Write each row of servo angles as a comma-separated line
        for (const auto& angles : trajectory.angles)
        {
            for (size_t i = 0; i < NUM_SERVOS - 1; i++)
            {
                file << angles[i] << ",";
            }

            file << angles[NUM_SERVOS - 1] << "\n";
        }

        file.close();
    }

    return result;
}

// --- Trajectory building -----------------------------------------------------

// Interpolate raw poses spaced by raw_pose_step at interval specified by
// trajectory_step. Both steps are in ms.
bool PayloadController::interpolateTrajectory(const std::vector<PlatformPose>& raw_poses, 
    trajectory_t& out, float raw_pose_step, float trajectory_step)
{
    // Require at least two poses to interpolate between
    if (raw_poses.size() < 2)
    {
        return false;
    }

    const int n_steps = raw_pose_step / trajectory_step;

    // Interpolate between each consecutive pair of raw poses
    for (size_t i = 0; i < raw_poses.size() - 1; i++)
    {
        // Generate n_steps interpolated poses between raw_poses[i] and raw_poses[i+1]
        for (int j = 0; j < n_steps; j++)
        {
            float t = (float)j / n_steps;

            // Linearly interpolate position
            Vector3f pos = raw_poses[i].position + t * (raw_poses[i + 1].position - raw_poses[i].position);

            // Spherically interpolate orientation
            Eigen::Quaternionf orientation = raw_poses[i].orientation.slerp(t, raw_poses[i + 1].orientation);

            out.poses.push_back({pos, orientation});
            out.times.push_back((float)(i * raw_pose_step + j * trajectory_step));
        }
    }

    // Append the final raw pose to close the trajectory
    out.poses.push_back(raw_poses.back());
    out.times.push_back((float)((raw_poses.size() - 1) * raw_pose_step));

    return true;
}

bool PayloadController::computeTrajectoryAngles(trajectory_t& traj)
{
    bool result = true;
    std::array<float, NUM_SERVOS> angles;

    // Pre-allocate memory for the angles
    traj.angles.resize(traj.poses.size());

    // Compute the required servo angles for each pose in the trajectory
    for (size_t i = 0; i < traj.poses.size(); i++)
    {
        if (!platform.getAnglesForMove(traj.poses[i], &angles))
        {
            result = false;
            break;
        }

        traj.angles[i] = angles;
    }

    return result;
}

bool PayloadController::buildTrajectory()
{
    trajectory_t temp;
    std::vector<PlatformPose> raw_poses;

    // Read the raw poses from the trajectory file
    if (readRawPoses(raw_poses) == false)
    {
        error.msg = "Could not read trajectory file.";
        return false;
    }

    // Interpolate between raw poses at TRAJECTORY_STRUCT_STEP intervals
    if (interpolateTrajectory(raw_poses, temp, TRAJECTORY_FILE_STEP, TRAJECTORY_STRUCT_STEP) == false)
    {
        error.msg = "Could not interpolate trajectory.";
        return false;
    }

    // Compute servo angles for each interpolated pose
    if (computeTrajectoryAngles(temp) == false)
    {
        error.msg = "Could not convert trajectory to servo angles.";
        return false;
    }

    // Assign only on complete success to avoid partial population
    trajectory = temp;
    return true;
}

// --- Trajectory debugging ----------------------------------------------------

bool PayloadController::generateTrajectoryAnglesFile(std::string file_path)
{
    bool result = true;

    // Build the full trajectory struct from the trajectory file
    if (buildTrajectory() == false)
    {
        result = false;
    }

    // Write the computed servo angles to the output file
    if (result == true && writeAnglesToFile(file_path) == false)
    {
        result = false;
    }

    if (result == false)
    {
        std::cout << "Error: could not generate trajectory angles file." << std::endl;
    }

    return result;
}

void PayloadController::printTrajectory()
{
    for (size_t i = 0; i < trajectory.poses.size(); i++)
    {
        const PlatformPose& pose = trajectory.poses[i];

        // Print timestamp, position, and orientation
        std::cout
            << "t=" << trajectory.times[i] << " ms  "
            << "pos=["  << pose.position.transpose() << "]  "
            << "ori=["  << pose.orientation.w() << " "
                        << pose.orientation.x() << "i "
                        << pose.orientation.y() << "j "
                        << pose.orientation.z() << "k]  "
            << "angles=[";

        // Print servo angles
        for (size_t j = 0; j < NUM_SERVOS; j++)
        {
            std::cout << trajectory.angles[i][j];
            if (j < NUM_SERVOS - 1)
            {
                std::cout << " ";
            }
        }

        std::cout << "]" << std::endl;
    }
};

bool PayloadController::waitForPose(const long int timeout)
{
    const auto& commanded = platform.getServoTargets(); 
    float servo_angs[6] = {0}; 

    auto start_time = std::chrono::steady_clock::now();
    
    // Setup a throttle for the print statements (e.g., print every 500ms)
    auto last_print_time = start_time;
    const auto print_interval = std::chrono::milliseconds(500); 

    std::cout << "[INFO] Waiting to reach target pose..." << std::endl; 

    while (true) 
    {
        lcm.handleTimeout(0);
        if (lcm_handler.checkServoAngs(servo_angs)) {
            
            auto current_time = std::chrono::steady_clock::now();
            
            // --- PRINT ERRORS AT THROTTLED INTERVAL ---
            if (current_time - last_print_time >= print_interval) {
                std::cout << "[INFO] Errors: [ ";
                std::cout << std::fixed << std::setprecision(3); // Lock formatting 
                
                for (int i = 0; i < 6; ++i) {
                    float error = commanded[i] - servo_angs[i]*M_PI/180.0f; 
                    std::cout << std::setw(7) << error*180.0f/M_PI;
                    if (i < 5) std::cout << ", ";
                }
                std::cout << " ]" << std::endl;
                
                last_print_time = current_time; // Reset the print timer
            }
            // ------------------------------------------

            bool all_within_tol = true;
            
            for (int i = 0; i < 6; ++i) {
                if ( std::abs(commanded[i] - servo_angs[i]) > tol ) {
                    all_within_tol = false;
                    break; // Keep the early exit for the tolerance check!
                }
            }

            if (all_within_tol) {
                std::cout << "[INFO] Target pose reached." << std::endl;
                return true;
            }
        }
        
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time
        ).count();

        if (elapsed >= timeout) {
            std::cout << "[WARN] waitForPose timed out!" << std::endl;
            return false; 
        }

        usleep(1000);
    }
}

// bool PayloadController::waitForPose(const long int timeout)
// {
//     const auto& commanded = platform.getServoTargets(); 
//     float servo_angs[6] = {0}; 

//     // Record the starting time point
//     auto start_time = std::chrono::steady_clock::now();

//     // Setup a throttle for the print statements (e.g., print every 500ms)
//     auto last_print_time = start_time;
//     const auto print_interval = std::chrono::milliseconds(500); 

//     std::cout << "[INFO] Waiting to reach target pose..." << std::endl;    
    
//     while (true) 
//     {
//         lcm.handleTimeout(0);
//         if (lcm_handler.checkServoAngs(servo_angs)) {
//             bool all_within_tol = true;
            
//             for (int i = 0; i < 6; ++i) {
//                 if (std::abs(commanded[i] - servo_angs[i]) > tol) {
//                     all_within_tol = false;
//                     break; 
//                 }
//             }

//             if (all_within_tol) return true;
//         }
        
//         // Calculate elapsed time in milliseconds
//         auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
//             std::chrono::steady_clock::now() - start_time
//         ).count();

//         // Exit if we've exceeded the specified timeout
//         if (elapsed >= timeout) {
//             return false; // Timed out before reaching targets
//         }

//         // Small sleep (1000 microseconds = 1 millisecond)
//         usleep(1000);
//     }
// }

// --- LCM publisher methods ---------------------------------------------------

void PayloadController::publishCameraCommand(state_t state, bool debug_mode)
{
    payload_messages::payload_cont_to_cam_msg_t msg;
    msg.cont_state = static_cast<int8_t>(state);
    msg.debug_mode = debug_mode;
    lcm.publish(CH_CONT_TO_CAM, &msg);
    std::cout << "[INFO] Published to " << CH_CONT_TO_CAM
                << ": cont_state=" << static_cast<int>(state)
                << " debug_mode=" << debug_mode << std::endl;
}

void PayloadController::publishRunResult(int8_t return_id)
{
    payload_messages::run_result_t msg;
    msg.return_id = return_id;
    std::cout << "[INFO] Publishing to RUN_RESULT: return_id=" << (int)return_id << std::endl;
    lcm.publish("RUN_RESULT", &msg);
}

// updated with error.msg
bool PayloadController::waitForCamStatus(int timeout_ms)
{
    lcm_handler.reset();
    auto start = std::chrono::steady_clock::now();

    while (!lcm_handler.isCamStatusReceived())
    {
        lcm.handleTimeout(100);

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeout_ms)
        {
            error.msg = "Timed out waiting for camera response.";
            return false;
        }
    }

    if (!lcm_handler.getCamStatus())
    {
        error.msg = "Camera reported failure.";
        return false;
    }

    return true;
}
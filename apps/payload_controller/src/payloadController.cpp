#include "payloadController.hpp"
#include "commands.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#define TRAJECTORY_FILE_STEP 1000 // ms, time between successive poses in the trajectory file
#define TRAJECTORY_STRUCT_STEP 250 // ms, time between successive poses in the trajectory struct
#define TRAJECTORY_FILE_PATH "data/trajectory_simple.csv"

#define CALIBRATION_END_POINT_STEP 1000 // ms
#define CALIBRATION_STRUCT_STEP 20 // ms
#define CALIBRATION_START_Z 15 // mm
#define CALIBRATION_END_Z 2 // mm

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

    // Automatically transition to servo calibration when the trajectory can be read
    std::cout << "[INFO] Payload controller state set to CALIBRATE_SERVOS." << std::endl;
    return CALIBRATE_SERVOS;
}

state_t PayloadController::handleCalibrateServosState()
{   
    // Max and min z values scanned during calibration
    const float max_z = CALIBRATION_START_Z; // mm
    const float min_z = CALIBRATION_END_Z; // mm

    // Generate the start and end positions for the servo calibration
    PlatformPose start_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()};
    PlatformPose end_pose = PlatformPose{Vector3f::Zero(), Quaternionf::Identity()}; 
    start_pose.position(2) = max_z;
    end_pose.position(2) = min_z;
    std::vector<PlatformPose> end_points = {start_pose, end_pose};

    // Generate trajectory between positions
    trajectory_t calibration_trajectory;
    bool success = interpolateTrajectory(end_points, calibration_trajectory, CALIBRATION_END_POINT_STEP, CALIBRATION_STRUCT_STEP);
    if (success == false)
    {
        error.msg = "Could not generate calibration trajectory.";
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        return ERROR;
    }

    // Lower platform along trajectory until limit switches activate 
    int switch_states[3] = {0}; 
    bool switches_activated = false; 

    for (size_t i = 0; i < calibration_trajectory.times.size(); i++)
    {   
        // Check if the limit switches have activated
        lcm.handleTimeout(0);

        // bool result;
        bool all_flag; // If all switches have been tripped 
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
            // if ( all_flag ) NOTE: COMMENTED OUT FOR DEBUGGING 
            // {
            //     switches_activated = true; 
            //     break; 
            // }
        }

        // Move platform along trajectory
        if ( platform.moveTo(calibration_trajectory.poses[i]) == false )
        {
            error.msg = "Could not move platform to starting pose during servo calibration.";
            std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
            return ERROR;
        }
        else 
        {
            // Wait for it to reach correct pose 
            long int timeout = 10000; // ms = 10s 
            waitForPose(timeout); 
        }
        std::cout << "Target position: " << calibration_trajectory.poses[i].position.transpose() << std::endl;

        // 20ms = 50Hz, matches servo PWM update rate 
        // usleep(20000); 
        // usleep(100000); 
    }
    
    // Set calibration offset for the Stewart platform
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
                offsets[i] = PHYSICAL_ANGLE_AT_ACTIVATION - servo_angs[i] * M_PI / 180.0f; 
        }
        else {
            std::cout << "[WARNING] Failed to get calibration angles" << std::endl << std::flush; 
        }

        // Set the offsets (defaults to zero)
        platform.setCalibrationOffsets(offsets); 
        std::cout << "[INFO] Calibration offsets set." << std::endl;

        while (true) {
            usleep(1000); 
            // PlatformPose platform_pose; 
            // platform_pose = platform.getPlatformPose(); 
            // std::cout << "[INFO] Plaform z: " << platform_pose.position[2] << std::endl << std::flush; 
        }
    }
    else 
    {
        std::cout << "[ERROR] Servo calibration procedure did not find state with all switches activated." << std::endl;
        // std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
        // return ERROR;
    }

    while (true) {};

    // Automatically transition to camera calibration when the servos are calibrated
    std::cout << "[INFO] Payload controller state set to CALIBRATE_CAMERA." << std::endl;
    return CALIBRATE_CAMERA;
}

state_t PayloadController::handleCalibrateCameraState()
{
    // Start camera nodes
    publishCameraCommand(Commands::CameraCommandId::START_CAMERA);

    // TODO: calibrate camera commands

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

    // Check if the platform is fully deployed before moving to RUNNING
    if (platform_deployed == true)
    {
        std::cout << "[INFO] Payload controller state set to RUNNING." << std::endl;
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
    // Create a results file and save the servo angles across the trajectory
    // TODO

    // Prompt the event camera node to save its results and wait for confirmation
    publishCameraCommand(Commands::CameraCommandId::STOP_AND_SAVE);

    if (waitForSaveComplete() == false)
    {
        std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
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

    // Automatically move back to IDLE
    std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
    return IDLE;
}

state_t PayloadController::handleErrorState()
{
    // TODO: determine if the error message should be used elsewhere
    std::cout << "[ERROR] " << error.msg << std::endl;

    // Let the OBC bridge know the experiment failed
    publishRunResult(Commands::RunResult::RUN_FAIL);

    // Automatically move back to IDLE
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
        std::cout << "[INFO] Moving to target pose:" 
            << " position: " << trajectory.poses[trajectory_step].position.transpose() << ","
            << " orientation: "<< trajectory.poses[trajectory_step].orientation << std::endl;
        
        
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

bool PayloadController::waitForSaveComplete()
{
    bool result = false;
    int return_id;

    while (lcm.getFileno() >= 0)
    {
        lcm.handle();

        if (lcm_handler.checkSaveComplete(return_id))
        {
            if (return_id == Commands::SaveResult::SAVE_SUCCESS)
                result = true;
            else if (return_id == Commands::SaveResult::SAVE_FAIL)
                error.msg = "Camera node failed to save results.";
            else
                error.msg = "Unknown return_id published to SAVE_COMPLETE.";

            break;
        }
    }

    return result;
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
                // Convert euler angles in degrees to a quaternion
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
                    float error = commanded[i] - servo_angs[i];
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

void PayloadController::publishCameraCommand(int8_t command_id)
{
    payload_messages::camera_command_t msg;
    msg.command_id = command_id;
    std::cout << "[INFO] Publishing to CAMERA_COMMAND: command_id=" << (int)command_id << std::endl;
    lcm.publish("CAMERA_COMMAND", &msg);
}

void PayloadController::publishRunResult(int8_t return_id)
{
    payload_messages::run_result_t msg;
    msg.return_id = return_id;
    std::cout << "[INFO] Publishing to RUN_RESULT: return_id=" << (int)return_id << std::endl;
    lcm.publish("RUN_RESULT", &msg);
}

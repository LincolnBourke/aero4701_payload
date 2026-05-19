#include "payload.hpp"
#include "commands.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#define TRAJECTORY_FILE_STEP 1000/5 // ms, time between successive poses in the trajectory file
#define TRAJECTORY_STRUCT_STEP 20 // ms, time between successive poses in the trajectory struct

// States for the core controller state machine
typedef enum State {
    IDLE,           // Waits for command to start an experiment
    SETUP,          // Reads the trajectory file + calibrates servos
    DEPLOY,         // Deploys the docking port to the starting position
    RUNNING,        // Runs the experiment
    SAVE_RESULTS,   // Saves experiment data and tells the camera node to do the same
    TERMINATE_RUN,  // Moves the platform back to the home position 
    ERROR,          // Publishes an erroneous run result
} state_t;


Payload::Payload()
    : lcm(), lcm_handler(), error(), platform() 
{
    trajectory_path = "../data/trajectory.csv";

    if (!lcm.good())
    {
        std::cout << "[ERROR] LCM object could not be created." << std::endl;
    }

    // Subscribe lcm handler to messages
    lcm.subscribe("RUN_COMMAND", &LcmHandler::handleRunCommand, &lcm_handler);
    lcm.subscribe("SAVE_COMPLETE", &LcmHandler::handleSaveComplete, &lcm_handler);
};

Payload::~Payload(){};

void Payload::run()
{
    state_t state = IDLE;
    int command_id;
    bool platform_deployed, trajectory_complete;
    
    while (true)
    {
        switch (state)
        {
        case IDLE:
            // Check if a command has been published
            if (lcm.getFileno() >= 0)
            {
                lcm.handle();

                // Check if a run command has been published
                if (lcm_handler.checkRunCommand(command_id))
                {   
                    // Only move to setup when start command received
                    if (command_id == Commands::RunId::RUN_CONTROLLER)
                    {
                        state = SETUP;
                        std::cout << "[INFO] Payload controller state set to SETUP." << std::endl;
                        break;
                    }
                }
            }

            break; 
        case SETUP:
            // Read the trajectory file, interpolate, and compute servo angles
            if (buildTrajectory() == false)
            {
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }

            // Calibrate the servos
            // TODO

            // Automatically transition to deploy when the setup steps succeed
            state = DEPLOY;
            std::cout << "[INFO] Payload controller state set to DEPLOY." << std::endl;
            break; 
        case DEPLOY: 
            // Make an incremental step to move the platform to the starting position
            if (deployPlatformStep(platform_deployed) == false)
            {
                error.msg = "Could not deploy platform.";
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }
            
            // Check if a message has been published to stop the experiment early
            if (lcm.getFileno() >= 0)
            {
                lcm.handle();

                if (lcm_handler.checkRunCommand(command_id))
                {   
                    if (command_id == Commands::RunId::STOP_CONTROLLER)
                    {
                        state = TERMINATE_RUN;
                        std::cout << "[INFO] Payload controller state set to TERMINATE_RUN." << std::endl;
                        break;
                    }
                }
            }
            
            // Check if the platform is fully deployed before moving to RUNNING
            if (platform_deployed == true)
            {
                // Start camera nodes
                publishCameraCommand(Commands::CameraCommandId::START_CAMERA);

                state = RUNNING;
                std::cout << "[INFO] Payload controller state set to RUNNING." << std::endl;
                break;
            }

            break;
        case RUNNING: 
            // Make an incremental step to move the platform along the trajectory
            if (trackTrajectoryStep(trajectory_complete) == false)
            {
                error.msg = "Failure while tracking trajectory.";
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }

            // Check if a message has been published to stop the experiment early
            if (lcm.getFileno() >= 0)
            {
                lcm.handle();

                if (lcm_handler.checkRunCommand(command_id))
                {   
                    if (command_id == Commands::RunId::STOP_CONTROLLER)
                    {
                        state = TERMINATE_RUN;
                        std::cout << "[INFO] Payload controller state set to TERMINATE_RUN." << std::endl;
                        break;
                    }
                }
            }

            // Check if the trajectory is complete before moving to SAVE_RESULTS
            if (trajectory_complete == true)
            {
                state = SAVE_RESULTS;
                std::cout << "[INFO] Payload controller state set to SAVE_RESULTS." << std::endl;
                break;
            }

            break;
        case SAVE_RESULTS:
            // Create a results file and save the servo angles across the trajectory
            // TODO

            // Prompt the event camera node to save its results and wait for confirmation
            publishCameraCommand(Commands::CameraCommandId::STOP_AND_SAVE);

            if (waitForSaveComplete() == false)
            {
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }

            // Let the OBC bridge know the experiment is complete and results file has been saved
            publishRunResult(Commands::RunResult::RUN_SUCCESS);

            state = IDLE;
            std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
            break;
        case TERMINATE_RUN:
            if (retractPlatform() == false)
            {
                error.msg = "Failed to retract the platform automatically.";
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }

            // Automatically move back to IDLE
            state = IDLE;
            std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
            break;
        case ERROR: 
            // TODO: determine if the error message should be used elsewhere
            std::cout << "[ERROR] " << error.msg << std::endl;
            
            // Let the OBC bridge know the experiment failed
            publishRunResult(Commands::RunResult::RUN_FAIL);

            // Automatically move back to IDLE
            state = IDLE;
            std::cout << "[INFO] Payload controller state set to IDLE." << std::endl;
            break;
        default: 
            std::cout << "[ERROR] Payload controller entered invalid state." << std::endl;
        }
    }
}

// --- Trajectory tracking -----------------------------------------------------

bool Payload::deployPlatformStep(bool &platform_deployed)
{
    platform_deployed = platform_deployed;

    return false; 
}

bool Payload::trackTrajectoryStep(bool &trajectory_complete)
{
    trajectory_complete = trajectory_complete;

    return false; 
}

bool Payload::retractPlatform()
{
    return false;
}

bool Payload::waitForSaveComplete()
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

bool Payload::readRawPoses(std::vector<PlatformPose>& raw_poses)
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

bool Payload::writeAnglesToFile(std::string file_path)
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

bool Payload::interpolateTrajectory(const std::vector<PlatformPose>& raw_poses, trajectory_t& out)
{
    // Require at least two poses to interpolate between
    if (raw_poses.size() < 2)
    {
        return false;
    }

    const int n_steps = TRAJECTORY_FILE_STEP / TRAJECTORY_STRUCT_STEP;

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
            out.times.push_back((float)(i * TRAJECTORY_FILE_STEP + j * TRAJECTORY_STRUCT_STEP));
        }
    }

    // Append the final raw pose to close the trajectory
    out.poses.push_back(raw_poses.back());
    out.times.push_back((float)((raw_poses.size() - 1) * TRAJECTORY_FILE_STEP));

    return true;
}

bool Payload::computeTrajectoryAngles(trajectory_t& traj)
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

bool Payload::buildTrajectory()
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
    if (interpolateTrajectory(raw_poses, temp) == false)
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

bool Payload::generateTrajectoryAnglesFile(std::string file_path)
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

void Payload::printTrajectory()
{
    for (const PlatformPose& pose : trajectory.poses)
    {
        std::cout 
            << "Position: " 
                << pose.position.transpose() << ", " 
            << "Orientation: " 
                << pose.orientation.w() << " "
                << pose.orientation.x() << "i "
                << pose.orientation.y() << "j "
                << pose.orientation.z() << "k" << std::endl;
    }
};

// --- LCM publisher methods ---------------------------------------------------

void Payload::publishCameraCommand(int8_t command_id)
{
    payload_messages::camera_command_t msg;
    msg.command_id = command_id;
    lcm.publish("CAMERA_COMMAND", &msg);
}

void Payload::publishRunResult(int8_t return_id)
{
    payload_messages::run_result_t msg;
    msg.return_id = return_id;
    lcm.publish("RUN_RESULT", &msg);
}

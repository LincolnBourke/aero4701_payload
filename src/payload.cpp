#include "payload.hpp"
#include "commands.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

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

    // Initialise empty trajectory 
    trajectory = std::vector<PlatformPose>();
    trajectory_angles = std::vector<std::array<float, NUM_SERVOS>>();

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
            // Read the saved trajectory file    
            if (readTrajectory() == false)
            {
                error.msg = "Could not read trajectory file.";
                state = ERROR;
                std::cout << "[INFO] Payload controller state set to ERROR." << std::endl;
                break;
            }
            
            // Compute the servo angles for the platform to track the trajectory
            if (computeTrajectoryAngles() == false)
            {
                error.msg = "Could not convert trajectory to servo angles.";
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

bool Payload::readTrajectory()
{
    bool read_success = true; 
    std::ifstream file(trajectory_path);

    // Clear the current trajectory 
    trajectory.clear();

    // Check for valid file
    if (!file.is_open())
    {
        std::cout << "Trajectory file not found." << std::endl;
        read_success = false;
    }
    else 
    {   
        // Read each line of the file 
        std::string line; 
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string p_x, p_y, p_z, roll, pitch, yaw;

            // Iterate through entries in the line  
            if (std::getline(ss, p_x, ',') &&
                std::getline(ss, p_y, ',') &&
                std::getline(ss, p_z, ',') &&
                std::getline(ss, roll, ',') &&
                std::getline(ss, pitch, ',') &&
                std::getline(ss, yaw))
            {
                // Convert euler angles in degrees to a quaternion
                Eigen::AngleAxisf rollAngle(std::stof(roll) * M_PI / 180,   Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitchAngle(std::stof(pitch) * M_PI / 180, Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yawAngle(std::stof(yaw) * M_PI / 180,     Eigen::Vector3f::UnitZ());

                Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

                PlatformPose pose {
                    Vector3f(std::stof(p_x), std::stof(p_y), std::stof(p_z)), q
                };
                
                trajectory.push_back(pose);
            }
            else
            {
                std::cout << "Error reading trajectory file." << std::endl;
                read_success = false; 
                break;
            }
        }
    
        file.close();
    }
    
    return read_success;
};

bool Payload::writeAnglesToFile(std::string file_path)
{
    bool write_success = true;
    std::ofstream file(file_path);

    // Check the file could be opened/created
    if (!file.is_open())
    {
        std::cout << "Error: could not open file for writing: " << file_path << std::endl;
        write_success = false;
    }
    else
    {   
        // Write the angles to the file 
        for (const auto& angles : trajectory_angles)
        {
            for (size_t i = 0; i < NUM_SERVOS-1; i++)
            {
                file << angles[i] << ",";
            }

            file << angles[NUM_SERVOS-1] << "\n";
        }

        file.close();
    }
   
    return write_success;
}

// --- Trajectory parsing ------------------------------------------------------

bool Payload::computeTrajectoryAngles()
{
    bool success = true; 
    std::array<float, NUM_SERVOS> angles; 

    // Pre-allocate memory for the angles 
    trajectory_angles.resize(trajectory.size());

    // Iterate through poses in the trajectory and calculate the required servo angles
    for (size_t i = 0; i < trajectory.size(); i++)
    {
        // Calculate the servo angles 
        if (!platform.getAnglesForMove(trajectory[i], &angles))
        {
            success = false; 
            break;
        }
        trajectory_angles[i] = angles;
    }   
    
    return success;
}

bool Payload::generateTrajectoryAnglesFile(std::string file_path)
{   
    bool success = false; 

    if (readTrajectory())
        if (computeTrajectoryAngles())
            if (writeAnglesToFile(file_path))
                success = true; 
    
    if (!success)
        std::cout << "Error: could not generate trajectory angles file." << std::endl;

    return success; 
}

void Payload::printTrajectory()
{
    for (const PlatformPose& pose : trajectory)
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



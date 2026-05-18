#include "payload.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

// States for the core controller state machine
typedef enum State {
    IDLE,           // waits for command to start an experiment
    SETUP,          // reads the trajectory file + calibrates servos
    DEPLOY,         // deploys the docking port to the starting position
    RUNNING,        // runs the experiment
    SAVE_RESULTS,   // saves experiment data and tells the camera node to do the same
    ERROR,          // publishes an erroneous run result
} state_t;

Payload::Payload()
    : platform()
{
    trajectory_path = "../data/trajectory.csv";

    // Initialise empty trajectory 
    trajectory = std::vector<PlatformPose>();
    trajectory_angles = std::vector<std::array<float, NUM_SERVOS>>();
};

Payload::~Payload(){};


bool Payload::run()
{
    state_t state = IDLE;
    
    switch (state)
    {
        case IDLE:

            break; 
        case SETUP: 

            break; 
        case DEPLOY: 

            break;
        case RUNNING: 

            break;
        case SAVE_RESULTS: 

            break;
        case ERROR: 

            break;
        default: 
            std::cout << "[ERROR] Payload controller entered invalid state." << std::endl;
    }
}


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


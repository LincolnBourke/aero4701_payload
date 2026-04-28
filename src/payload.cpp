#include "payload.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

Payload::Payload()
{
    trajectory_path = "../data/trajectory.csv";
};

Payload::~Payload(){};

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
            std::string p_x, p_y, p_z, q_x, q_y, q_z, q_w;

            // Iterate through entries in the line  
            if (std::getline(ss, p_x, ',') &&
                std::getline(ss, p_y, ',') &&
                std::getline(ss, p_z, ',') &&
                std::getline(ss, q_x, ',') &&
                std::getline(ss, q_y, ',') &&
                std::getline(ss, q_z, ',') &&
                std::getline(ss, q_w))
            {
                PlatformPose pose {
                    Vector3f(std::stof(p_x), std::stof(p_y), std::stof(p_z)),
                    Quaternionf(std::stof(q_w), std::stof(q_x), std::stof(q_y), std::stof(q_z))
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
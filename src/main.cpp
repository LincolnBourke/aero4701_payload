#include "stewartPlatform.hpp"
#include "payload.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Quaternionf;

int main() 
{
    // Payload payload; 
    // payload.readTrajectory();
    // payload.printTrajectory();

    StewartPlatformAnalyser platform_analyser; 
    platform_analyser.generatePointCloud("../data/point_cloud2.csv");
}
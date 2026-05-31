#include "stewartPlatform.hpp"
#include "payloadController.hpp"

int main()
{
    PayloadController payload;
    payload.run();

    // payload.generateTrajectoryAnglesFile("../data/angles.csv");

    // StewartPlatformAnalyser platform_analyser; 
    // platform_analyser.generatePointCloud("../data/point_cloud.csv");
}
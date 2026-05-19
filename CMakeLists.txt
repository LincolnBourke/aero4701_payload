cmake_minimum_required(VERSION 3.16)
project(aero4701_payload VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})

lcm_add_library(payload_messages_lcm CPP lcm_messages/payloadMessages.lcm)

add_subdirectory(apps/payload_controller)

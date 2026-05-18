cmake_minimum_required(VERSION 3.16)
project(aero4701_payload VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})

lcm_add_library(payload_messages_lcm CPP lcm_messages/payloadMessages.lcm)

add_executable(payload
    src/main.cpp
    src/payload.cpp
    src/lcmHandler.cpp
    src/stewartPlatform.cpp
    src/servoMotor.cpp
)

target_include_directories(payload PRIVATE
    "include" 
    "lcm_messages/"
    "lcm_messages/payload_messages"
)


target_link_libraries(payload PRIVATE 
    Eigen3::Eigen
    ${LCM_NAMESPACE}lcm 
    payload_messages_lcm
)


target_compile_options(payload PRIVATE -Werror -Wall -Wextra)

add_executable(test_lcm_publisher tests/test_lcm_publisher.cpp)

target_include_directories(test_lcm_publisher PRIVATE
    include
    "lcm_messages/payload_messages"
)

target_link_libraries(test_lcm_publisher PRIVATE
    ${LCM_NAMESPACE}lcm
    payload_messages_lcm
)

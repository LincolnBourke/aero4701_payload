cmake_minimum_required(VERSION 3.16)
project(aero4701_payload VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(aero4701_payload 
    src/main.cpp
    src/stewartPlatform.cpp
)

target_include_directories(aero4701_payload PRIVATE include)
target_link_libraries(aero4701_payload PRIVATE Eigen3::Eigen)

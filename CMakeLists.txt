cmake_minimum_required(VERSION 3.16)
project(aero4701_payload VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# LCM's installed cmake modules use cmake_minimum_required with versions < 3.5,
# which triggers deprecation warnings in CMake 3.27+. Suppress for LCM import.
set(CMAKE_WARN_DEPRECATED OFF CACHE INTERNAL "")
# find_package(lcm REQUIRED)
# include(${LCM_USE_FILE})

# Required to use PkgConfig on Raspberry Pi linux version due to old 
find_package(PkgConfig REQUIRED)
pkg_check_modules(lcm REQUIRED IMPORTED_TARGET lcm)

unset(CMAKE_WARN_DEPRECATED CACHE)

# lcm_add_library(payload_messages_lcm CPP lcm_messages/payloadMessages.lcm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lcm_messages)

add_subdirectory(apps/payload_controller)
add_subdirectory(apps/servo_controller)
add_subdirectory(apps/servo_monitor)
add_subdirectory(apps/servo_state_reader)
add_subdirectory(apps/camera)
# add_subdirectory(apps/limit_switches)
add_subdirectory(apps/obc_bridge)
add_subdirectory(apps/switch_monitor)


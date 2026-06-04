# Find libgpiod via pkg-config (install with: sudo apt install libgpiod-dev)
find_package(PkgConfig REQUIRED)

add_executable(switch_monitor
    src/main.cpp
    src/switchMonitor.cpp
)

target_include_directories(switch_monitor PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(switch_monitor PRIVATE
    PkgConfig::lcm
)

target_compile_options(switch_monitor PRIVATE -Werror -Wall -Wextra)

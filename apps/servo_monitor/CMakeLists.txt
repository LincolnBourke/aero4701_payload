add_executable(servo_monitor
    src/main.cpp
    src/servoMonitor.cpp
)

target_include_directories(servo_monitor PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(servo_monitor PRIVATE
    PkgConfig::lcm
)

target_compile_options(servo_monitor PRIVATE -Werror -Wall -Wextra)

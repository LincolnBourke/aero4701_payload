add_executable(servo_controller
    src/main.cpp
    src/servoController.cpp
)

target_include_directories(servo_controller PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(servo_controller PRIVATE
    PkgConfig::lcm
)

target_compile_options(servo_controller PRIVATE -Werror -Wall -Wextra)

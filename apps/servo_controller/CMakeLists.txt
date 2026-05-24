add_executable(servo_controller
    src/main.cpp
    src/servo_controller.cpp
)

target_include_directories(servo_controller PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(servo_controller PRIVATE
    ${LCM_NAMESPACE}lcm
    payload_messages_lcm
)

target_compile_options(servo_controller PRIVATE -Werror -Wall -Wextra)

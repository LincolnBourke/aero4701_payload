add_executable(servo_state_reader
    src/main.cpp
    src/servo_state_reader.cpp
)

target_include_directories(servo_state_reader PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(servo_state_reader PRIVATE
    PkgConfig::lcm
)

target_compile_options(servo_state_reader PRIVATE -Werror -Wall -Wextra)

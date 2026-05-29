add_executable(payload_controller
    src/main.cpp
    src/payloadController.cpp
    src/lcmHandler.cpp
    src/stewartPlatform.cpp
    src/servoMotor.cpp
)

target_include_directories(payload_controller PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(payload_controller PRIVATE
    Eigen3::Eigen
    PkgConfig::lcm
)

target_compile_options(payload_controller PRIVATE -Werror -Wall -Wextra)

add_executable(test_kinematics_closure
    tests/test_kinematics_closure.cpp
    src/stewartPlatform.cpp
    src/servoMotor.cpp
)

target_include_directories(test_kinematics_closure PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(test_kinematics_closure PRIVATE
    Eigen3::Eigen
    PkgConfig::lcm
)

add_executable(test_lcm_publisher tests/test_lcm_publisher.cpp)

target_include_directories(test_lcm_publisher PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(test_lcm_publisher PRIVATE
    PkgConfig::lcm
)

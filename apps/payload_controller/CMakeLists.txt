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
    tests/testKinematicsClosure.cpp
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

add_executable(run_controller 
    tests/publishRunController.cpp
)

target_include_directories(run_controller PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(run_controller PRIVATE
    PkgConfig::lcm
)


add_executable(camera_payload_controller
    src/main.cpp
    tests/cameraPayloadController.cpp
    src/lcmHandler.cpp
    src/stewartPlatform.cpp
    src/servoMotor.cpp
)

target_include_directories(camera_payload_controller PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(camera_payload_controller PRIVATE
    Eigen3::Eigen
    PkgConfig::lcm
)

target_compile_options(camera_payload_controller PRIVATE -Werror -Wall -Wextra)


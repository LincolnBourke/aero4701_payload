add_executable(dummy_payload_controller
    tests/dummyPayloadController.cpp
)

target_link_libraries(dummy_payload_controller PRIVATE
    PkgConfig::lcm
)
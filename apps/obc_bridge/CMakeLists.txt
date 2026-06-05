add_executable(obc_bridge
    src/main.cpp
    src/obcBridge.cpp
    src/obcBridgeLcmHandler.cpp
    src/uartInterface.cpp
    src/obcMessageHandler.cpp
)

target_include_directories(obc_bridge PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(obc_bridge PRIVATE
    PkgConfig::lcm
)

target_compile_options(obc_bridge PRIVATE -Werror -Wall -Wextra)

add_executable(test_obc_comms
    tests/test_obc_comms.cpp
    src/obcBridge.cpp
    src/obcBridgeLcmHandler.cpp
    src/uartInterface.cpp
    src/obcMessageHandler.cpp
)

target_include_directories(test_obc_comms PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(test_obc_comms PRIVATE
    PkgConfig::lcm
)

target_compile_options(test_obc_comms PRIVATE -Werror -Wall -Wextra)
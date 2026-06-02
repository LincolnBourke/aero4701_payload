# Find libgpiod via pkg-config (install with: sudo apt install libgpiod-dev)
find_package(PkgConfig REQUIRED)
pkg_check_modules(gpiod REQUIRED IMPORTED_TARGET libgpiod)

add_executable(limit_switch_reader
    src/main.cpp
    src/limitSwitchReader.cpp
)

target_include_directories(limit_switch_reader PRIVATE
    include
    ${CMAKE_SOURCE_DIR}/lcm_messages
    ${CMAKE_SOURCE_DIR}/lcm_messages/payload_messages
)

target_link_libraries(limit_switch_reader PRIVATE
    PkgConfig::lcm
    PkgConfig::gpiod 
)

target_compile_options(limit_switch_reader PRIVATE -Werror -Wall -Wextra)

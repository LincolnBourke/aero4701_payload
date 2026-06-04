#include "run_command_t.hpp"
#include "commands.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>

int main(int argc, char* argv[])
{
    lcm::LCM lcm;

    if (!lcm.good()) {
        std::cerr << "Failed to create LCM instance" << std::endl;
        return 1;
    }

    payload_messages::run_command_t msg;
    msg.command_id = Commands::RunId::RUN_CONTROLLER;

    std::cout << "Publishing payload controller start message..." << std::endl;
    std::cout << "Publishing command_id: " << (int)msg.command_id << std::endl;
    lcm.publish("RUN_COMMAND", &msg);

    std::cout << "Done!" << std::endl;
    return 0;
}

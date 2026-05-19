#include "run_command_t.hpp"
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

    std::cout << "Publishing run_command messages..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        msg.command_id = i;
        std::cout << "Publishing command_id: " << (int)msg.command_id << std::endl;
        lcm.publish("RUN_COMMAND", &msg);
        sleep(1);
    }

    std::cout << "Done!" << std::endl;
    return 0;
}

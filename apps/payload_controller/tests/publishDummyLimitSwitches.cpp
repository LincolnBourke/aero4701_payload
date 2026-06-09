#include "switch_state_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>

int main()
{
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to create LCM instance" << std::endl;
        return 1;
    }

    std::cout << "[INFO] Publishing dummy limit switch states (all activated) at 10Hz" << std::endl;

    while (true)
    {
        payload_messages::switch_state_t msg;
        msg.switch1 = 1;
        msg.switch2 = 1;
        msg.switch3 = 1;
        lcm.publish("LIMIT_SWITCH_STATES", &msg);
        usleep(100000); // 100ms
    }

    return 0;
}

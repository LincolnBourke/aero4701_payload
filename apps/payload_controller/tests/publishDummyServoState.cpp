#include "true_servo_angles_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main()
{
    lcm::LCM lcm;
    if (!lcm.good()) {
        std::cerr << "Failed to create LCM instance" << std::endl;
        return 1;
    }

    std::cout << "[INFO] Publishing dummy servo states at 50Hz on SERVO_STATE" << std::endl;

    float t = 0.0f;
    const float dt     = 0.02f;           // 20ms = 50Hz
    const float centre = M_PI / 4.0f;    // 45 deg midpoint
    const float amp    = M_PI / 8.0f;    // +/-22.5 deg swing

    while (true)
    {
        payload_messages::true_servo_angles_t msg;
        for (int i = 0; i < 6; i++)
            msg.angles[i] = centre + amp * sinf(t + i * (float)M_PI / 3.0f);

        lcm.publish("SERVO_STATE", &msg);
        t += dt;
        usleep(20000); // 20ms
    }

    return 0;
}

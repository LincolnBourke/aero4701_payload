#include "servo_controller.hpp"
#include <iostream>

int main()
{
    ServoController controller("/dev/ttyAMA0", 6, "SERVO_STATE");
    std::cout << "Waiting for LCM messages..." << std::endl;
    while (0 == controller.lcm.handle());
    return 0;
}

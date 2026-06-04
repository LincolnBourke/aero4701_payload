#include "servoMonitor.hpp"
#include <iostream>

int main()
{
    ServoMonitor monitor(6, "SERVO_STATE");
    std::cout << "Listening for servo data... (Ctrl+C to stop)" << std::endl;
    while (true)
    {
        if (monitor.lcm.handle() != 0) break;
    }
    return 0;
}

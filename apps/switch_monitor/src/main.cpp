#include "switchMonitor.hpp"
#include <iostream>

int main()
{
    std::string channel_name = "LIMIT_SWITCH_STATES";

    SwitchMonitor monitor(channel_name);

    std::cout << "Listening for switch data... (Ctrl+C to stop)" << std::endl;
    while (true)
    {
        if (monitor.lcm.handle() != 0) break;
    }
    return 0; 
}

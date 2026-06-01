#include "switchMonitor.hpp"


// == Constructor ===============================================================
SwitchMonitor::SwitchMonitor(const std::string& channel_name)
    : _channel_name(channel_name)  { 

    std::cout << "Starting Switch Monitor on channel: " << _channel_name << std::endl;

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return;
    }

    // Subscribe to the channel
    // Syntax: Topic, Member Function Pointer, Instance Pointer
    lcm.subscribe(_channel_name, &SwitchMonitor::handleSwitchMsg, this);
}

void SwitchMonitor::handleSwitchMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const payload_messages::switch_state_t* msg) {
    (void)rbuf; // Cast to void to avoid unused parameter compiler warnings 

    std::cout << "\n-----------------------------------" << std::endl;
    std::cout << "RECEIVE EVENT" << std::endl;
    std::cout << "Channel:   " << chan << std::endl;
    std::cout << "Timestamp: " << msg->timestamp << std::endl;
    std::cout << "Switches:  [ "
          << static_cast<int>(msg->switch1) << ", "
          << static_cast<int>(msg->switch2) << ", "
          << static_cast<int>(msg->switch3) << " ]"
          << std::endl;
}
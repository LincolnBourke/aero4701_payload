#include "servoMonitor.hpp"

#include <iostream>
#include <iomanip>

ServoMonitor::ServoMonitor(int num_motors, const std::string& channel_name)
    : _num_motors(num_motors), _channel_name(channel_name) {

    std::cout << "Starting Servo Monitor on channel: " << _channel_name << std::endl;

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return;
    }

    // Subscribe to the channel
    // Syntax: Topic, Member Function Pointer, Instance Pointer
    lcm.subscribe(_channel_name, &ServoMonitor::handleServoAngMsg, this);
}

void ServoMonitor::handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const payload_messages::servo_angs* msg) {
    (void)rbuf; // Cast to void to avoid unused parameter compiler warnings 

    std::cout << "\n-----------------------------------" << std::endl;
    std::cout << "RECEIVE EVENT" << std::endl;
    std::cout << "Channel:   " << chan << std::endl;
    std::cout << "Timestamp: " << msg->timestamp << std::endl;
    std::cout << "Angles:    [ ";

    // Use the size from the message itself for safety
    for (size_t i = 0; i < msg->position.size(); ++i) {
        std::cout << std::fixed << std::setprecision(2) << msg->position[i];
        if (i < msg->position.size() - 1) std::cout << ", ";
    }
    std::cout << " ]" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
}

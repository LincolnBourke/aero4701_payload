/*
    Defines a class for monitoring servo angle messages published on an
    LCM channel. Prints each incoming message to stdout for debugging.
*/

#ifndef SERVO_MONITOR_H
#define SERVO_MONITOR_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "payload_messages/servo_angs.hpp"

class ServoMonitor
{
    public:
        ServoMonitor(int num_motors, const std::string& channel_name);

        // Handles incoming servo angle messages and prints them to stdout.
        void handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
                               const std::string& chan,
                               const payload_messages::servo_angs* msg);

        // Public LCM instance used by main to pump the message loop.
        lcm::LCM lcm;

    private:
        // Number of servo motors being monitored.
        int _num_motors;

        std::string _channel_name;
};

#endif

/*
    Defines a class for monitoring switch state messages published on an
    LCM channel. Prints each incoming message to stdout for debugging.
*/

#ifndef SWITCH_MONITOR_H
#define SWITCH_MONITOR_H

#include <iostream>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "payload_messages/switch_state_t.hpp"

class SwitchMonitor
{
    public:
        SwitchMonitor(const std::string& channel_name);

        // Handles incoming servo angle messages and prints them to stdout.
        void handleSwitchMsg(const lcm::ReceiveBuffer* rbuf,
            const std::string& chan,
            const payload_messages::switch_state_t* msg);

        // Public LCM instance used by main to pump the message loop.
        lcm::LCM lcm;

    private:
        std::string _channel_name;
};

#endif

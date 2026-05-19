/*
Handles LCM messaging for the payload controller node. 
*/

#ifndef LCMHANDLER_H
#define LCMHANDLER_H

#include "run_command_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string.h>


class LcmHandler 
{
    private: 
        // Store the latest messages until they are read 
        payload_messages::run_command_t last_run_command_msg;
        bool run_command_received;

    public:
        LcmHandler();
        ~LcmHandler();

        // Message publishing functions
        // void publishCameraCommand(int8_t command_id);

        // Message handling functions
        void handleRunCommand(const lcm::ReceiveBuffer* rbuf, 
            const std::string& channel,
            const payload_messages::run_command_t* msg);
        
        // Returns true if a run command was received and stores the message id
        bool checkRunCommand(int& command_id);
};

#endif
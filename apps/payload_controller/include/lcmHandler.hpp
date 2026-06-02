/*
Handles LCM messaging for the payload controller node. 
*/

#ifndef LCMHANDLER_H
#define LCMHANDLER_H

#include "run_command_t.hpp"
#include "save_complete_t.hpp"
#include "switch_state_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string.h>

class LcmHandler 
{
    private:
        // Store the latest messages until they are read
        payload_messages::run_command_t last_run_command_msg;
        bool run_command_received;

        payload_messages::save_complete_t last_save_complete_msg;
        bool save_complete_received;

        payload_messages::switch_state_t last_switch_state_msg;
        bool switch_state_received;

    public:
        LcmHandler();
        ~LcmHandler();

        // Message handling functions
        void handleRunCommand(const lcm::ReceiveBuffer* rbuf,
            const std::string& channel,
            const payload_messages::run_command_t* msg);

        void handleSaveComplete(const lcm::ReceiveBuffer* rbuf,
            const std::string& channel,
            const payload_messages::save_complete_t* msg);

        void handleSwitchStateMsg(const lcm::ReceiveBuffer* rbuf,
            const std::string& channel,
            const payload_messages::switch_state_t* msg);

        // Returns true if a message was received and stores the id
        bool checkRunCommand(int& command_id);
        bool checkSaveComplete(int& return_id);
        bool checkSwitchState( int (&switch_states)[3] ); 
};

#endif
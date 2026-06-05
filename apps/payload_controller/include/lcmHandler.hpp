/*
Handles LCM messaging for the payload controller node. 
*/

#ifndef LCMHANDLER_H
#define LCMHANDLER_H

#include "run_command_t.hpp"
#include "save_complete_t.hpp"
#include "cam_msg_t.hpp"

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

        payload_messages::cam_msg_t last_cam_msg;
        bool cam_status_received;

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

        void handleCamMsg(const lcm::ReceiveBuffer* rbuf,
            const std::string& channel,
            const payload_messages::cam_msg_t* msg);

        // Returns true if a message was received and stores the id
        bool checkRunCommand(int& command_id);
        bool checkSaveComplete(int& return_id);
        
        // Reset to clear for camera communications
        void reset();
        
        // Getters for receiving and contents of camera status
        bool isCamStatusReceived() const { return cam_status_received; }
        bool getCamStatus() const { return last_cam_msg.cam_status; }
};

#endif
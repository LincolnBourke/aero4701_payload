/*
Handles LCM subscriptions for the OBC bridge node. 
*/

#ifndef OBC_BRIDGE_LCM_HANDLER_H
#define OBC_BRIDGE_LCM_HANDLER_H

#include "run_result_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string.h>


class ObcBridgeLcmHandler 
{
    private:
        // Store the latest messages until they are read
        payload_messages::run_result_t last_run_result_msg;
        bool run_result_received;

    public:
        ObcBridgeLcmHandler();
        ~ObcBridgeLcmHandler();

        // Message handling functions
        void handleRunResult(const lcm::ReceiveBuffer* rbuf,
            const std::string& channel,
            const payload_messages::run_result_t* msg);

        // Returns true if a message was received and stores the id
        bool checkRunResult(int& return_id);
};

#endif
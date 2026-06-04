#include "obcBridgeLcmHandler.hpp"

#include <iostream>

ObcBridgeLcmHandler::ObcBridgeLcmHandler()
    : last_run_result_msg(), run_result_received(false)
{};

ObcBridgeLcmHandler::~ObcBridgeLcmHandler() {};

// --- Subscription methods ----------------------------------------------------

void ObcBridgeLcmHandler::handleRunResult(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::run_result_t* msg)
{
    (void)rbuf; // Avoid compile error

    printf("[INFO] Received message on channel %s with return_id %d\n", channel.c_str(), msg->return_id);
    
    run_result_received = true;
    last_run_result_msg = *msg;  
}

bool ObcBridgeLcmHandler::checkRunResult(int& return_id)
{
    if (run_result_received == false)
    {
        return false;
    }

    return_id = last_run_result_msg.return_id;

    run_result_received = false;
    return true;
}
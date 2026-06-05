#include "lcmHandler.hpp"

#include <iostream>

LcmHandler::LcmHandler()
    : last_run_command_msg(), run_command_received(false),
      last_save_complete_msg(), save_complete_received(false),
      last_cam_msg(), cam_status_received(false)
{};

LcmHandler::~LcmHandler() {};

// --- Subscription methods ----------------------------------------------------

void LcmHandler::handleRunCommand(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::run_command_t* msg)
{
    printf("[INFO] Received message on channel %s with command_id %d\n", channel.c_str(), msg->command_id);
    
    run_command_received = true;
    last_run_command_msg = *msg;

    rbuf = rbuf; // Avoid compile error   
}

bool LcmHandler::checkRunCommand(int& command_id)
{
    if (run_command_received == false)
    {
        return false;
    }

    command_id = last_run_command_msg.command_id;

    run_command_received = false;
    return true;
}

void LcmHandler::handleSaveComplete(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::save_complete_t* msg)
{
    printf("[INFO] Received message on channel %s with return_id %d\n", channel.c_str(), msg->return_id);

    save_complete_received = true;
    last_save_complete_msg = *msg;

    rbuf = rbuf;
}

bool LcmHandler::checkSaveComplete(int& return_id)
{
    if (save_complete_received == false)
    {
        return false;
    }

    return_id = last_save_complete_msg.return_id;

    save_complete_received = false;
    return true;
}

// // no longer needed? but leaving in case 
// bool LcmHandler::checkCamStatus(bool& cam_status)
// {
//     if (cam_status_received == false)
//     {
//         return false;
//     }

//     cam_status = last_cam_msg.cam_status;

//     cam_status_received = false;
//     return true;
// }

void LcmHandler::handleCamMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::cam_msg_t* msg)
{
    printf("[INFO] Received message on channel %s with cam_status %d\n", channel.c_str(), (int)msg->cam_status);

    cam_status_received = true;
    last_cam_msg = *msg;

    rbuf = rbuf;
}

void LcmHandler::reset()
{
    cam_status_received  = false;
    last_cam_msg         = payload_messages::cam_msg_t{}; // not really needed
}
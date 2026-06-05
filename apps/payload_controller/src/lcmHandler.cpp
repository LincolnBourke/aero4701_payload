#include "lcmHandler.hpp"

#include <iostream>

LcmHandler::LcmHandler()
    : last_run_command_msg(), run_command_received(false),
      last_save_complete_msg(), save_complete_received(false),
<<<<<<< HEAD
      last_switch_state_msg(), switch_state_received(false), 
      all_switched(false)
=======
      last_cam_msg(), cam_status_received(false)
>>>>>>> master
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

<<<<<<< HEAD
void LcmHandler::handleSwitchStateMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::switch_state_t* msg)
{
    // printf("[INFO] Received message on channel %s with states [%d, %d, %d]\n", 
    //         channel.c_str(), 
    //         static_cast<int>(msg->switch1),
    //         static_cast<int>(msg->switch2),
    //         static_cast<int>(msg->switch3) );

    // Check if all have been switched 
    int switch_states[3] = {
        static_cast<int>(msg->switch1),
        static_cast<int>(msg->switch2),
        static_cast<int>(msg->switch3) 
    };

    // Activate if any triggered 
    if (switch_states[0] || switch_states[1] || switch_states[2])
    // if (switch_states[0] && switch_states[1] && switch_states[2])
    {
        all_switched = true; // Latch until read 
    }

    switch_state_received = true;
    last_switch_state_msg = *msg;

    // Avoid compile errors
    (void)channel; 
    (void)rbuf;
}

bool LcmHandler::checkSwitchState( int (&switch_states)[3], bool &all_flag ) 
{
    if ((switch_state_received == false) && (!all_switched)) 
    {
        return false;
    }

    // Store message 
    switch_states[0] = static_cast<int>(last_switch_state_msg.switch1);
    switch_states[1] = static_cast<int>(last_switch_state_msg.switch2);
    switch_states[2] = static_cast<int>(last_switch_state_msg.switch3);

    // If all activated, flag  
    if (all_switched) 
    {
        all_flag = true; 
        all_switched = false; // Unlatch 
    }
    else 
    {
        all_flag = false; 
    }

    // std::cout << "[INFO] Returning switch states [";
    // std::cout << static_cast<int>(last_switch_state_msg.switch1) << ", "; 
    // std::cout << static_cast<int>(last_switch_state_msg.switch2) << ", "; 
    // std::cout << static_cast<int>(last_switch_state_msg.switch2) << "]"; 
    // std::cout << std::endl << std::flush; 

    switch_state_received = false;
    return true;
}

void LcmHandler::handleServoStateMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::true_servo_angles_t* msg)
{ 
    // printf("[INFO] Received message on channel %s\n", channel.c_str());

    servo_angs_received = true;
    last_servo_angs_msg = *msg;

    // Avoid compile errors
    (void)channel; 
    (void)rbuf;
}

bool LcmHandler::checkServoAngs( float (&servo_angs)[6] ) {
    if (!servo_angs_received) return false;

    // Assign each index
    for (int i = 0; i < 6; ++i) {
        servo_angs[i] = last_servo_angs_msg.angles[i];
    }

    servo_angs_received = false;
    return true;
}
=======
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
>>>>>>> master

#include "lcmHandler.hpp"

#include <iostream>

LcmHandler::LcmHandler() 
    : last_run_command_msg(), run_command_received(false)
{};

LcmHandler::~LcmHandler() {};

void LcmHandler::handleRunCommand(const lcm::ReceiveBuffer* rbuf,
    const std::string& channel, const payload_messages::run_command* msg)
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


/*
void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::example_t* msg)
        {
            int i;
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->timestamp);
            printf("  position    = (%f, %f, %f)\n",
                    msg->position[0], msg->position[1], msg->position[2]);
            printf("  orientation = (%f, %f, %f, %f)\n",
                    msg->orientation[0], msg->orientation[1], 
                    msg->orientation[2], msg->orientation[3]);
            printf("  ranges:");
            for(i = 0; i < msg->num_ranges; i++)
                printf(" %d", msg->ranges[i]);
            printf("\n");
            printf("  name        = '%s'\n", msg->name.c_str());
            printf("  enabled     = %d\n", msg->enabled);
        }
*/
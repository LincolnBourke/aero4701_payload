/*
Entry point for the OBC bridge executable. Handles high level logic of processing
commands from the teensy, communicating with the payload controller and preparing
to send data back over UART. 
*/

#ifndef OBC_BRIDGE_H
#define OBC_BRDIGE_H

#include "obcBridgeLcmHandler.hpp"
#include "obcMessageHandler.hpp"

#include <lcm/lcm-cpp.hpp>
#include <chrono>

// States for the top level OBC-payload comms state machine
enum class ObcBridgeState 
{
    IDLE, 
    RECEIVE_SETTINGS,
    DO_EXPERIMENT,
    TRANSMIT_EXPERIMENT_RESULTS,
    TRANSMIT_EXPERIMENT_ERROR,
    DEBUG,
    TRANSMIT_DEBUG_RESULTS
};

// States for the RECEIVE_SETTINGS state machine
enum class ReceiveSettingsState
{
    WAIT_HEADER,
    WAIT_PACKET,
    WAIT_TRANSFER_COMPLETE
};

// States for the TRANSMIT_RESULT state machine
enum class TransmitResultState
{
    REQUEST_TRANSFER,
    WAIT_TRANSFER_ACK,
    SEND_HEADER,
    WAIT_HEADER_ACK,
    SEND_PACKET,
    WAIT_PACKET_ACK,
    TRANSFER_COMPLETE,
    TRANSFER_COMPLETE_ACK
};

enum class TransmitErrorState
{
    REQUEST_TRANSFER,
    WAIT_TRANSFER_ACK
}; // TODO: finish specifying states

class ObcBridge
{
    private:
        // For communication between nodes on the payload computer
        lcm::LCM lcm;
        ObcBridgeLcmHandler lcm_handler;

        // LCM message publishers
        void publishRunCommand(int8_t command_id);

    protected:
        // For communication with the OBC
        ObcMessageHandler obc_messager;

        // For timing acknowledgement wait timeouts
        std::chrono::time_point<std::chrono::steady_clock> timer_start;

        // readTime returns the time in ms since the last startTimer() call
        void startTimer();
        float readTime();

        // State handlers for the main OBC-payload comms state machine
        ObcBridgeState handleIdleState();
        ObcBridgeState handleReceiveSettingsState();
        ObcBridgeState handleDoExperimentState();
        ObcBridgeState handleTransmitResultState();
        ObcBridgeState handleTransmitErrorState();
        ObcBridgeState handleDebugState();
        ObcBridgeState handleTransmitDebugResultsState();

    public: 
        // Setup UART port 
        ObcBridge();
        ~ObcBridge();

        // Run the main event loop
        void run();
};

#endif
#include "obcBridge.hpp"
#include "run_command_t.hpp"
#include "commands.hpp"

#include <iostream>

// Time to wait for an acknowledgement before repeating a message sent over UART
#define WAIT_ACK_TIMEOUT 1000 // ms
#define RESULTS_FILEPATH ""

ObcBridge::ObcBridge()
    : lcm(), lcm_handler(), obc_messager()
{
    if (!lcm.good())
    {
        std::cout << "[ERROR] LCM not good." << std::endl;
    }

    // Subscribe lcm handler to messages
    lcm.subscribe("RUN_RESULT", &ObcBridgeLcmHandler::handleRunResult, &lcm_handler);
};

ObcBridge::~ObcBridge() {};

// Runs the top-level state machine
void ObcBridge::run()
{
    ObcBridgeState state = ObcBridgeState::IDLE;

    while (true)
    {
        switch (state)
        {
            case ObcBridgeState::IDLE:
                state = handleIdleState();
                break;
            case ObcBridgeState::RECEIVE_SETTINGS:
                state = handleReceiveSettingsState();
                break;
            case ObcBridgeState::DO_EXPERIMENT:
                state = handleDoExperimentState();
                break;
            case ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS:
                state = handleTransmitResultState();
                break;
            case ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR:
                state = handleTransmitErrorState();
                break;
            case ObcBridgeState::DEBUG:
                state = handleDebugState();
                break;
            case ObcBridgeState::TRANSMIT_DEBUG_RESULTS:
                state = handleTransmitDebugResultsState();
                break;
        }
    }
}

// --- Core ObcBridge state machine logic --------------------------------------

ObcBridgeState ObcBridge::handleIdleState()
{
    // Transition state depending on OBC message sent over UART
    if (obc_messager.checkStartMsg())
    {
        obc_messager.transmitStartAck();
        return ObcBridgeState::DO_EXPERIMENT;
    }
    else if (obc_messager.checkTransferRequest())
    {
        // Assume transfer requests are for the experiment settings file
        obc_messager.transmitTransferAck();
        return ObcBridgeState::RECEIVE_SETTINGS;
    }
    else if (obc_messager.checkEnterDebugMsg())
    {
        obc_messager.transmitDebugAck();
        return ObcBridgeState::DEBUG;
    }

    return ObcBridgeState::IDLE;
}

ObcBridgeState ObcBridge::handleReceiveSettingsState()
{
    ReceiveSettingsState state = ReceiveSettingsState::WAIT_HEADER;
    float ack_timeout = ACK_TIMEOUT;
    int packet_idx = 0;
    int num_packets = 0; // TODO: populated from header

    startTimer();

    while (true)
    {
        switch (state)
        {
            case ReceiveSettingsState::WAIT_HEADER:
                if (obc_messager.checkHeader() == true)
                {
                    obc_messager.transmitHeaderAck();
                    startTimer();
                    state = ReceiveSettingsState::WAIT_PACKET;
                }
                else if (readTime() > ack_timeout)
                    startTimer();
                break;

            case ReceiveSettingsState::WAIT_PACKET:
                if (obc_messager.checkPacket() == true)
                {
                    obc_messager.transmitPacketAck();
                    if (packet_idx < num_packets - 1)
                    {
                        packet_idx++;
                        startTimer();
                        state = ReceiveSettingsState::WAIT_PACKET;
                    }
                    else
                    {
                        startTimer();
                        state = ReceiveSettingsState::WAIT_TRANSFER_COMPLETE;
                    }
                }
                else if (readTime() > ack_timeout)
                {
                    startTimer();
                    state = ReceiveSettingsState::WAIT_HEADER;
                }
                break;

            case ReceiveSettingsState::WAIT_TRANSFER_COMPLETE:
                if (obc_messager.checkTransferComplete() == true)
                    return ObcBridgeState::IDLE;
                else if (readTime() > ack_timeout)
                {
                    startTimer();
                    state = ReceiveSettingsState::WAIT_HEADER;
                }
                break;
        }
    }

    return ObcBridgeState::RECEIVE_SETTINGS; // should never reach this line
}

ObcBridgeState ObcBridge::handleDoExperimentState()
{
    int run_result_id;

    // Start the payload experiment
    publishRunCommand(Commands::RunId::RUN_CONTROLLER);

    // Poll for stop run messages from the OBC and run complete messages from 
    // the payload controller
    while (true)
    {
        if (lcm_handler.checkRunResult(run_result_id) == true)
        {
            if (run_result_id == Commands::RunResult::RUN_SUCCESS)
                return ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS;

            if (run_result_id == Commands::RunResult::RUN_FAIL)
                return ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR;
        }

        if (obc_messager.checkStopMsg())
        {
            publishRunCommand(Commands::RunId::STOP_CONTROLLER);
            return ObcBridgeState::IDLE;
        }
    }

    return ObcBridgeState::DO_EXPERIMENT; // should never reach this line
}

ObcBridgeState ObcBridge::handleTransmitResultState()
{
    TransmitResultState state = TransmitResultState::REQUEST_TRANSFER;
    float ack_timeout = ACK_TIMEOUT;
    bool transmit_success;

    // Read and serialise the results before entering state machine
    if (!obc_messager.serialiseResults(RESULTS_FILEPATH))
    {
        std::cout << "[ERROR] Failed to serialise results file." << std::endl;
        return ObcBridgeState::IDLE;
    }

    while (true)
    {
        switch (state)
        {
            case TransmitResultState::REQUEST_TRANSFER:
                transmit_success = obc_messager.transmitTransferRequest();
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit results transfer request to OBC." << std::endl;
                    return ObcBridgeState::IDLE;
                }

                startTimer();
                state = TransmitResultState::WAIT_TRANSFER_ACK;
                break;
            
            case TransmitResultState::WAIT_TRANSFER_ACK:
                if (obc_messager.checkTransferAck() == true)
                    state = TransmitResultState::SEND_HEADER;
                else if (readTime() > ack_timeout)
                    state = TransmitResultState::REQUEST_TRANSFER;
                break;
            
            case TransmitResultState::SEND_HEADER:
                transmit_success = obc_messager.transmitHeader();
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit results header to OBC." << std::endl;
                    return ObcBridgeState::IDLE;
                }

                startTimer();
                state = TransmitResultState::WAIT_HEADER_ACK;
                break;

            case TransmitResultState::WAIT_HEADER_ACK:
                if (obc_messager.checkHeaderAck() == true)
                    state = TransmitResultState::SEND_PACKET;
                else if (readTime() > ack_timeout)
                    state = TransmitResultState::SEND_HEADER;
                break;

            case TransmitResultState::SEND_PACKET:
                transmit_success = obc_messager.transmitResultsPacket();
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit results packet to OBC, retrying after timeout..." << std::endl;
                }

                startTimer();
                state = TransmitResultState::WAIT_PACKET_ACK;
                break;

            case TransmitResultState::WAIT_PACKET_ACK:
                if (obc_messager.checkPacketAck() == true)
                {
                    if (!obc_messager.isTransmitQueueEmpty())
                        state = TransmitResultState::SEND_PACKET;
                    else 
                        state = TransmitResultState::TRANSFER_COMPLETE;
                }
                else if (readTime() > ack_timeout)
                {
                    obc_messager.transmitLastResultsPacket(); // retry the transmission
                    startTimer();
                }
                break;

            case TransmitResultState::TRANSFER_COMPLETE:
                transmit_success = obc_messager.transmitTransferComplete();
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit transfer complete signal to OBC." << std::endl;
                }

                return ObcBridgeState::IDLE;
        }
    }

    return ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS; // should never reach this line
}

ObcBridgeState ObcBridge::handleTransmitErrorState()
{
    // TODO: currently does nothing with errors
    return ObcBridgeState::IDLE;
}

ObcBridgeState ObcBridge::handleDebugState()
{
    // TODO: need to implement debug functionality for the payload controller
    return ObcBridgeState::DEBUG;
}

ObcBridgeState ObcBridge::handleTransmitDebugResultsState()
{
    // TODO: need to implement debug functionality for payload controller
    return ObcBridgeState::TRANSMIT_DEBUG_RESULTS;
}

// --- Timer -------------------------------------------------------------------

void ObcBridge::startTimer()
{
    timer_start = std::chrono::steady_clock::now();
}

float ObcBridge::readTime()
{
    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    return std::chrono::duration<float, std::milli>(now - timer_start).count();
}

// --- LCM publishers ----------------------------------------------------------

void ObcBridge::publishRunCommand(int8_t command_id)
{
    payload_messages::run_command_t msg;
    msg.command_id = command_id;
    std::cout << "[INFO] Publishing to RUN_COMMAND: command_id=" << (int)command_id << std::endl;
    lcm.publish("RUN_COMMAND", &msg);
}
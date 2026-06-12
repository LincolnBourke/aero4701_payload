#include "obcBridge.hpp"
#include "run_command_t.hpp"
#include "commands.hpp"
#include "payloadConfig.hpp"

#include <iostream>

// Time to wait for an acknowledgement before repeating a message sent over UART
#define ACK_TIMEOUT        500 // ms
#define DEBUG_IMAGE_WIDTH  640 // pixels
#define DEBUG_IMAGE_HEIGHT 480 // pixels

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

// Returns a human-readable label for each top-level state
static const char* stateName(ObcBridgeState s)
{
    switch (s)
    {
        case ObcBridgeState::STARTUP:                     return "STARTUP";
        case ObcBridgeState::IDLE:                        return "IDLE";
        case ObcBridgeState::RECEIVE_SETTINGS:            return "RECEIVE_SETTINGS";
        case ObcBridgeState::DO_EXPERIMENT:               return "DO_EXPERIMENT";
        case ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS: return "TRANSMIT_EXPERIMENT_RESULTS";
        case ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR:   return "TRANSMIT_EXPERIMENT_ERROR";
        case ObcBridgeState::DEBUG:                       return "DEBUG";
        case ObcBridgeState::TRANSMIT_DEBUG_RESULTS:      return "TRANSMIT_DEBUG_RESULTS";
        default:                                          return "UNKNOWN";
    }
}

// Runs the top-level state machine
void ObcBridge::run()
{
    ObcBridgeState state = ObcBridgeState::STARTUP;
    std::cout << "[INFO] ObcBridge state: " << stateName(state) << std::endl;

    while (true)
    {
        ObcBridgeState next = ObcBridgeState::IDLE;

        switch (state)
        {
            case ObcBridgeState::STARTUP:
                next = handleStartupState();
                break;
            case ObcBridgeState::IDLE:
                next = handleIdleState();
                break;
            case ObcBridgeState::RECEIVE_SETTINGS:
                next = handleReceiveSettingsState();
                break;
            case ObcBridgeState::DO_EXPERIMENT:
                next = handleDoExperimentState();
                break;
            case ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS:
                next = handleTransmitResultState();
                break;
            case ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR:
                next = handleTransmitErrorState();
                break;
            case ObcBridgeState::DEBUG:
                next = handleDebugState();
                break;
            case ObcBridgeState::TRANSMIT_DEBUG_RESULTS:
                next = handleTransmitDebugResultsState();
                break;
        }

        // Log only on actual state transitions
        if (next != state)
            std::cout << "[INFO] ObcBridge state: " << stateName(state)
                      << " -> " << stateName(next) << std::endl;

        state = next;
    }
}

// --- Core ObcBridge state machine logic --------------------------------------

// Only entered when the payload is first turned on. Sends a message to the OBC
// to inform it that the payload has turned on successfully. 
ObcBridgeState ObcBridge::handleStartupState()
{
    float ack_timeout = ACK_TIMEOUT;

    if (obc_messager.transmit(PYLD_ON_ID) == false)
    {
        std::cout << "Failed to send UART message." << std::endl;
    }

    startTimer();

    // Wait for an acknowledge before retrying
    while (true)
    {
        obc_messager.drainUart();

        if (obc_messager.checkAck(PYLD_ON_ID))
            break;

        if (readTime() > ack_timeout)
        {
            obc_messager.transmit(PYLD_ON_ID);
            startTimer();
        }
    }

    // Automatically transition to IDLE when acknowledgement is received
    return ObcBridgeState::IDLE;
}

ObcBridgeState ObcBridge::handleIdleState()
{
    // Drain all available UART messages into the receive queue
    obc_messager.drainUart();

    // Transition state depending on OBC message sent over UART
    if (obc_messager.checkMessage(PYLD_START_ID))
    {
        // Requested to start payload experiment
        obc_messager.transmitAck(PYLD_START_ID);
        return ObcBridgeState::DO_EXPERIMENT;
    }
    else if (obc_messager.checkHeader())
    {
        // Request from OBC to transfer a data file
        // Assume transfer requests are for the experiment settings file
        obc_messager.transmitAck(PYLD_TRANSFER_HEADER_ID);
        return ObcBridgeState::RECEIVE_SETTINGS;
    }
    // else if (obc_messager.checkMessage(PYLD_TRANSFER_HEADER_ID))
    // {
    //     // Request from OBC to transfer a data file
    //     // Assume transfer requests are for the experiment settings file
    //     obc_messager.transmitAck(PYLD_REQUEST_TRANSFER_ID);
    //     return ObcBridgeState::RECEIVE_SETTINGS;
    // }
    // else if (obc_messager.checkMessage(PYLD_REQUEST_TRANSFER_ID))
    // {
    //     // Request from OBC to transfer a data file
    //     // Assume transfer requests are for the experiment settings file
    //     obc_messager.transmitAck(PYLD_REQUEST_TRANSFER_ID);
    //     return ObcBridgeState::RECEIVE_SETTINGS;
    // }
    else if (obc_messager.checkMessage(PYLD_ENTER_DEBUG_ID))
    {
        // Request from OBC to enter debug mode
        obc_messager.transmitAck(PYLD_ENTER_DEBUG_ID);
        return ObcBridgeState::DEBUG;
    }

    return ObcBridgeState::IDLE;
}

ObcBridgeState ObcBridge::handleReceiveSettingsState()
{
    ReceiveSettingsState state = ReceiveSettingsState::WAIT_PACKET; // update from WAIT_HEADER because the header is now the first message recieved

    // Reset file receive buffer from any prior transfer attempt
    obc_messager.clearFileBuffer();

    while (true)
    {
        // Drain all available UART messages into the receive queue once per cycle
        obc_messager.drainUart();

        switch (state)
        {
            case ReceiveSettingsState::WAIT_HEADER:
                if (obc_messager.checkHeader() == true)
                {
                    obc_messager.transmitAck(PYLD_TRANSFER_HEADER_ID);
                    state = ReceiveSettingsState::WAIT_PACKET;
                }
                break;

            case ReceiveSettingsState::WAIT_PACKET:
                if (obc_messager.checkPacket() == true)
                {
                    obc_messager.transmitAck(PYLD_PACKET_ID);
                    if (!obc_messager.isReceiveComplete())
                        state = ReceiveSettingsState::WAIT_PACKET;
                    else
                    {
                        std::cout << "[INFO] All packets received. Waiting for transfer complete signal." << std::endl;
                        state = ReceiveSettingsState::WAIT_TRANSFER_COMPLETE;
                    }
                }
                break;

            case ReceiveSettingsState::WAIT_TRANSFER_COMPLETE:
                if (obc_messager.checkMessage(PYLD_TRANSFER_COMPLETE_ID) == true)
                {
                    // Acknowledge the transfer complete signal before processing
                    obc_messager.transmitAck(PYLD_TRANSFER_COMPLETE_ID);

                    // Deserialise received packets into trajectory and scalar settings files
                    if (!obc_messager.deserialise(TRAJECTORY_SETTINGS_FILEPATH, CAMERA_SETTINGS_FILEPATH))
                        std::cout << "[ERROR] Failed to deserialise received settings file." << std::endl;
                    else
                        std::cout << "[INFO] Settings received and deserialised successfully." << std::endl;

                    return ObcBridgeState::IDLE;
                }
                break;
        }
    }

    return ObcBridgeState::RECEIVE_SETTINGS; // should never reach this line
}

ObcBridgeState ObcBridge::handleDoExperimentState()
{
    return ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS;
    int run_result_id;

    // Start the payload experiment
    publishRunCommand(Commands::RunId::RUN_CONTROLLER);

    // Poll for stop run messages from the OBC and run complete messages from
    // the payload controller
    while (true)
    {
        // Drain all available UART messages into the receive queue once per cycle
        obc_messager.drainUart();

        if (lcm_handler.checkRunResult(run_result_id) == true)
        {
            if (run_result_id == Commands::RunResult::RUN_SUCCESS)
                return ObcBridgeState::TRANSMIT_EXPERIMENT_RESULTS;

            if (run_result_id == Commands::RunResult::RUN_FAIL)
                return ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR;
        }

        if (obc_messager.checkMessage(PYLD_STOP_ID))
        {
            publishRunCommand(Commands::RunId::STOP_CONTROLLER);
            return ObcBridgeState::IDLE;
        }
    }

    return ObcBridgeState::DO_EXPERIMENT; // should never reach this line
}

ObcBridgeState ObcBridge::handleTransmitResultState()
{
    // Serialise results CSV into the transmit queue before entering the state machine
    if (!obc_messager.serialiseResults(RESULTS_FILEPATH))
    {
        std::cout << "[ERROR] Failed to serialise results file." << std::endl;
        return ObcBridgeState::IDLE;
    }
    return runTransmitStateMachine();
}

ObcBridgeState ObcBridge::handleTransmitDebugResultsState()
{
    // Serialise debug JPEG into the transmit queue before entering the state machine
    if (!obc_messager.serialiseDebugResults(DEBUG_RESULTS_FILEPATH))
    {
        std::cout << "[ERROR] Failed to serialise debug results file." << std::endl;
        return ObcBridgeState::IDLE;
    }
    return runTransmitStateMachine();
}

ObcBridgeState ObcBridge::runTransmitStateMachine()
{
    TransmitResultState state = TransmitResultState::SEND_HEADER;
    float ack_timeout = ACK_TIMEOUT;
    bool transmit_success;
    int packet_num = 0;

    while (true)
    {
        // Drain all available UART messages into the receive queue once per cycle
        obc_messager.drainUart();

        switch (state)
        {
            case TransmitResultState::REQUEST_TRANSFER:
                std::cout << "[INFO] Sending transfer request to OBC." << std::endl;
                transmit_success = obc_messager.transmit(PYLD_REQUEST_TRANSFER_ID);
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit results transfer request to OBC." << std::endl;
                    return ObcBridgeState::IDLE;
                }

                startTimer();
                state = TransmitResultState::WAIT_TRANSFER_ACK;
                break;

            case TransmitResultState::WAIT_TRANSFER_ACK:
                if (obc_messager.checkAck(PYLD_REQUEST_TRANSFER_ID) == true)
                    state = TransmitResultState::SEND_HEADER;
                else if (readTime() > ack_timeout)
                {
                    std::cout << "[WARN] No ACK for transfer request, retransmitting." << std::endl;
                    state = TransmitResultState::REQUEST_TRANSFER;
                }
                break;

            case TransmitResultState::SEND_HEADER:
                std::cout << "[INFO] Sending transfer header to OBC." << std::endl;
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
                if (obc_messager.checkAck(PYLD_TRANSFER_HEADER_ID) == true)
                    state = TransmitResultState::SEND_PACKET;
                else if (readTime() > ack_timeout)
                {
                    std::cout << "[WARN] No ACK for transfer header, retransmitting." << std::endl;
                    state = TransmitResultState::SEND_HEADER;
                }
                break;

            case TransmitResultState::SEND_PACKET:
                // std::cout << "[INFO] Sending packet " << (packet_num + 1) << "." << std::endl;
                if (packet_num % 1000 == 0)
                {
                    std::cout << "[INFO] Sending packet " << (packet_num + 1) << "." << std::endl;
                }

                transmit_success = obc_messager.transmitResultsPacket();
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit packet " << (packet_num + 1)
                              << " to OBC, retrying after timeout." << std::endl;
                }

                startTimer();
                state = TransmitResultState::WAIT_PACKET_ACK;
                break;

            case TransmitResultState::WAIT_PACKET_ACK:
                if (obc_messager.checkAck(PYLD_PACKET_ID) == true)
                {
                    packet_num++;
                    if (!obc_messager.isTransmitQueueEmpty())
                        state = TransmitResultState::SEND_PACKET;
                    else
                        state = TransmitResultState::TRANSFER_COMPLETE;
                }
                else if (readTime() > ack_timeout)
                {
                    std::cout << "[WARN] No ACK for packet " << packet_num + 1
                              << ", retransmitting." << std::endl;
                    obc_messager.transmitLastResultsPacket();
                    startTimer();
                }
                break;

            case TransmitResultState::TRANSFER_COMPLETE:
                std::cout << "[INFO] All " << packet_num << " packets sent. Sending transfer complete." << std::endl;
                transmit_success = obc_messager.transmit(PYLD_TRANSFER_COMPLETE_ID);
                if (!transmit_success)
                {
                    std::cout << "[ERROR] Failed to transmit transfer complete signal to OBC." << std::endl;
                }

                startTimer();
                state = TransmitResultState::TRANSFER_COMPLETE_ACK;
                break;

            case TransmitResultState::TRANSFER_COMPLETE_ACK:
                if (obc_messager.checkAck(PYLD_TRANSFER_COMPLETE_ID) == true)
                {
                    std::cout << "[INFO] Transfer complete, ACK received." << std::endl;
                    return ObcBridgeState::IDLE;
                }
                else if (readTime() > ack_timeout)
                {
                    std::cout << "[WARN] No ACK for transfer complete, retransmitting." << std::endl;
                    obc_messager.transmit(PYLD_TRANSFER_COMPLETE_ID);
                    startTimer();
                }
                break;
        }
    }

    return ObcBridgeState::IDLE; // should never reach this line
}

ObcBridgeState ObcBridge::handleTransmitErrorState()
{
    // TODO: currently does nothing with errors
    return ObcBridgeState::IDLE;
}

// (modelled off of do experiment state)
ObcBridgeState ObcBridge::handleDebugState()
{
    int run_result_id;

    // Start the debug mode
    publishRunCommand(Commands::RunId::RUN_DEBUG);

    // Poll for stop run messages from the OBC and run complete messages from
    // the payload controller
    while (true)
    {
        // Drain all available UART messages into the receive queue once per cycle
        obc_messager.drainUart();

        if (lcm_handler.checkRunResult(run_result_id) == true)
        {
            if (run_result_id == Commands::RunResult::RUN_SUCCESS)
                return ObcBridgeState::TRANSMIT_DEBUG_RESULTS;

            if (run_result_id == Commands::RunResult::RUN_FAIL)
                return ObcBridgeState::TRANSMIT_EXPERIMENT_ERROR;
        }

        if (obc_messager.checkMessage(PYLD_STOP_ID))
        {
            publishRunCommand(Commands::RunId::STOP_CONTROLLER);
            return ObcBridgeState::IDLE;
        }
    }

    return ObcBridgeState::DEBUG; // should never reach this line    
}


// --- Timer -------------------------------------------------------------------

void ObcBridge::startTimer()
{
    timer_start = std::chrono::steady_clock::now();
}

// Returns the time since startTimer() was called in milliseconds
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

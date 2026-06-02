#include "obcMessageHandler.hpp"

#include <iostream>

ObcMessageHandler::ObcMessageHandler() 
    : uart_interface(), last_message_read(), message_is_stored(false)
{
    if (uart_interface.setupUart() == false)
    {
        std::cout << "[ERROR] Could not setup UART port." << std::endl;
        return;
    }
    std::cout << "[INFO] UART port config set." << std::endl;
};

ObcMessageHandler::~ObcMessageHandler() {};

// --- Helper message constructors and readers ---------------------------------

bool ObcMessageHandler::transmitIdOnlyMessage(uint8_t id)
{
    // Populate message with length one where the payload is the message ID
    UART_msg_t msg;
    msg.sof        = UART_SOF;
    msg.id         = id;
    msg.length     = 1;
    msg.payload[0] = id;
    // TODO: CRC automatically populated? 
    
    return uart_interface.transmit(&msg); 
}

// Check for any message from the UART port 
bool ObcMessageHandler::getMessage()
{
    UART_msg_t temp_msg;
    bool msg_received = uart_interface.receive(&temp_msg, DEFAULT_UART_TIMEOUT_US);

    // Only update the internal state if a message was received
    if (msg_received == true)
    {
        last_message_read = temp_msg;
        message_is_stored = true; 
    }

    return msg_received; 
}

bool ObcMessageHandler::checkForMessage(uint8_t id)
{
    // First check if the message is already stored
    if (message_is_stored && last_message_read.id == id)
        return true; 

    // Check if one is waiting at the UART port
    if (getMessage() == true && last_message_read.id == id)
        return true;

    return false; 
}

// --- Experiment start / stop messaging ---------------------------------------

bool ObcMessageHandler::transmitStartAck()
{
    return transmitIdOnlyMessage(PYLD_START_ACK_ID);
}

bool ObcMessageHandler::transmitStopAck()
{
    return transmitIdOnlyMessage(PYLD_STOP_ACK_ID);
}

bool ObcMessageHandler::checkStartMsg()
{
    return checkForMessage(PYLD_START_ID);
}

bool ObcMessageHandler::checkStopMsg()
{
    return checkForMessage(PYLD_STOP_ID);
}

// --- Result transfer messages ------------------------------------------------

bool ObcMessageHandler::transmitTransferRequest()
{
    return transmitIdOnlyMessage(PYLD_REQUEST_TRANSFER_ID);
}

bool ObcMessageHandler::transmitTransferComplete()
{
    return transmitIdOnlyMessage(PYLD_TRANSFER_COMPLETE_ID);
}
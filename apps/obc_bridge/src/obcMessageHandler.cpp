#include "obcMessageHandler.hpp"

#include <iostream>

ObcMessageHandler::ObcMessageHandler() 
    : uart_interface()
{
    if (uart_interface.setupUart() == false)
    {
        std::cout << "[ERROR] Could not setup UART port." << std::endl;
        return;
    }
    std::cout << "[INFO] UART port config set." << std::endl;
};

ObcMessageHandler::~ObcMessageHandler() {};

// --- Helper message constructors ---------------------------------------------

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

// --- Experiment start / stop messaging ---------------------------------------

bool ObcMessageHandler::transmitStartAck()
{
    return transmitIdOnlyMessage(PYLD_START_ACK_ID);
}

bool ObcMessageHandler::transmitStopAck()
{
    return transmitIdOnlyMessage(PYLD_STOP_ACK_ID);
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
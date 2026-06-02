/*
Constructs and decodes messages of the UART_msg_t format specified in 
uartInterface.hpp. 
*/

#ifndef OBC_MESSAGE_HANDLER_H
#define OBC_MESSAGE_HANDLER_H

#include "uartInterface.hpp"

#include <cstdint>

// --- Message IDs -------------------------------------------------------------
// Experiment start / stop 
#define PYLD_START_ID       0xA0
#define PYLD_START_ACK_ID   0xA1
#define PYLD_STOP_ID        0xA2
#define PYLD_STOP_ACK_ID    0xA3

// Results transfer
#define PYLD_REQUEST_TRANSFER_ID    0xA4
#define PYLD_TRANSFER_ACK_ID        0xA5
#define PYLD_TRANSFER_HEADER_ID     0xA6
#define PYLD_HEADER_ACK_ID          0xA7
#define PYLD_PACKET_ID              0xA8
#define PYLD_PACKET_ACK_ID          0xA9
#define PYLD_TRANSFER_COMPLETE_ID   0xAA

#define ACK_TIMEOUT 1000 // ms, time before a new message is sent because an acknowledge was not received

class ObcMessageHandler
{
    private: 
        // For reading and writing messages to the UART port
        UartInterface uart_interface;

        UART_msg_t last_message_read;
        bool message_is_stored; 

        // Formats and transmits UART messages with a single payload byte equal
        // to the message ID. 
        bool transmitIdOnlyMessage(uint8_t id);
        
        // Check for a message sent over UART and store in last_message_read.
        // Flag when a message has been received in last_message_received.
        // Overwrites any message already stored in last_message_read. 
        bool getMessage();

        // Checks if a message with the given id is stored in last_message_read 
        // or if it can be read from the UART terminal 
        bool checkForMessage(uint8_t id);

    public: 
        ObcMessageHandler();
        ~ObcMessageHandler();

        // Experiment start / stop messages
        bool transmitStartAck();
        bool transmitStopAck();
        bool checkStartMsg();
        bool checkStopMsg();

        // Result transfer messages
        bool transmitTransferRequest();
        bool transmitHeader();
        bool transmitResultsPacket();
        bool transmitTransferComplete();
        bool checkTransferAck();
        bool checkHeaderAck();
        bool checkPacketAck();
};

#endif 
/*
Entry point for the OBC bridge executable. Handles high level logic of processing
commands from the teensy, communicating with the payload controller and preparing
to send data back over UART. 
*/

#ifndef OBC_BRIDGE_H
#define OBC_BRDIGE_H

#include "uartInterface.hpp"

class ObcBridge
{
    private: 
        // For reading and writing messages to the UART port
        UartInterface uart_interface;

    public: 
        ObcBridge();
        ~ObcBridge();
};

#endif
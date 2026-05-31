/*
Handles UART hardware interface. 
*/

#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include <string>
#include <cstdint>

class UartInterface 
{
    private:
    
    public:
        // Opens the serial port file corresponding to UART & saves the existing 
        // settings to restore in the constructor.
        UartInterface(const std::string& port, int baud_rate);

        // Closes the serial port and restores its original settings.
        ~UartInterface();
};

#endif
#include "obcBridge.hpp"

#include <iostream>

ObcBridge::ObcBridge()
    : uart_interface()
{
    if (uart_interface.setupUart() == false)
    {
        std::cout << "[ERROR] Could not setup UART port." << std::endl;
        return;
    }
    std::cout << "[INFO] UART port config set." << std::endl;
}

ObcBridge::~ObcBridge() {};
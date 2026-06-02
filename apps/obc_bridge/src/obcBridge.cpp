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
            case ObcBridgeState::DO_EXPERIMENT:
                state = handleDoExperimentState();
                break; 
            case ObcBridgeState::TRANSMIT_RESULT:
                state = handleTransmitResultState();
                break; 
            case ObcBridgeState::TRANSMIT_ERROR:
                state = handleTransmitErrorState();
                break;
        }
    }
}

// --- Main state machine logic ------------------------------------------------
ObcBridgeState ObcBridge::handleIdleState()
{
    return ObcBridgeState::DO_EXPERIMENT;
}

ObcBridgeState ObcBridge::handleDoExperimentState()
{
    return ObcBridgeState::TRANSMIT_RESULT;
}

ObcBridgeState ObcBridge::handleTransmitResultState()
{
    return ObcBridgeState::IDLE;
}

ObcBridgeState ObcBridge::handleTransmitErrorState()
{
    return ObcBridgeState::IDLE;
}
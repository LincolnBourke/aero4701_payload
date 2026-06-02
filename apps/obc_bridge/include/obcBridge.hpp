/*
Entry point for the OBC bridge executable. Handles high level logic of processing
commands from the teensy, communicating with the payload controller and preparing
to send data back over UART. 
*/

#ifndef OBC_BRIDGE_H
#define OBC_BRDIGE_H

// States for the top level OBC-payload comms state machine
enum class ObcBridgeState 
{
    IDLE, 
    DO_EXPERIMENT,
    TRANSMIT_RESULT,
    TRANSMIT_ERROR
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
    TRANSFER_COMPLETE
};

enum class TransmitErrorState
{
    REQUEST_TRANSFER,
    WAIT_TRANSFER_ACK
}; // TODO: finish specifying states

class ObcBridge
{
    private: 
        // State handlers for the main OBC-payload comms state machine
        ObcBridgeState handleIdleState();
        ObcBridgeState handleDoExperimentState();
        ObcBridgeState handleTransmitResultState();
        ObcBridgeState handleTransmitErrorState();

        // State handlers for the transmit results state machine
        // TODO

        // State handlers for the transmit error state machine
        // TODO

        // File i/o
        // TODO

    public: 
        // Setup UART port 
        ObcBridge();
        ~ObcBridge();

        // Run the main event loop
        void run();
};

#endif
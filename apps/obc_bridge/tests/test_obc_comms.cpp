#include "obcBridge.hpp"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

// Subclass that exposes the protected ObcBridge state handlers as public entry points.
// This exercises the real state machine code rather than a re-implementation.
class ObcBridgeTest : public ObcBridge
{
    public:
        // Tests the outbound path: payload serialises results.csv and sends to OBC.
        // Mirrors the TRANSMIT_EXPERIMENT_RESULTS state in the main ObcBridge state machine.
        void testSendResults()
        {
            std::cout << "[TEST] Starting send-results test." << std::endl;
            handleTransmitResultState();
            std::cout << "[TEST] send-results test complete." << std::endl;
        }

        // Tests the outbound debug path: payload serialises the debug JPEG and sends to OBC.
        // Mirrors the TRANSMIT_DEBUG_RESULTS state in the main ObcBridge state machine.
        void testSendDebugResults()
        {
            std::cout << "[TEST] Starting send-debug-results test." << std::endl;
            handleTransmitDebugResultsState();
            std::cout << "[TEST] send-debug-results test complete." << std::endl;
        }

        // Tests the full debug round-trip: payload waits for the OBC to enter debug mode,
        // simulates debug work by sleeping, then transmits the debug JPEG back to the OBC.
        void testRoundLoopDebug(int wait_seconds)
        {
            std::cout << "[TEST] Starting debug round-loop test." << std::endl;

            // Wait for ENTER_DEBUG from OBC and send DEBUG_ACK, mirroring handleIdleState()
            std::cout << "[TEST] Waiting for enter-debug message from OBC..." << std::endl;
            while (true)
            {
                obc_messager.drainUart();
                if (obc_messager.checkMessage(PYLD_ENTER_DEBUG_ID))
                {
                    obc_messager.transmit(PYLD_DEBUG_ACK_ID);
                    break;
                }
            }

            // Simulate debug mode work
            std::cout << "[TEST] Debug mode entered. Simulating work for " << wait_seconds << " seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(wait_seconds));

            // Transmit debug results (JPEG) back to OBC
            std::cout << "[TEST] Transmitting debug results to OBC..." << std::endl;
            handleTransmitDebugResultsState();

            std::cout << "[TEST] Debug round-loop test complete." << std::endl;
        }

        // Tests both directions end-to-end: payload receives settings from OBC,
        // waits, then sends results back to OBC.
        void testRoundLoop(int wait_seconds)
        {
            std::cout << "[TEST] Starting round-loop test." << std::endl;

            // Phase 1: receive experiment settings from OBC.
            // Replicate the handleIdleState() handshake for REQUEST_TRANSFER before
            // entering handleReceiveSettingsState(), which starts at WAIT_HEADER.
            std::cout << "[TEST] Phase 1: waiting for transfer request from OBC..." << std::endl;
            while (true)
            {
                obc_messager.drainUart();
                if (obc_messager.checkMessage(PYLD_REQUEST_TRANSFER_ID))
                {
                    obc_messager.transmit(PYLD_TRANSFER_ACK_ID);
                    break;
                }
            }
            std::cout << "[TEST] Transfer request received, entering receive settings state..." << std::endl;
            handleReceiveSettingsState();

            // Wait before sending results
            std::cout << "[TEST] Phase 1 complete. Waiting " << wait_seconds << " seconds..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(wait_seconds));

            // Phase 2: send results back to OBC
            std::cout << "[TEST] Phase 2: sending results to OBC..." << std::endl;
            handleTransmitResultState();

            std::cout << "[TEST] Round-loop test complete." << std::endl;
        }
};

static void printUsage(const char* prog)
{
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << prog << " send-results" << std::endl;
    std::cout << "  " << prog << " roundloop [wait_seconds]" << std::endl;
    std::cout << "  " << prog << " send-debug-results" << std::endl;
    std::cout << "  " << prog << " debug-roundloop [wait_seconds]" << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printUsage(argv[0]);
        return 1;
    }

    std::string mode = argv[1];
    ObcBridgeTest test;

    if (mode == "send-results")
    {
        test.testSendResults();
    }
    else if (mode == "roundloop")
    {
        int wait_seconds = (argc >= 3) ? std::stoi(argv[2]) : 5;
        test.testRoundLoop(wait_seconds);
    }
    else if (mode == "send-debug-results")
    {
        test.testSendDebugResults();
    }
    else if (mode == "debug-roundloop")
    {
        int wait_seconds = (argc >= 3) ? std::stoi(argv[2]) : 5;
        test.testRoundLoopDebug(wait_seconds);
    }
    else
    {
        std::cout << "[ERROR] Unknown mode '" << mode << "'." << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    return 0;
}

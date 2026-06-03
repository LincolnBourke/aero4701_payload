#include "obcMessageHandler.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

#define RESULT_TIMESTEPS 150 

ObcMessageHandler::ObcMessageHandler() 
    : uart_interface(), last_message_read(), message_is_stored(false), transmit_queue(),
      last_msg_buffer(), results_header(), receive_queue(), num_expected_msgs(0), msg_counter(0)
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
    {
        message_is_stored = false; 
        return true; 
    }
        

    // Check if one is waiting at the UART port
    if (getMessage() == true && last_message_read.id == id)
    {
        message_is_stored = false;
        return true;
    }    
    
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

bool ObcMessageHandler::transmitHeader()
{   
    return uart_interface.transmit(&results_header);
}

bool ObcMessageHandler::transmitResultsPacket()
{
    // Save message to try again if need be
    last_msg_buffer = transmit_queue.front();

    // Attempt to transmit the first message in the queue
    bool success = uart_interface.transmit(&last_msg_buffer);

    // Always pop, re-try is handled by transmitLastResultsPacket()
    transmit_queue.pop();
    return success;
}

bool ObcMessageHandler::transmitLastResultsPacket()
{
    // Attempt to transmit the last message again 
    return uart_interface.transmit(&last_msg_buffer);
}

bool ObcMessageHandler::transmitTransferComplete()
{
    return transmitIdOnlyMessage(PYLD_TRANSFER_COMPLETE_ID);
}

bool ObcMessageHandler::isTransmitQueueEmpty()
{
    return transmit_queue.empty();
}

bool ObcMessageHandler::checkTransferAck()
{
    return checkForMessage(PYLD_TRANSFER_ACK_ID);
}

bool ObcMessageHandler::checkHeaderAck()
{
    return checkForMessage(PYLD_HEADER_ACK_ID);
}

bool ObcMessageHandler::checkPacketAck()
{
    return checkForMessage(PYLD_PACKET_ACK_ID);
}

bool ObcMessageHandler::checkTransferCompleteAck()
{
    return checkForMessage(PYLD_TRANSFER_COMPLETE_ACK_ID);
}

// --- Experiment settings transfer messages -----------------------------------

bool ObcMessageHandler::transmitTransferAck()
{
    return transmitIdOnlyMessage(PYLD_TRANSFER_ACK_ID);
}

bool ObcMessageHandler::transmitHeaderAck()
{
    return transmitIdOnlyMessage(PYLD_HEADER_ACK_ID);
}

bool ObcMessageHandler::transmitPacketAck()
{
    return transmitIdOnlyMessage(PYLD_PACKET_ACK_ID);
}

bool ObcMessageHandler::checkTransferRequest()
{
    return checkForMessage(PYLD_TRANSFER_ACK_ID);
}

// Reads the header sent by the OBC and sets the number of messages expected to receive
bool ObcMessageHandler::checkHeader()
{
    // First check if the message is already stored
    if (!(message_is_stored && last_message_read.id == PYLD_TRANSFER_HEADER_ID))
        return false; 

    // Check if one is waiting at the UART port
    if (!(getMessage() == true && last_message_read.id == PYLD_TRANSFER_HEADER_ID))
        return false;

    // Read the number of payload messages to follow
    memcpy(&num_expected_msgs, &last_message_read.payload[0], sizeof(uint16_t));
    message_is_stored = false; 

    return true;
}



// --- Debug messages ----------------------------------------------------------

bool ObcMessageHandler::transmitDebugAck()
{
    return transmitIdOnlyMessage(PYLD_DEBUG_ACK_ID);
}

bool ObcMessageHandler::checkEnterDebugMsg()
{
    return checkForMessage(PYLD_ENTER_DEBUG_ID);
}

// --- Message serialising -----------------------------------------------------

// Reads a CSV file and packs into payloads of UART_msg_t, populating the transmit queue
bool ObcMessageHandler::serialiseResults(std::string file_path)
{
    // Open the CSV file and verify it is accessible
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cout << "[ERROR] Could not open file when serialising results." << std::endl;
        return false;
    }

    // Iterate through csv lines and pack as binary data, removing commas
    std::string line;       // line of the csv
    int line_num = 0;       // line number of the csv
    std::stringstream ss;   // line of the csv as a string stream
    std::string raw_entry;  // entry of the csv line
    
    // Prepare the first message struct
    UART_msg_t msg = {}; 
    uint16_t num_msgs = 0;
    std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t)); // First two bytes are the index of the payload message
    msg.length = sizeof(uint16_t);     // Update length as the message is filled
    
    while (std::getline(file, line))
    {   
        ss.clear();
        ss.str(line);

        // Iterate through entries in the row 
        while (std::getline(ss, raw_entry, ','))
        {
            // Get the type of the next entry
            size_t entry_size = line_num < 2 * RESULT_TIMESTEPS ? sizeof(float) : sizeof(int); 

            // If the last message is full add it to the queue and make a new message  
            if (msg.length + entry_size > RX_BUFFER_BYTES)
            {
                // Assume crc populated by uartInterface at time of tranmission
                msg.sof = UART_SOF; 
                msg.id = PYLD_PACKET_ID;
                transmit_queue.push(msg);
                num_msgs++;

                // Make a new message ?
                msg = {}; // zero initialise
                std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t));
                msg.length = sizeof(uint16_t);
            }

            // Add the entry to the latest message
            if (line_num < 2 * RESULT_TIMESTEPS)
            {
                // The line is servo angles or a camera pose estimate, interpret as float
                float value = std::stof(raw_entry);
                std::memcpy(&msg.payload[msg.length], &value, sizeof(float));
                msg.length += sizeof(float);
            }
            else 
            {
                // The line is camera data, interpret as an integer
                uint8_t value = static_cast<uint8_t>(std::stoi(raw_entry));
                std::memcpy(&msg.payload[msg.length], &value, sizeof(uint8_t));
                msg.length += sizeof(uint8_t);
            }
        }

        line_num++;
    }
    file.close();

    // Push the final partial message if it contains more than just the index header
    if (msg.length > sizeof(uint16_t))
    {
        msg.sof = UART_SOF;
        msg.id  = PYLD_PACKET_ID;
        transmit_queue.push(msg);
        num_msgs++;
    }

    // Create a header message based on the number of packets added to the queue
    results_header = {};
    results_header.sof = UART_SOF; 
    results_header.id = PYLD_TRANSFER_HEADER_ID;
    memcpy(&results_header.payload[0], &num_msgs, sizeof(uint16_t));
    results_header.length = sizeof(uint16_t); 

    return true;
}

bool ObcMessageHandler::serialiseDebugResults(std::string file_path)
{
    // Open the CSV file and verify it is accessible
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cout << "[ERROR] Could not open file when serialising results." << std::endl;
        return false;
    }

    // TODO: read debug results file 

    file.close();

    return false; // TODO: update 
}


#include "obcMessageHandler.hpp"
#include "payloadConfig.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>

ObcMessageHandler::ObcMessageHandler()
    : uart_interface(), last_message_read(), receive_queue(), transmit_queue(),
      last_msg_buffer(), results_header(), file_receive_buffer(), num_expected_msgs(0), msg_counter(0)
{
    if (uart_interface.setupUart() == false)
    {
        std::cout << "[ERROR] Could not setup UART port." << std::endl;
        return;
    }
    std::cout << "[INFO] UART port config set." << std::endl;
};

ObcMessageHandler::~ObcMessageHandler() {};

// --- UART intake -------------------------------------------------------------

// Drain all complete messages from the UART port into receive_queue.
// Drops any message that fails CRC validation without sending an ack,
// forcing the OBC to retransmit after its timeout.
void ObcMessageHandler::drainUart()
{
    UART_msg_t msg;
    while (uart_interface.receive(&msg, DEFAULT_UART_TIMEOUT_US))
    {
        // std::cout << "Received UART message." << std::endl;

        if (UART_checkCRC(&msg))
        {
            receive_queue.push_back(msg);
            // std::cout << "CRC Passed." << std::endl;
        }
            
    }
}

// --- General message transmit and check --------------------------------------

// Populate and transmit a message whose only payload byte is the message ID
bool ObcMessageHandler::transmit(uint8_t id)
{
    UART_msg_t msg;
    msg.sof        = UART_SOF;
    msg.id         = id;
    msg.length     = 1;
    msg.payload[0] = id;

    return uart_interface.transmit(&msg);
}

// Transmit a unified ACK; payload[0] identifies the message being acknowledged
bool ObcMessageHandler::transmitAck(uint8_t acked_id)
{
    std::cout << "Transmitting ACK." << std::endl;
    (void)acked_id;

    UART_msg_t msg;
    msg.sof        = UART_SOF;
    msg.id         = PYLD_ACK_ID;
    msg.length     = 1;
    msg.payload[0] = PYLD_ACK_ID;// acked_id;

    return uart_interface.transmit(&msg);
}

// Search the receive queue for a message with the given ID.
// On match, stores the message in last_message_read and removes it from the queue.
bool ObcMessageHandler::checkMessage(uint8_t id)
{
    for (auto it = receive_queue.begin(); it != receive_queue.end(); ++it)
    {
        if (it->id == id)
        {
            last_message_read = *it;
            receive_queue.erase(it);
            return true;
        }
    }
    return false;
}

// Search the receive queue for a unified ACK whose payload[0] matches acked_id.
// On match, stores the message in last_message_read and removes it from the queue.
bool ObcMessageHandler::checkAck(uint8_t acked_id)
{
    (void)acked_id;

    for (auto it = receive_queue.begin(); it != receive_queue.end(); ++it)
    {
        if (it->id == PYLD_ACK_ID && it->length >= 1)
        {
            last_message_read = *it;
            receive_queue.erase(it);
            return true;
        }

        // if (it->id == PYLD_ACK_ID && it->length >= 1 && it->payload[0] == acked_id)
        // {
        //     last_message_read = *it;
        //     receive_queue.erase(it);
        //     return true;
        // }
    }
    return false;
}

// --- File receive messages ---------------------------------------------------

// Search the receive queue for a transfer header message and read the
// expected packet count from its payload
bool ObcMessageHandler::checkHeader()
{
    if (!checkMessage(PYLD_TRANSFER_HEADER_ID))
        return false;

    // Header format: [file_id: 1 byte][chunk_size: 4 bytes][num_chunks: 4 bytes]
    uint8_t  file_id;
    uint32_t chunk_size;
    uint32_t num_chunks;
    memcpy(&file_id,    &last_message_read.payload[0], sizeof(uint8_t));
    memcpy(&chunk_size, &last_message_read.payload[1], sizeof(uint32_t));
    memcpy(&num_chunks, &last_message_read.payload[5], sizeof(uint32_t));
    num_expected_msgs = static_cast<uint16_t>(num_chunks);
    std::cout << "[INFO] Transfer header received: file_id=" << (int)file_id
              << " chunk_size=" << chunk_size << " num_chunks=" << num_chunks << std::endl;
    return true;
}

// Search the receive queue for the next file packet and validate its sequence index.
// Appends to file_receive_buffer on success.
// Returns true if the state machine should send a packet ack to the OBC.
bool ObcMessageHandler::checkPacket()
{
    for (auto it = receive_queue.begin(); it != receive_queue.end(); ++it)
    {
        if (it->id != PYLD_PACKET_ID)
        {
            // std::cout << "Skipping message in queue when searching for packet." << std::endl;
            continue;
        }

        // Extract the sequence index embedded in the first two payload bytes
        uint16_t index;
        memcpy(&index, &it->payload[0], sizeof(uint16_t));

        if (index == msg_counter)
        {
            // Expected next packet - store and advance counter
            file_receive_buffer.push_back(*it);
            receive_queue.erase(it);
            msg_counter++;
            std::cout << "[INFO] Received packet " << msg_counter << " / " << num_expected_msgs << std::endl;
            return true;
        }
        else if (index < msg_counter)
        {
            // OBC retransmit: this packet was already stored, our ack was lost.
            // Remove and return true so the state machine acks again and OBC advances.
            std::cout << "[WARN] Duplicate packet " << index << " (already received), re-ACKing." << std::endl;
            receive_queue.erase(it);
            return true;
        }
        else
        {
            // Unexpected gap in sequence - do not consume or ack.
            // OBC will retransmit the missing packet after its timeout.
            std::cout << "[WARN] Unexpected packet index " << index
                      << ", expected " << msg_counter << std::endl;
            return false;
        }
    }
    return false;
}

// Returns true when all expected file packets have been received
bool ObcMessageHandler::isReceiveComplete()
{
    return msg_counter == num_expected_msgs;
}

// Reset the file receive buffer and packet counter ready for a new transfer
void ObcMessageHandler::clearFileBuffer()
{
    file_receive_buffer.clear();
    msg_counter = 0;
}

// --- Message deserialising ---------------------------------------------------

// Deserialise the file receive buffer into a trajectory CSV and a scalar settings CSV.
// Binary schema (in order): frame_rate, threshold, exposure, NUM_TIMESTEPS x position[3],
// NUM_TIMESTEPS x attitude[3] (degrees), satellite_attitude[3] (degrees).
bool ObcMessageHandler::deserialise(std::string trajectory_path, std::string settings_path)
{
    // Flatten all packet payloads into a single byte stream,
    // skipping the 2-byte packet index prefix at the start of each payload
    std::vector<uint8_t> stream;
    for (const auto& pkt : file_receive_buffer)
    {
        for (int i = sizeof(uint16_t); i < pkt.length; i++)
            stream.push_back(pkt.payload[i]);
    }

    // Lambda to read one float from the stream at the current offset
    size_t offset = 0;
    auto read_float = [&](float& out) -> bool
    {
        if (offset + sizeof(float) > stream.size())
            return false;
        memcpy(&out, &stream[offset], sizeof(float));
        offset += sizeof(float);
        return true;
    };

    // Parse scalar settings
    float frame_rate, threshold, exposure;
    if (!read_float(frame_rate) || !read_float(threshold) || !read_float(exposure))
    {
        std::cout << "[ERROR] deserialise: stream too short for scalar settings." << std::endl;
        return false;
    }

    // Parse NUM_TIMESTEPS platform positions and attitudes (separate blocks in the stream)
    float positions[NUM_TIMESTEPS][3];
    float attitudes[NUM_TIMESTEPS][3];

    for (int i = 0; i < NUM_TIMESTEPS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (!read_float(positions[i][j]))
            {
                std::cout << "[ERROR] deserialise: stream too short for position data." << std::endl;
                return false;
            }
        }
    }

    for (int i = 0; i < NUM_TIMESTEPS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (!read_float(attitudes[i][j]))
            {
                std::cout << "[ERROR] deserialise: stream too short for attitude data." << std::endl;
                return false;
            }
        }
    }

    // Parse satellite attitude
    float sat_attitude[3];
    for (int j = 0; j < 3; j++)
    {
        if (!read_float(sat_attitude[j]))
        {
            std::cout << "[ERROR] deserialise: stream too short for satellite attitude." << std::endl;
            return false;
        }
    }

    // Write trajectory CSV: zip position[i] and attitude[i] onto each row
    std::ofstream traj_file(trajectory_path);
    traj_file << std::fixed << std::setprecision(6);
    if (!traj_file.is_open())
    {
        std::cout << "[ERROR] deserialise: could not open trajectory file for writing." << std::endl;
        return false;
    }

    for (int i = 0; i < NUM_TIMESTEPS; i++)
    {
        traj_file << positions[i][0] << "," << positions[i][1] << "," << positions[i][2] << ","
                  << attitudes[i][0] << "," << attitudes[i][1] << "," << attitudes[i][2] << "\n";
    }
    traj_file.close();

    // Write scalar settings CSV: frame_rate, threshold, exposure, satellite attitude
    std::ofstream settings_file(settings_path);
    settings_file << std::fixed << std::setprecision(6);
    if (!settings_file.is_open())
    {
        std::cout << "[ERROR] deserialise: could not open settings file for writing." << std::endl;
        return false;
    }

    settings_file << frame_rate << "\n"
                  << threshold  << "\n"
                  << exposure   << "\n"
                  << sat_attitude[0] << "," << sat_attitude[1] << "," << sat_attitude[2] << "\n";
    settings_file.close();

    std::cout << "[INFO] Settings deserialised: trajectory -> '" << trajectory_path
              << "', scalars -> '" << settings_path << "'." << std::endl;
    return true;
}

// --- Result transfer messages ------------------------------------------------

// Transmit the results transfer header
bool ObcMessageHandler::transmitHeader()
{
    return uart_interface.transmit(&results_header);
}

// Save and transmit the front of the transmit queue.
// Always pops the front; retries are handled by transmitLastResultsPacket().
bool ObcMessageHandler::transmitResultsPacket()
{
    last_msg_buffer = transmit_queue.front();

    bool success = uart_interface.transmit(&last_msg_buffer);

    // Always pop, re-try is handled by transmitLastResultsPacket()
    transmit_queue.pop();
    return success;
}

// Retransmit the last results packet
bool ObcMessageHandler::transmitLastResultsPacket()
{
    return uart_interface.transmit(&last_msg_buffer);
}

// Returns true when the transmit queue is empty
bool ObcMessageHandler::isTransmitQueueEmpty()
{
    return transmit_queue.empty();
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
    uint32_t max_chunk_size = 0;
    std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t)); // First two bytes are the packet index
    msg.length = sizeof(uint16_t);     // Update length as the message is filled

    while (std::getline(file, line))
    {
        ss.clear();
        ss.str(line);

        if (line_num < 3 * NUM_TIMESTEPS)
        {
            // Float rows: servo angles (0-149), camera position (150-299), camera attitude (300-449)
            while (std::getline(ss, raw_entry, ','))
            {
                // If the message is full, push it and start a new one
                if (msg.length + sizeof(float) > RX_BUFFER_BYTES)
                {
                    msg.sof = UART_SOF;
                    msg.id  = PYLD_PACKET_ID;
                    if (msg.length > max_chunk_size)
                    {
                        max_chunk_size = msg.length;
                    }
                    transmit_queue.push(msg);
                    num_msgs++;

                    msg = {};
                    std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t));
                    msg.length = sizeof(uint16_t);
                }

                float value = std::stof(raw_entry);
                std::memcpy(&msg.payload[msg.length], &value, sizeof(float));
                msg.length += sizeof(float);
            }
        }
        else
        {
            // Histogram rows (450-599): single hex string, 2 chars per packed byte.
            // Decode each pair of hex characters into a uint8_t and pack directly.
            std::getline(ss, raw_entry);
            for (size_t j = 0; j + 1 < raw_entry.size(); j += 2)
            {
                // If the message is full, push it and start a new one
                if (msg.length + sizeof(uint8_t) > RX_BUFFER_BYTES)
                {
                    msg.sof = UART_SOF;
                    msg.id  = PYLD_PACKET_ID;
                    if (msg.length > max_chunk_size)
                    {
                        max_chunk_size = msg.length;
                    }
                    transmit_queue.push(msg);
                    num_msgs++;

                    msg = {};
                    std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t));
                    msg.length = sizeof(uint16_t);
                }

                uint8_t value = static_cast<uint8_t>(std::stoul(raw_entry.substr(j, 2), nullptr, 16));
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
        if (msg.length > max_chunk_size)
        {
            max_chunk_size = msg.length;
        }
        transmit_queue.push(msg);
        num_msgs++;
    }

    // Create a header message based on the number of packets added to the queue
    results_header = {};
    results_header.sof = UART_SOF;
    results_header.id = PYLD_TRANSFER_HEADER_ID;
    uint8_t  file_id    = 0x05;
    uint32_t num_chunks = static_cast<uint32_t>(num_msgs);
    memcpy(&results_header.payload[0], &file_id,        sizeof(uint8_t));
    memcpy(&results_header.payload[1], &max_chunk_size, sizeof(uint32_t));
    memcpy(&results_header.payload[5], &num_chunks,     sizeof(uint32_t));
    results_header.length = sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint32_t);

    std::cout << "[INFO] Serialised " << num_msgs << " packets (max_chunk_size=" << max_chunk_size
              << ") from '" << file_path << "'." << std::endl;
    return true;
}

bool ObcMessageHandler::serialiseDebugResults(std::string file_path)
{
    // Open the JPEG file in binary mode and verify it is accessible
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open())
    {
        std::cout << "[ERROR] Could not open debug results file when serialising." << std::endl;
        return false;
    }

    // Read all raw bytes into a buffer
    std::vector<uint8_t> jpeg_bytes(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>()
    );
    file.close();

    // Pack bytes into UART messages with a 2-byte packet index prefix, same as serialiseResults()
    UART_msg_t msg = {};
    uint16_t num_msgs = 0;
    uint32_t max_chunk_size = 0;
    std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t));
    msg.length = sizeof(uint16_t);

    for (uint8_t byte : jpeg_bytes)
    {
        // If the message is full, push it and start a new one
        if (msg.length + sizeof(uint8_t) > RX_BUFFER_BYTES)
        {
            msg.sof = UART_SOF;
            msg.id  = PYLD_PACKET_ID;
            if (msg.length > max_chunk_size)
            {
                max_chunk_size = msg.length;
            }
            transmit_queue.push(msg);
            num_msgs++;

            msg = {};
            std::memcpy(&msg.payload[0], &num_msgs, sizeof(uint16_t));
            msg.length = sizeof(uint16_t);
        }

        msg.payload[msg.length] = byte;
        msg.length += sizeof(uint8_t);
    }

    // Push the final partial message if it contains more than just the index header
    if (msg.length > sizeof(uint16_t))
    {
        msg.sof = UART_SOF;
        msg.id  = PYLD_PACKET_ID;
        if (msg.length > max_chunk_size)
        {
            max_chunk_size = msg.length;
        }
        transmit_queue.push(msg);
        num_msgs++;
    }

    // Create a header message based on the number of packets added to the queue
    results_header = {};
    results_header.sof = UART_SOF;
    results_header.id  = PYLD_TRANSFER_HEADER_ID;
    uint8_t  file_id    = 1;
    uint32_t num_chunks = static_cast<uint32_t>(num_msgs);
    memcpy(&results_header.payload[0], &file_id,        sizeof(uint8_t));  // File ID
    memcpy(&results_header.payload[1], &max_chunk_size, sizeof(uint32_t)); // Maximum chunk size
    memcpy(&results_header.payload[5], &num_chunks,     sizeof(uint32_t)); // Number of chunks
    results_header.length = sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint32_t);

    // File info message payload format:
    // payload[0] = file id (1 byte long)
    // payload[1] = chunk size (max size of a message payload) (4 bytes)
    // payload[5] = number of chunks (number of messages coming) (4 bytes)

    std::cout << "[INFO] Serialised " << jpeg_bytes.size() << " bytes from '"
              << file_path << "' into " << num_msgs << " packets." << std::endl;
    return true;
}

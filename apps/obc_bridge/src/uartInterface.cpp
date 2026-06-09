#include "uartInterface.hpp"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstring>

#define UART_FILE "/dev/pts/7"      // change for testing depending on socat virtual port
// #define UART_FILE "/dev/serial0"    // Should point to the active UART
// #define UART_FILE "/dev/ttyAMA0"    // GPIO header (pin 14 tx, pin 15 rx)
#define BAUD_RATE B115200

UartInterface::UartInterface() {};

// Automatically clean up 
UartInterface::~UartInterface()
{
    if (uart_filestream != -1)
    {
        // Re-apply initial config 
        tcflush(uart_filestream, TCIFLUSH);
        tcsetattr(uart_filestream, TCSANOW, &initial_config);

        close(uart_filestream);
    }
}

bool UartInterface::setupUart()
{
    // Open UART filestream in non-blocking read/write mode
	uart_filestream = open(UART_FILE, O_RDWR | O_NOCTTY | O_NDELAY);
	if (uart_filestream == -1)
	{
		printf("[Error] Unable to open UART.\n");
        return false; 
	}

    // Get current filestream setup to save and reapply in destructor
    tcgetattr(uart_filestream, &initial_config);

    // Setup config for reading from UART 
    struct termios new_config;
	new_config.c_cflag = BAUD_RATE | CS8 | CLOCAL | CREAD;
	new_config.c_iflag = IGNPAR; // Ignore characters with parity errors
	new_config.c_oflag = 0;
	new_config.c_lflag = 0;

    // Set config
	tcflush(uart_filestream, TCIFLUSH);
	tcsetattr(uart_filestream, TCSANOW, &new_config);

    return true; 
};

bool UartInterface::receive(UART_msg_t* msg, uint32_t timeout_us)
{
    // Check the file stream is open 
    if (uart_filestream == -1)
    {
        return false; 
    }

    // Read buffering 
    unsigned char rx_buffer[1];
    ssize_t bytes_read; 

    // State machine
    UART_rx_state_t state = UART_RX_WAIT_SOF;
    uint8_t byte = 0x00; // must not equal UART_SOF
    uint8_t idx = 0;
    uint8_t crc_idx = 0;

    // Keep a microsecond counter of time passed since the last byte was received for timeout check
    std::chrono::time_point<std::chrono::steady_clock> last_byte_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff;
    double time_since_last_byte = 0;

    while (true)
    {
        // Check that the time since the last byte was recieved does not exceed the timeout 
        time_diff = std::chrono::steady_clock::now() - last_byte_time;
        time_since_last_byte = std::chrono::duration<double, std::micro>(time_diff).count();
        if (time_since_last_byte > timeout_us)
        {   
            break; 
        }

        // Attempt to read one byte from the UART port
        bytes_read = read(uart_filestream, (void*)rx_buffer, 1);
        if (bytes_read == -1 || bytes_read == 0)
        {
            // No bytes to read: loop and poll again
            continue; 
        }

        // Reset inter-byte timeout when a byte is read
        last_byte_time = std::chrono::steady_clock::now();
        byte = rx_buffer[0];

        switch (state)
        {
            case UART_RX_WAIT_SOF:
            {
                if (byte == UART_SOF)
                {
                    memset(msg, 0, sizeof(UART_msg_t));
                    msg->sof = byte;
                    idx = 0;
                    crc_idx = 0;
                    state = UART_RX_GET_ID;
                }
                break;
            }

            case UART_RX_GET_ID:
            {
                msg->id = byte;
                state = UART_RX_GET_LENGTH;
                break;
            }

            case UART_RX_GET_LENGTH:
            {
                msg->length = byte;

                if (msg->length == 0 || msg->length > RX_BUFFER_BYTES)
                {
                    return false;
                }

                state = UART_RX_READ_PAYLOAD;
                break;
            }

            case UART_RX_READ_PAYLOAD:
            {
                msg->payload[idx++] = byte;

                if (idx >= msg->length)
                {
                    state = UART_RX_READ_CRC;
                }
                break;
            }

            case UART_RX_READ_CRC:
            {
                msg->crc |= ((uint16_t)byte << (8 * crc_idx));
                crc_idx++;

                if (crc_idx == RX_CRC_BYTES)
                {
                    if (UART_checkCRC(msg))
                    {
                        std::cout << "[INFO] UART rx: id=0x" << std::hex << std::uppercase
                                  << (int)msg->id << std::dec << " len=" << (int)msg->length << std::endl;
                        return true;
                    }
                    std::cout << "[WARN] UART rx: CRC mismatch on id=0x" << std::hex << std::uppercase
                              << (int)msg->id << std::dec << ", dropping." << std::endl;
                }
                break;
            }

            default:
            {
                state = UART_RX_WAIT_SOF;
                break;
            }
        }
    }

    return false;
}

bool UartInterface::transmit(UART_msg_t* msg)
{
    // Check the file stream is open 
    if (uart_filestream == -1)
    {
        return false;
    }

    // Check if the message length is longer than the maximum
    if (msg->length > RX_BUFFER_BYTES)
    {
        std::cout << "[ERROR] Message length exceeds maximum allowable." << std::endl;
        return false;
    }
    
    // Serialise the message 
    uint8_t data[RX_HEADER_BYTES + RX_BUFFER_BYTES + RX_CRC_BYTES];
    msg->sof = UART_SOF;
    data[0] = msg->sof;
    data[1] = msg->id;
    data[2] = msg->length;
    memcpy(&data[RX_HEADER_BYTES], msg->payload, msg->length);
    msg->crc = UART_crc16_ccitt(data, msg->length + RX_HEADER_BYTES);
    // uint8_t overflows for full-length packets (3 + 254 = 257 > 255), corrupting data[1] and data[2]
    uint16_t crc_offset = RX_HEADER_BYTES + msg->length;
    data[crc_offset]     = msg->crc & 0xFF;
    data[crc_offset + 1] = msg->crc >> 8;

    // Write the data to the port
    long unsigned int expected = msg->length + RX_HEADER_BYTES + RX_CRC_BYTES;
    long unsigned int bytes_transmitted = write(uart_filestream, &data, expected);
    if (bytes_transmitted < expected)
    {
        std::cout << "[ERROR] Not all bytes from buffer written to UART port." << std::endl;
        return false;
    }

    std::cout << "[INFO] UART tx: id=0x" << std::hex << std::uppercase
              << (int)msg->id << std::dec << " len=" << (int)msg->length << std::endl;
    return true;
}

bool UART_checkCRC(UART_msg_t* msg)
{
    uint16_t crc;
    uint8_t data[RX_HEADER_BYTES + RX_BUFFER_BYTES];
    data[0] = msg->sof;
    data[1] = msg->id;
    data[2] = msg->length;
    memcpy(&data[3], msg->payload, msg->length);
    crc = UART_crc16_ccitt(data, msg->length + RX_HEADER_BYTES);
    return (crc == msg->crc);
}

uint16_t UART_crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

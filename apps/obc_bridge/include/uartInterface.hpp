/*
Handles UART hardware interface. Transmits and receives messages of the type
UART_msg_t. 
*/

#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include <string>
#include <cstdint>

// For UART
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

//--------------Defines----------------
#define RX_CRC_BYTES      2
#define RX_HEADER_BYTES   3
#define RX_BUFFER_BYTES 254 // Had to decrease max length from 256 to 254 to make compatible with UART_msg_t maximum length comparisons in .cpp file
#define UART_SOF       0x64
#define DEFAULT_UART_TIMEOUT_US 100

// States for UART message reception state machine
typedef enum{
    UART_RX_WAIT_SOF,
    UART_RX_GET_ID,
    UART_RX_GET_LENGTH,
    UART_RX_READ_PAYLOAD,
    UART_RX_READ_CRC,
    UART_RX_CHECK_CRC
}UART_rx_state_t;

// Define a message transmitted over UART
typedef struct{
    uint8_t id;
    uint8_t sof;
    uint8_t length; 
    uint8_t payload[RX_BUFFER_BYTES];
    uint16_t crc;
}UART_msg_t;

//--------------Function Prototypes----------------
uint16_t UART_crc16_ccitt(const uint8_t *data, uint16_t length);
bool UART_checkCRC(UART_msg_t* msg);

class UartInterface 
{
    private:
        int uart_filestream;

        // Initial UART filestream config
        struct termios initial_config;    
        
    public:
        // Opens the serial port file corresponding to UART & saves the existing 
        // settings to restore in the constructor.
        UartInterface();

        // Closes the serial port and restores its original settings.
        ~UartInterface();

        // Setup UART filestream for read/write. Return false on failure. 
        bool setupUart();  

        // Look for a mesage to receive and read it. Return value indicates if a
        // message was read successfully. 
        bool receive(UART_msg_t* msg, uint32_t timeout_us);

        // Transmit a message. Return value indicates if the message could be sent.
        bool transmit(UART_msg_t* msg);  
};

#endif
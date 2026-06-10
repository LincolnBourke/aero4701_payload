#!/usr/bin/env python3

import serial
import time
import sys

UART_DEVICE = "/dev/ttyAMA3"   # Change if required
BAUDRATE = 115200

TEST_MESSAGE = b"UART1 Loopback Test\r\n"

try:
    ser = serial.Serial(
        port=UART_DEVICE,
        baudrate=BAUDRATE,
        timeout=2
    )

    # Clear buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print(f"Opened {UART_DEVICE}")

    # Transmit
    print(f"Sending: {TEST_MESSAGE}")
    ser.write(TEST_MESSAGE)
    ser.flush()

    # Give UART time to receive its own data
    time.sleep(0.1)

    # Read expected number of bytes
    received = ser.read(len(TEST_MESSAGE))

    print(f"Received: {received}")

    if received == TEST_MESSAGE:
        print("PASS: UART loopback successful")
        sys.exit(0)
    else:
        print("FAIL: Received data does not match")
        sys.exit(1)

except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(2)
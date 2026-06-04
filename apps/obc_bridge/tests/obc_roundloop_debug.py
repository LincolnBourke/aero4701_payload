#!/usr/bin/env python3
"""
OBC simulator — debug mode round-loop test.

Drives the full debug round-trip from the OBC side:
  1. Send ENTER_DEBUG to the payload and wait for DEBUG_ACK
  2. Wait for the payload to finish simulated debug work
  3. Receive the debug results JPEG from the payload

The payload side is driven by:
  ./test_obc_comms debug-roundloop [wait_seconds]

Usage:
  python3 apps/obc_bridge/tests/obc_roundloop_debug.py <serial_port> [output_jpeg] [wait_seconds]

  serial_port   e.g. /dev/pts/4
  output_jpeg   default: data/test_obc_debug/debug_mode_focus_received.jpeg
  wait_seconds  default: 5 (must match the value passed to test_obc_comms)

Dependencies:
  pip install pyserial
"""

import sys
import time
import serial

from uart_helpers import BAUD_RATE, PYLD_ENTER_DEBUG_ID, PYLD_DEBUG_ACK_ID, build_msg, recv_msg
from obc_receive_jpeg import receive_jpeg


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [output_jpeg] [wait_seconds]")
        sys.exit(1)

    serial_port  = sys.argv[1]
    output_jpeg  = sys.argv[2] if len(sys.argv) > 2 \
        else "data/test_obc_debug/debug_mode_focus_received.jpeg"
    wait_seconds = int(sys.argv[3]) if len(sys.argv) > 3 else 5

    port = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=30)
    try:
        # Phase 1: send ENTER_DEBUG, wait for DEBUG_ACK
        print("[OBC] Phase 1: sending enter-debug command to payload...")
        port.write(build_msg(PYLD_ENTER_DEBUG_ID, bytes([PYLD_ENTER_DEBUG_ID])))
        result = recv_msg(port)
        assert result and result[0] == PYLD_DEBUG_ACK_ID, "Expected DEBUG_ACK"
        print("[OBC] Debug mode acknowledged by payload.")

        # The payload sleeps for wait_seconds before initiating the JPEG transfer.
        print(f"[OBC] Waiting {wait_seconds}s for payload to complete debug work...")
        time.sleep(wait_seconds)

        # Phase 2: receive debug results (JPEG) from payload
        print("[OBC] Phase 2: receiving debug results from payload...")
        receive_jpeg(port, output_jpeg)

        print("[OBC] Debug round-loop test complete.")
    finally:
        port.close()

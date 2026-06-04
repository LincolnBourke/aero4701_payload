#!/usr/bin/env python3
"""
OBC simulator — debug results (JPEG) receive side.

Waits for the payload to initiate a debug results file transfer, drives the
OBC receiver role of the protocol, then writes the received raw bytes directly
to a JPEG file for verification.

Usage:
  python3 apps/obc_bridge/tests/obc_receive_jpeg.py <serial_port> [output_jpeg]

  serial_port  e.g. /dev/pts/4 (the socat end not used by the payload)
  output_jpeg  default: data/test_obc_debug/debug_mode_focus_received.jpeg

Dependencies:
  pip install pyserial
"""

import sys
import struct
import serial

from uart_helpers import (
    PYLD_REQUEST_TRANSFER_ID, PYLD_TRANSFER_ACK_ID,
    PYLD_TRANSFER_HEADER_ID, PYLD_HEADER_ACK_ID,
    PYLD_PACKET_ID, PYLD_PACKET_ACK_ID,
    PYLD_TRANSFER_COMPLETE_ID, PYLD_TRANSFER_COMPLETE_ACK_ID,
    BAUD_RATE,
    build_msg, recv_msg,
)

# Mirror the #defines in obcBridge.cpp
DEBUG_IMAGE_WIDTH  = 640
DEBUG_IMAGE_HEIGHT = 480


# --- Transfer protocol -------------------------------------------------------

def receive_jpeg(port: serial.Serial, output_jpeg_path: str):
    """
    Drive the OBC receiver role for a debug results (JPEG) file transfer,
    mirroring handleTransmitDebugResultsState() on the payload side.

    Collects packet payloads, concatenates the raw bytes, and writes them
    directly to output_jpeg_path for verification.
    """
    # Step 1: wait for REQUEST_TRANSFER, send TRANSFER_ACK
    print("[OBC] Waiting for debug transfer request from payload...")
    result = recv_msg(port)
    assert result and result[0] == PYLD_REQUEST_TRANSFER_ID, "Expected REQUEST_TRANSFER"
    port.write(build_msg(PYLD_TRANSFER_ACK_ID, bytes([PYLD_TRANSFER_ACK_ID])))

    # Step 2: wait for HEADER (num_packets), send HEADER_ACK
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_HEADER_ID, "Expected TRANSFER_HEADER"
    num_packets = struct.unpack('<H', result[1][:2])[0]
    print(f"[OBC] Expecting {num_packets} packets")
    port.write(build_msg(PYLD_HEADER_ACK_ID, bytes([PYLD_HEADER_ACK_ID])))

    # Step 3: receive each packet, ack, collect data bytes (strip 2-byte index prefix)
    raw_chunks = []
    for i in range(num_packets):
        result = recv_msg(port)
        assert result and result[0] == PYLD_PACKET_ID, f"Expected PACKET_ID for packet {i}"
        index = struct.unpack('<H', result[1][:2])[0]
        assert index == i, f"Sequence error: expected {i}, got {index}"
        raw_chunks.append(bytes(result[1][2:]))
        port.write(build_msg(PYLD_PACKET_ACK_ID, bytes([PYLD_PACKET_ACK_ID])))
        print(f"[OBC] Packet {i + 1}/{num_packets} received")

    # Step 4: wait for TRANSFER_COMPLETE, send TRANSFER_COMPLETE_ACK
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_COMPLETE_ID, "Expected TRANSFER_COMPLETE"
    port.write(build_msg(PYLD_TRANSFER_COMPLETE_ACK_ID, bytes([PYLD_TRANSFER_COMPLETE_ACK_ID])))
    print("[OBC] Transfer complete.")

    # Concatenate raw bytes and write to file
    jpeg_bytes = b''.join(raw_chunks)
    print(f"[OBC] Received {len(jpeg_bytes)} bytes total.")

    # Verify JPEG magic bytes (SOI marker: FF D8 FF)
    if jpeg_bytes[:3] == b'\xff\xd8\xff':
        print(f"[OBC] JPEG magic bytes verified (FF D8 FF). Expected image: {DEBUG_IMAGE_WIDTH}x{DEBUG_IMAGE_HEIGHT}.")
    else:
        print(f"[OBC WARN] Unexpected file header: {jpeg_bytes[:4].hex()} — not a valid JPEG.")

    with open(output_jpeg_path, 'wb') as f:
        f.write(jpeg_bytes)

    print(f"[OBC] Written to '{output_jpeg_path}'.")


# --- Entry point -------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [output_jpeg]")
        sys.exit(1)

    serial_port = sys.argv[1]
    output_jpeg = sys.argv[2] if len(sys.argv) > 2 \
        else "data/test_obc_debug/debug_mode_focus_received.jpeg"

    port = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=30)
    try:
        receive_jpeg(port, output_jpeg)
    finally:
        port.close()

#!/usr/bin/env python3
"""
OBC simulator — results receive side.

Waits for the payload to initiate a results file transfer, drives the
OBC receiver role of the protocol, then decodes the received binary stream
back into the results CSV format for verification.

Usage:
  python3 apps/obc_bridge/tests/obc_receive.py <serial_port> [output_csv]

  serial_port  e.g. /dev/pts/4 (the socat end not used by the payload)
  output_csv   default: data/test_obc_nominal/results_received.csv

Dependencies:
  pip install pyserial
"""

import sys
import csv
import struct
import serial

from uart_helpers import (
    PYLD_REQUEST_TRANSFER_ID,
    PYLD_TRANSFER_HEADER_ID,
    PYLD_PACKET_ID,
    PYLD_TRANSFER_COMPLETE_ID,
    BAUD_RATE, RESULT_TIMESTEPS,
    build_ack, recv_msg,
)

# Results file binary schema (must match serialiseResults() in obcMessageHandler.cpp)
NUM_SERVOS          = 6
CAM_W, CAM_H        = 640, 480
BYTES_PER_HISTOGRAM = CAM_W * CAM_H // 8  # 38400


# --- Transfer protocol -------------------------------------------------------

def receive_results(port: serial.Serial, output_csv_path: str):
    """
    Drive the OBC receiver role for a results file transfer,
    mirroring handleTransmitResultState() on the payload side.

    Collects packet payloads, decodes the binary stream, and writes
    the decoded data to output_csv_path for verification.
    """
    # Step 1: wait for REQUEST_TRANSFER, send TRANSFER_ACK
    print("[OBC] Waiting for transfer request from payload...")
    result = recv_msg(port)
    assert result and result[0] == PYLD_REQUEST_TRANSFER_ID, "Expected REQUEST_TRANSFER"
    port.write(build_ack(PYLD_REQUEST_TRANSFER_ID))

    # Step 2: wait for HEADER (file_id, chunk_size, num_packets), send HEADER_ACK
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_HEADER_ID, "Expected TRANSFER_HEADER"
    file_id, chunk_size, num_packets = struct.unpack('<BII', result[1][:9])
    print(f"[OBC] Expecting {num_packets} packets (file_id={file_id}, chunk_size={chunk_size})")
    port.write(build_ack(PYLD_TRANSFER_HEADER_ID))

    # Step 3: receive each packet, ack, collect data bytes (strip 2-byte index prefix)
    raw_chunks = []
    for i in range(num_packets):
        result = recv_msg(port)
        assert result and result[0] == PYLD_PACKET_ID, f"Expected PACKET_ID for packet {i}"
        index = struct.unpack('<H', result[1][:2])[0]
        assert index == i, f"Sequence error: expected {i}, got {index}"
        raw_chunks.append(bytes(result[1][2:]))
        port.write(build_ack(PYLD_PACKET_ID))
        print(f"[OBC] Packet {i + 1}/{num_packets} received")

    # Step 4: wait for TRANSFER_COMPLETE, send TRANSFER_COMPLETE_ACK
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_COMPLETE_ID, "Expected TRANSFER_COMPLETE"
    port.write(build_ack(PYLD_TRANSFER_COMPLETE_ID))
    print("[OBC] Transfer complete.")

    # Flatten packet chunks into a single byte stream
    stream = b''.join(raw_chunks)
    print(f"[OBC] Received {len(stream)} bytes total. Decoding...")

    decode_results(stream, output_csv_path)


# --- Binary decode -----------------------------------------------------------

def decode_results(stream: bytes, output_csv_path: str):
    """
    Decode the binary results stream back to CSV format matching results.csv.

    Binary schema (must match serialiseResults() in obcMessageHandler.cpp):
      150 x 6 floats  — servo angles (rows 0-149)
      150 x 3 floats  — camera position (rows 150-299)
      150 x 3 floats  — camera attitude (rows 300-449)
      150 x 38400 bytes — event histogram as packed bits (rows 450-599, hex strings)
    """
    offset = 0
    rows = []

    def read_float() -> float:
        nonlocal offset
        val = struct.unpack_from('<f', stream, offset)[0]
        offset += 4
        return val

    # Servo angles: 150 rows x 6 floats
    for _ in range(RESULT_TIMESTEPS):
        row = [f"{read_float():.6f}" for _ in range(NUM_SERVOS)]
        rows.append(row)

    # Camera position: 150 rows x 3 floats
    for _ in range(RESULT_TIMESTEPS):
        row = [f"{read_float():.6f}" for _ in range(3)]
        rows.append(row)

    # Camera attitude: 150 rows x 3 floats
    for _ in range(RESULT_TIMESTEPS):
        row = [f"{read_float():.6f}" for _ in range(3)]
        rows.append(row)

    # Histogram: 150 rows x 38400 bytes as hex string
    for _ in range(RESULT_TIMESTEPS):
        chunk = stream[offset : offset + BYTES_PER_HISTOGRAM]
        offset += BYTES_PER_HISTOGRAM
        rows.append([chunk.hex()])

    with open(output_csv_path, 'w', newline='') as f:
        csv.writer(f).writerows(rows)

    print(f"[OBC] Decoded results written to '{output_csv_path}' ({len(rows)} rows)")


# --- Entry point -------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [output_csv]")
        sys.exit(1)

    serial_port = sys.argv[1]
    output_csv  = sys.argv[2] if len(sys.argv) > 2 else "data/test_obc_nominal/results_received.csv"

    port = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=30)
    try:
        receive_results(port, output_csv)
    finally:
        port.close()

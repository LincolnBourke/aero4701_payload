#!/usr/bin/env python3
"""
OBC simulator — settings transmit side.

Reads experiment_settings.csv, packs it into the binary wire format that
ObcMessageHandler::deserialise() expects, then drives the file transfer
protocol as the OBC would when sending settings to the payload.

Setup:
  1. Open a terminal and run:
       socat -d -d pty,raw,echo=0 pty,raw,echo=0
     Note the two /dev/pts/N paths printed (e.g. /dev/pts/3 and /dev/pts/4).

  2. Build the payload binary with UART_DEVICE set to the first path:
       cmake -DUART_DEVICE=/dev/pts/3 ..
       make

  3. Run the payload binary, then run this script with the second path:
       python3 apps/obc_bridge/tests/obc_transmit.py /dev/pts/4

Dependencies:
  pip install pyserial
"""

import sys
import csv
import struct
import serial

from uart_helpers import (
    PYLD_REQUEST_TRANSFER_ID, PYLD_TRANSFER_ACK_ID,
    PYLD_TRANSFER_HEADER_ID, PYLD_HEADER_ACK_ID,
    PYLD_PACKET_ID, PYLD_PACKET_ACK_ID,
    PYLD_TRANSFER_COMPLETE_ID, PYLD_TRANSFER_COMPLETE_ACK_ID,
    BAUD_RATE, MAX_DATA_PER_PACKET, RESULT_TIMESTEPS,
    build_msg, recv_msg,
)


# --- Settings packing --------------------------------------------------------

def load_settings(csv_path: str) -> bytes:
    """
    Read experiment_settings.csv and pack into the binary schema that
    ObcMessageHandler::deserialise() parses, in this exact field order:

      float frame_rate
      float threshold
      float exposure
      float positions[RESULT_TIMESTEPS][3]   (x, y, z)
      float attitudes[RESULT_TIMESTEPS][3]   (roll, pitch, yaw, in degrees)
      float sat_attitude[3]                  (in degrees)

    All floats are little-endian.
    """
    with open(csv_path, newline='') as f:
        rows = list(csv.reader(f))

    # Rows 0-2: one scalar per row
    frame_rate = float(rows[0][0])
    threshold  = float(rows[1][0])
    exposure   = float(rows[2][0])

    # Rows 3 to 3+RESULT_TIMESTEPS-1: platform positions
    positions = []
    for row in rows[3 : 3 + RESULT_TIMESTEPS]:
        positions.append([float(v) for v in row])

    # Rows 3+RESULT_TIMESTEPS to 3+2*RESULT_TIMESTEPS-1: platform attitudes (degrees)
    attitudes = []
    for row in rows[3 + RESULT_TIMESTEPS : 3 + 2 * RESULT_TIMESTEPS]:
        attitudes.append([float(v) for v in row])

    # Final row: satellite attitude (degrees)
    sat_attitude = [float(v) for v in rows[3 + 2 * RESULT_TIMESTEPS]]

    # Pack in schema order (little-endian floats)
    stream  = struct.pack('<f', frame_rate)
    stream += struct.pack('<f', threshold)
    stream += struct.pack('<f', exposure)
    for pos in positions:
        stream += struct.pack('<3f', *pos)
    for att in attitudes:
        stream += struct.pack('<3f', *att)
    stream += struct.pack('<3f', *sat_attitude)

    print(f"[OBC] Packed {len(stream)} bytes from '{csv_path}'")
    return stream


def make_packets(data: bytes) -> list:
    """Split binary data into packet payloads, each prefixed with a 2-byte little-endian index."""
    packets = []
    for i in range(0, len(data), MAX_DATA_PER_PACKET):
        index   = struct.pack('<H', len(packets))
        payload = data[i : i + MAX_DATA_PER_PACKET]
        packets.append(index + payload)
    return packets


# --- Transfer protocol -------------------------------------------------------

def send_settings(port: serial.Serial, packets: list):
    """
    Drive the file transfer protocol as the OBC would when sending settings,
    mirroring handleReceiveSettingsState() on the payload side.
    """
    num = len(packets)
    print(f"[OBC] Starting settings transfer: {num} packets")

    # Step 1: request transfer, wait for TRANSFER_ACK
    port.write(build_msg(PYLD_REQUEST_TRANSFER_ID, bytes([PYLD_REQUEST_TRANSFER_ID])))
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_ACK_ID, "Expected TRANSFER_ACK"

    # Step 2: send header (uint16_t packet count), wait for HEADER_ACK
    port.write(build_msg(PYLD_TRANSFER_HEADER_ID, struct.pack('<H', num)))
    result = recv_msg(port)
    assert result and result[0] == PYLD_HEADER_ACK_ID, "Expected HEADER_ACK"

    # Step 3: send each packet, wait for PACKET_ACK before sending the next
    for i, pkt_payload in enumerate(packets):
        port.write(build_msg(PYLD_PACKET_ID, pkt_payload))
        result = recv_msg(port)
        assert result and result[0] == PYLD_PACKET_ACK_ID, f"Expected PACKET_ACK for packet {i}"
        print(f"[OBC] Packet {i + 1}/{num} acknowledged")

    # Step 4: signal transfer complete, wait for TRANSFER_COMPLETE_ACK
    port.write(build_msg(PYLD_TRANSFER_COMPLETE_ID, bytes([PYLD_TRANSFER_COMPLETE_ID])))
    result = recv_msg(port)
    assert result and result[0] == PYLD_TRANSFER_COMPLETE_ACK_ID, "Expected TRANSFER_COMPLETE_ACK"

    print("[OBC] Settings transfer complete.")


# --- Entry point -------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [csv_path]")
        print(f"  serial_port  e.g. /dev/pts/4 (the socat end not used by the payload)")
        print(f"  csv_path     default: data/test_obc_nominal/experiment_settings.csv")
        sys.exit(1)

    serial_port = sys.argv[1]
    csv_path    = sys.argv[2] if len(sys.argv) > 2 else "data/test_obc_nominal/experiment_settings.csv"

    data    = load_settings(csv_path)
    packets = make_packets(data)

    port = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=5)
    try:
        send_settings(port, packets)
    finally:
        port.close()

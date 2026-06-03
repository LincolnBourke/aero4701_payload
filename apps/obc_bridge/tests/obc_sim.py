#!/usr/bin/env python3
"""
OBC simulator for testing UART communication on Ubuntu via socat virtual serial ports.

Reads experiment_settings.csv, packs it into the binary wire format that
ObcMessageHandler::deserialise() expects, then drives the file transfer
protocol as the OBC would.

Setup:
  1. Open a terminal and run:
       socat -d -d pty,raw,echo=0 pty,raw,echo=0
     Note the two /dev/pts/N paths printed (e.g. /dev/pts/3 and /dev/pts/4).

  2. Build the payload binary with UART_DEVICE set to the first path.

  3. Run the payload binary, then run this script with the second path:
       python3 apps/obc_bridge/tests/obc_sim.py /dev/pts/4

Dependencies:
  pip install pyserial
"""

import sys
import csv
import struct
import serial

# --- Message IDs (mirror obcMessageHandler.hpp) ------------------------------
PYLD_REQUEST_TRANSFER_ID      = 0xA4
PYLD_TRANSFER_ACK_ID          = 0xA5
PYLD_TRANSFER_HEADER_ID       = 0xA6
PYLD_HEADER_ACK_ID            = 0xA7
PYLD_PACKET_ID                = 0xA8
PYLD_PACKET_ACK_ID            = 0xA9
PYLD_TRANSFER_COMPLETE_ID     = 0xAA
PYLD_TRANSFER_COMPLETE_ACK_ID = 0xAB

SOF       = 0x64
BAUD_RATE = 115200

# RX_BUFFER_BYTES is 254; each packet payload starts with a 2-byte index prefix,
# leaving 252 bytes available for actual data per packet.
MAX_DATA_PER_PACKET = 252

# Must match RESULT_TIMESTEPS in obcMessageHandler.cpp
RESULT_TIMESTEPS = 150


# --- CRC and framing ---------------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT matching UART_crc16_ccitt() in uartInterface.cpp."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF


def build_msg(msg_id: int, payload: bytes) -> bytes:
    """Pack SOF | ID | LENGTH | PAYLOAD | CRC16 matching UartInterface::transmit()."""
    header = bytes([SOF, msg_id, len(payload)])
    crc = crc16_ccitt(header + payload)
    return header + payload + struct.pack('<H', crc)


def recv_msg(port: serial.Serial) -> tuple | None:
    """
    Block until a complete, valid UART_msg_t is received.
    Returns (msg_id, payload) or None on timeout or CRC failure.
    """
    # Scan incoming bytes for SOF
    while True:
        b = port.read(1)
        if not b:
            print("[OBC] Timeout waiting for message SOF")
            return None
        if b[0] == SOF:
            break

    # Read ID and length
    hdr = port.read(2)
    if len(hdr) < 2:
        return None
    msg_id, length = hdr[0], hdr[1]

    # Read payload and CRC
    payload   = port.read(length)
    crc_bytes = port.read(2)
    if len(payload) < length or len(crc_bytes) < 2:
        return None

    # Validate CRC
    crc_recv = struct.unpack('<H', crc_bytes)[0]
    crc_calc = crc16_ccitt(bytes([SOF, msg_id, length]) + payload)
    if crc_recv != crc_calc:
        print(f"[OBC WARN] CRC mismatch on id=0x{msg_id:02X}")
        return None

    print(f"[OBC] Received id=0x{msg_id:02X} len={length}")
    return msg_id, payload


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
    Drive the file transfer protocol as the OBC would, mirroring
    handleReceiveSettingsState() on the payload side.
    """
    num = len(packets)
    print(f"[OBC] Starting transfer: {num} packets")

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

    print("[OBC] Transfer complete.")


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

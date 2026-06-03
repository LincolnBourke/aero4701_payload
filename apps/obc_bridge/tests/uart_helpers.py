"""
Shared UART protocol constants and framing helpers for OBC simulator scripts.
Mirrors the message IDs and wire format defined in obcMessageHandler.hpp and uartInterface.cpp.
"""

import struct
import serial

# --- Message IDs (mirror obcMessageHandler.hpp) ------------------------------
PYLD_START_ID                 = 0xA0
PYLD_START_ACK_ID             = 0xA1
PYLD_STOP_ID                  = 0xA2
PYLD_STOP_ACK_ID              = 0xA3
PYLD_REQUEST_TRANSFER_ID      = 0xA4
PYLD_TRANSFER_ACK_ID          = 0xA5
PYLD_TRANSFER_HEADER_ID       = 0xA6
PYLD_HEADER_ACK_ID            = 0xA7
PYLD_PACKET_ID                = 0xA8
PYLD_PACKET_ACK_ID            = 0xA9
PYLD_TRANSFER_COMPLETE_ID     = 0xAA
PYLD_TRANSFER_COMPLETE_ACK_ID = 0xAB
PYLD_ENTER_DEBUG_ID           = 0xAC
PYLD_DEBUG_ACK_ID             = 0xAD

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


def recv_msg(port: serial.Serial):
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

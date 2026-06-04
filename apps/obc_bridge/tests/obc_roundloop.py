#!/usr/bin/env python3
"""
OBC simulator — round-loop test.

Drives both phases of the round-loop test from the OBC side:
  1. Send experiment_settings.csv to the payload (OBC → payload)
  2. Wait for the payload to be ready
  3. Receive results.csv from the payload (payload → OBC)

The payload side is driven by:
  ./test_obc_comms roundloop [wait_seconds]

Usage:
  python3 apps/obc_bridge/tests/obc_roundloop.py <serial_port> [settings_csv] [results_out] [wait_seconds]

  serial_port   e.g. /dev/pts/4
  settings_csv  default: data/test_obc_nominal/experiment_settings.csv
  results_out   default: data/test_obc_nominal/results_received.csv
  wait_seconds  default: 5 (must match the value passed to test_obc_comms)

Dependencies:
  pip install pyserial
"""

import sys
import time
import serial

from uart_helpers import BAUD_RATE
from obc_transmit import load_settings, make_packets, send_settings
from obc_receive  import receive_results


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial_port> [settings_csv] [results_out] [wait_seconds]")
        sys.exit(1)

    serial_port  = sys.argv[1]
    settings_csv = sys.argv[2] if len(sys.argv) > 2 else "data/test_obc_nominal/experiment_settings.csv"
    results_out  = sys.argv[3] if len(sys.argv) > 3 else "data/test_obc_nominal/results_received.csv"
    wait_seconds = int(sys.argv[4]) if len(sys.argv) > 4 else 5

    port = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=30)
    try:
        # Phase 1: send settings to payload
        print("[OBC] Phase 1: sending experiment settings to payload...")
        data    = load_settings(settings_csv)
        packets = make_packets(data)
        send_settings(port, packets)

        # The payload sleeps for wait_seconds before initiating results transfer.
        # The OBC waits here with a generous timeout on recv_msg (set on the port).
        print(f"[OBC] Phase 1 complete. Waiting {wait_seconds}s for payload to initiate results transfer...")
        time.sleep(wait_seconds)

        # Phase 2: receive results from payload
        print("[OBC] Phase 2: receiving results from payload...")
        receive_results(port, results_out)

        print("[OBC] Round-loop test complete.")
    finally:
        port.close()

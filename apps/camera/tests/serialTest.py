#!/usr/bin/env python3

import serial
import time
import sys

# UART config - matches Arduino sketch
UART_PORT = "/dev/ttyAMA0"  # UART0 on Pi 5
BAUD_RATE = 9600            # Match whatever you set in Arduino

def open_serial():
    try:
        ser = serial.Serial(
            port=UART_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2
        )
        return ser
    except serial.SerialException as e:
        print(f"Failed to open {UART_PORT}: {e}")
        print("Check: sudo raspi-config -> Interface Options -> Serial Port")
        sys.exit(1)

def test_basic(ser):
    """Send simple incrementing messages"""
    print("--- Basic Test ---")
    for i in range(10):
        msg = f"Test message {i}\n"
        ser.write(msg.encode())
        print(f"Sent: {msg.strip()}")
        time.sleep(0.5)

def test_burst(ser):
    """Send a burst of data to check for dropped characters"""
    print("\n--- Burst Test ---")
    lines = [
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC",
        "123456789012345678901234567890",
        "!@#$%^&*()_+-=[]{}|;':\",./<>?",
    ]
    for line in lines:
        msg = line + "\n"
        ser.write(msg.encode())
        print(f"Sent: {msg.strip()}")
        time.sleep(0.1)  # Short delay between bursts

def test_boot_simulation(ser):
    """Simulate real boot log output to test realistic conditions"""
    print("\n--- Boot Log Simulation ---")
    fake_boot_log = [
        "Booting Linux on physical CPU 0x0000000000 [0x414fd0b1]",
        "Linux version 6.18.29+rpt-rpi-2712",
        "KASLR enabled",
        "random: crng init done",
        "Machine model: Raspberry Pi 5 Model B Rev 1.1",
        "efi: UEFI not found.",
        "Reserved memory: created CMA memory pool at 0x3a000000",
        "OF: reserved mem: initialized node linux,cma",
        "NUMA: Faking a node at [mem 0x0000000000000000-0x00000001ffffffff]",
        "Zone ranges:",
        "  DMA      [mem 0x0000000000000000-0x00000000ffffffff]",
        "  Normal   [mem 0x0000000100000000-0x00000001ffffffff]",
        "Kernel command line: console=serial0,9600 console=tty1",
        "PID hash table entries: 4096",
        "systemd[1]: Starting Raspberry Pi OS",
        "systemd[1]: Reached target Basic System",
        "raspberrypi login:",
    ]
    for line in fake_boot_log:
        msg = line + "\n"
        ser.write(msg.encode())
        print(f"Sent: {line}")
        time.sleep(0.2)  # Realistic pacing

def test_loopback(ser):
    """If Pi RX is also connected, check we receive what we send"""
    print("\n--- Loopback Test (requires RX connected) ---")
    test_msg = "LOOPBACK_TEST_123\n"
    ser.write(test_msg.encode())
    print(f"Sent:     {test_msg.strip()}")

    time.sleep(0.2)
    if ser.in_waiting:
        received = ser.read(ser.in_waiting).decode(errors='replace')
        print(f"Received: {received.strip()}")
        if test_msg.strip() in received:
            print("Loopback OK")
        else:
            print("Loopback MISMATCH - check wiring")
    else:
        print("Nothing received - RX not connected or Arduino not echoing")

def main():
    print(f"Opening {UART_PORT} at {BAUD_RATE} baud...")
    ser = open_serial()
    print(f"Opened successfully\n")

    try:
        test_basic(ser)
        test_burst(ser)
        test_boot_simulation(ser)
        test_loopback(ser)

        print("\n--- All tests complete ---")
        print("Check Arduino Serial Monitor for received output")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        ser.close()
        print("Serial port closed")

if __name__ == "__main__":
    main()
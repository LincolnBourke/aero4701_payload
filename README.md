# AERO4701 Payload

Implements all code required for a satellite payload that simulates docking using a Stewart platform.
This includes: 
- Stewart platform control: kinematics, trajectory tracking and servo control. 
- Platform pose estimation: event camera image processing pipeline. 

## Requirements
<!-- - CMake 3.16 or higher -->
- C++17 compatible compiler
- Eigen3 (>= 3.3)
- Lightweight communications and marshalling

Required packages can be installed with:
    
```sh
sudo apt-get install libeigen3-dev liblcm-dev
```

## Build Instructions
Generate LCM message files for C++ and Python scripts. From the project directory: 

```bash
cd lcm_messages
lcm-gen -x -p payloadMessages.lcm
```

Create directory for build files and run cmake. From the project directory:

```bash
cd ..
mkdir build
cd build
cmake ..
cmake --build .
```

## Running Apps

All apps are run from the root directory.

C++ apps are run from an entry point with the same name as the app:  

```bash
./build/apps/<app_name>/<app_name>
```

Python apps may have a different entry point name and are not located in build/:

```bash 
python apps/<app_name>/<entry_point>.py
```


## OBC–Payload UART Communication Tests

These tests validate UART communication between the payload computer and OBC on Ubuntu using `socat` virtual serial ports. The C++ binary always plays the **payload role**; the Python scripts always play the **OBC role**.

### Setup

Install dependencies:

```bash
sudo apt install socat
pip install pyserial
```

Create a virtual serial port pair and note the two `/dev/pts/N` paths printed:

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
# e.g. /dev/pts/3  <-->  /dev/pts/4
```

Set `UART_FILE` at the top of `apps/obc_bridge/src/uartInterface.cpp` to the first path, then rebuild:

```cpp
#define UART_FILE "/dev/pts/3"
```

```bash
cd build && make
```

Run all Python scripts from the **project root directory** so that relative paths to `data/` resolve correctly. All three tests require `socat` to be running first.

---

### Test 1 — OBC sends settings to payload (`obc_transmit`)

Tests the inbound path: OBC transmits `experiment_settings.csv` to the payload, which receives and deserialises it into `trajectory_settings.csv` and `scalar_settings.csv`.

**Terminal 1 (payload):**
```bash
./build/apps/obc_bridge/obc_bridge
```

**Terminal 2 (OBC):**
```bash
python3 apps/obc_bridge/tests/obc_transmit.py /dev/pts/4
```

---

### Test 2 — Payload sends results to OBC (`send-results`)

Tests the outbound path in isolation: payload serialises `results.csv` and sends it to the OBC.

**Terminal 1 (payload):**
```bash
./build/apps/obc_bridge/test_obc_comms send-results
```

**Terminal 2 (OBC):**
```bash
python3 apps/obc_bridge/tests/obc_receive.py /dev/pts/4 [output_csv]
```

---

### Test 3 — Full round loop (`roundloop`)

Tests both directions end-to-end: OBC sends settings to payload, payload waits, then sends results back to OBC. Both terminals must be started at roughly the same time.

**Terminal 1 (payload):**
```bash
./build/apps/obc_bridge/test_obc_comms roundloop [wait_seconds]
```

**Terminal 2 (OBC):**
```bash
python3 apps/obc_bridge/tests/obc_roundloop.py /dev/pts/4 [settings_csv] [results_out] [wait_seconds]
```

The `wait_seconds` argument must match between the two terminals (default: 5).

---

### Verification

After Test 2 or Test 3, compare the received results against the original to verify round-trip fidelity:

```bash
diff data/test_obc_nominal/results.csv data/test_obc_nominal/results_received.csv
```

Values may differ in the last decimal place due to float32 quantisation; all differences should be smaller than `0.000001`.

## Notes
Zero angles: [ 90.00, 180.00, 79.71, 180.00, 97.71, 180.00 ] 
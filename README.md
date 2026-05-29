# AERO4701 Payload

Implements control code for the Stewart platform payload. 

<!-- ## Features
- **Modern C++**: Built using C++17/20 standards. -->

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
Generate LCM message files: 

```bash
cd lcm_messages
lcm-gen -x payloadMessages.lcm
```

Create directory for build files and run cmake. From the project directory:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Run with:

```bash
./aero4701_payload
```


## Notes
Zero angles: [ 90.00, 180.00, 79.71, 180.00, 97.71, 180.00 ] 
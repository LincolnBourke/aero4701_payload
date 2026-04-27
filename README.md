# AERO4701 Payload

Implements control code for the Stewart platform payload. 

<!-- ## Features
- **Modern C++**: Built using C++17/20 standards. -->

## Requirements
<!-- - CMake 3.16 or higher -->
- C++17 compatible compiler
- Eigen3 (>= 3.3)
- LCM

Required packages can be installed with:
    
```sh
sudo apt install libeigen3-dev liblcm-dev
```

## Build Instructions
In order to use the LCM topics, language specific bindings must be created. 
Generate these for C++ and Python with,

```bash
cd lcm_definitions
lcm-gen -x payload_topics.lcm
lcm-gen -p payload_topics.lcm
```

Then to build the project, navigate back to the top level directory, create a build directory and run cmake:

```bash
cd ..
mkdir build
cd build
cmake ..
cmake --build .
```

Run with:

```bash
./payload
```

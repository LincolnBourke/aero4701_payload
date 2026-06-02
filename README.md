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
- Libgpiod

Required packages can be installed with:
    
```sh
sudo apt-get install libeigen3-dev liblcm-dev libgpiod-dev
```

## Build Instructions
Generate LCM message files for C++ and Python scripts. From the project directory: 

```bash
cd lcm_messages
lcm-gen -x -p payloadMessages.lcm
```

Create directory for build files and run cmake. From the project directory:

```bash
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


## Notes
Zero angles: [ 90.00, 180.00, 79.71, 180.00, 97.71, 180.00 ] 
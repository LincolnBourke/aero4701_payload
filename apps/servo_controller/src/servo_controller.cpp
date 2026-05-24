#include "servo_controller.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <cmath>

ServoController::ServoController(const char* device, int num_motors, const std::string& channel_name)
    : _num_motors(num_motors), _channel_name(channel_name) {

    std::cout << "Starting Servo Controller..." << std::endl;

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
    }

    // Correct LCM Subscription syntax
    lcm.subscribe(_channel_name, &ServoController::handleServoAngMsg, this);

    // Use the class member _fd, don't redeclare 'int _fd' here!
    _fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd == -1) {
        perror("Error opening serial port");
        return;
    }

    struct termios options;
    tcgetattr(_fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(_fd, TCSANOW, &options);

    // Initialize vectors to the correct size
    _positions.resize(_num_motors, 0.0);

    std::cout << "Initialized Servo Controller on " << device << std::endl;
}

ServoController::~ServoController() {
    if (_fd != -1) close(_fd);
}

void ServoController::handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, const payload_messages::servo_targets_t* msg) 
{
    (void)rbuf; // Cast to void to avoid unused parameter compiler warnings 
    (void)chan;

    // Assume fixed number of servos
    int count = 6;

    printf("Servo controller received message on %s, writing servo targets...", chan.c_str());
    for (int i = 0; i < count; i++)
    {
        std::cout << " s" << i << ": " << msg->angles[i];
    }
    std::cout << std::endl;

    // Extract servo targets and convert to Maestro units
    for (int i = 0; i < count; i++)
    {
        float float_target = msg->angles[i];
        (void)float_target;
        
        // Convert angle to maestro units
        unsigned short maestro_target; // TODO: conversion logic  
        (void)maestro_target;                       

        // Write to hardware
        // _maestroSetTarget(i, maestro_target);
    }
}

// void ServoController::handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
//                     const std::string& chan,
//                     const payload_messages::servo_angs* msg) {
//     (void)rbuf; // Cast to void to avoid unused parameter compiler warnings 
//     (void)chan;
    
//     int count = std::min((int)msg->position.size(), _num_motors);

//     // Sine Wave Parameters
//     double frequency = 0.5;  // 0.5 Hz (one full swing every 2 seconds)
//     double amplitude = 45.0; // Swings +/- 45 degrees
//     double offset = 90.0;    // Centered at 90 degrees

//     // FIX: Convert microseconds to SECONDS (1 second = 1,000,000 microseconds)
//     double t = static_cast<double>(msg->timestamp) / 1000.0;

//     // std::cout << "\n[Sine-Wave Mode] Time: " << std::fixed << std::setprecision(2) << t << "s" << std::endl;

//     // Calculate the target angle once outside the loop since it only depends on time 't'
//     // Equation: Angle = Offset + A * sin(2 * PI * f * t)
//     double sine_angle = offset + amplitude * std::sin(2.0 * M_PI * frequency * t);

//     // Convert the single calculated sine angle to Maestro units
//     unsigned short target = 4000 + (sine_angle * (4000.0 / 180.0));

//     for (int i = 0; i < count; i++) {
//         // 1. CONTROL: Write to hardware
//         _maestroSetTarget(i, target);

//         // // 2. SHOW: Contrast the sensor reading with the generated sine command
//         // std::cout << "  Motor " << i
//         //         << " | Actual (Sensor): " << std::setw(6) << msg->position[i] << " deg"
//         //         << " | Target (Sine): " << std::setw(6) << sine_angle << " deg"
//         //         << std::endl << std::flush;
//     }
// }

void ServoController::_maestroSetTarget(unsigned char channel, unsigned short target) {
    if (_fd == -1) return;

    unsigned char command[] = {
        0x84,
        channel,
        static_cast<unsigned char>(target & 0x7F),
        static_cast<unsigned char>((target >> 7) & 0x7F)
    };

    if (write(_fd, command, sizeof(command)) == -1) {
        perror("Serial write failed");
    }
}

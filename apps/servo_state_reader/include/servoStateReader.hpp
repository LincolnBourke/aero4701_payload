/*
    Defines a class for reading servo feedback angles from an ADS7830
    ADC over I2C and publishing them as LCM messages.
*/

#ifndef SERVO_STATE_READER_H
#define SERVO_STATE_READER_H

#include <cmath>
#include <string>
#include <iomanip> // Required for formatting
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include "true_servo_angles_t.hpp"

/*

channel 1: 
    down: 255
    up: 93

channel 2: 
    down: 42
    up: 255

channel 3: 
    down: 255
    up: 94

channel 4: 
    down: 40
    up: 240 

channel 5: 
    down: 255
    up: 90

channel 6: 
    down: 40
    up: 240
*/

class ServoStateReader
{
    public:
        ServoStateReader(int num_channels, const std::string& channel_name);
        ~ServoStateReader();

        // Reads one sample from each ADC channel and publishes a servo_angs message.
        void pubState();

    private:
        struct ChannelCalibration {
            int    raw_down;    // ADC value when servo arm is at ang_down_rad
            int    raw_up;      // ADC value when servo arm is at ang_up_rad
            double ang_down_rad; // Physical angle (rad) at raw_down: 0=horizontal, +ve=above
            double ang_up_rad;   // Physical angle (rad) at raw_up
        };

        const ChannelCalibration _cal[6] = {
            { 110, 255, -M_PI/4,  M_PI/2},  // ch5 (inverted ADC)
            { 255, 110, -M_PI/4,  M_PI/2},  // ch4
            { 110, 255, -M_PI/4,  M_PI/2},  // ch3 (inverted ADC)
            { 255, 110, -M_PI/4,  M_PI/2},  // ch2
            { 110, 225, -M_PI/4,  M_PI/2},  // ch1 (inverted ADC)
            { 255, 110, -M_PI/4,  M_PI/2},  // ch0
        };


        // Maps an 8-bit ADC reading to an angle in radians (0=horizontal, +ve=above).
        double _mapRawToAngle(int raw, int raw_down, int raw_up, double ang_down_rad, double ang_up_rad);

        // Path to the I2C bus device file.
        const char* _bus_path = "/dev/i2c-1";

        // File descriptor for the I2C bus.
        int _file;

        // Number of ADC channels (one per servo).
        int _num_channels;

        lcm::LCM _lcm;
        std::string _channel_name;
};

#endif


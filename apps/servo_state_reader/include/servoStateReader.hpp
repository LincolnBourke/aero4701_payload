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

channel 0: 
    up: 104
    zero adc value: 217

channel 1: 
    up: 252
    zero adc value: 150

channel 2: 
    up: 100
    zero adc value: 210

channel 3: 
    up: 248
    zero adc value: 129

channel 4: 
    up: 97
    zero adc value: 206

channel 5: 
    up: 249
    zero adc value: 145

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

        // const ChannelCalibration _cal[6] = {
        //     { 217, 104, 0,  M_PI/2},  // ch5 (inverted ADC)
        //     { 150, 252, 0,  M_PI/2},  // ch4
        //     { 210, 100, 0,  M_PI/2},  // ch3 (inverted ADC)
        //     { 129, 248, 0,  M_PI/2},  // ch2
        //     { 206, 97, 0,  M_PI/2},  // ch1 (inverted ADC)
        //     { 145, 249, 0,  M_PI/2},  // ch0
        // };
        // const ChannelCalibration _cal[6] = {
        //     { 145, 249, 0, M_PI/2},  // ch0
        //     { 206,  97, 0, M_PI/2},  // ch1 (inverted ADC)
        //     { 129, 248, 0, M_PI/2},  // ch2
        //     { 210, 100, 0, M_PI/2},  // ch3 (inverted ADC)
        //     { 150, 252, 0, M_PI/2},  // ch4
        //     { 217, 104, 0, M_PI/2},  // ch5 (inverted ADC)
        // };

        const ChannelCalibration _cal[6] = {
            // raw_down, raw_up, ang_down_rad, ang_up_rad
            { 205, 99 , 0, M_PI/2},  // S7
            { 126, 235, 0, M_PI/2},  // S8 (inverted ADC)
            { 198, 96 , 0, M_PI/2},  // S9
            { 142, 249, 0, M_PI/2},  // S10 (inverted ADC)
            { 211, 110, 0, M_PI/2},  // S11
            { 132, 239, 0, M_PI/2},  // S12 (inverted ADC)
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


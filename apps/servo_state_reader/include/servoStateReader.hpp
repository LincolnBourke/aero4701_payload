/*
    Defines a class for reading servo feedback angles from an ADS7830
    ADC over I2C and publishing them as LCM messages.
*/

#ifndef SERVO_STATE_READER_H
#define SERVO_STATE_READER_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "payload_messages/servo_angs.hpp"

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
            int raw_down;   // ADC value at 0 degrees (fully down)
            int raw_up;     // ADC value at 180 degrees (fully up)
        };

        // One entry per channel, from your calibration comments
        const ChannelCalibration _cal[6] = {
            { 40, 240},  // ch6: down=40,  up=240
            {255,  90},  // ch5: down=255, up=90  (inverted)
            { 40, 240},  // ch4: down=40,  up=240
            {255,  94},  // ch3: down=255, up=94  (inverted)
            { 42, 255},  // ch2: down=42,  up=255
            {255,  93},  // ch1: down=255, up=93  (inverted)
        };

        // Maps an 8-bit ADC reading to an angle in degrees using calibration limits.
        double _mapRawToAngle(int raw, int raw_down, int raw_up); // now takes limits

        // // Calibration limits for the ADC-to-angle mapping.
        // const int RAW_MIN = 0;
        // const int RAW_MAX = 128;
        // const double ANG_MIN = 0.0;
        // const double ANG_MAX = 180.0;

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


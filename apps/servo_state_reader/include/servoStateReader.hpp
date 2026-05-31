/*
    Defines a class for reading servo feedback angles from an ADS7830
    ADC over I2C and publishing them as LCM messages.
*/

#ifndef SERVO_STATE_READER_H
#define SERVO_STATE_READER_H

#include <string>
#include <lcm/lcm-cpp.hpp>
#include "payload_messages/servo_angs.hpp"

class ServoStateReader
{
    public:
        ServoStateReader(int num_channels, const std::string& channel_name);
        ~ServoStateReader();

        // Reads one sample from each ADC channel and publishes a servo_angs message.
        void pubState();

    private:
        // Maps an 8-bit ADC reading to an angle in degrees using calibration limits.
        double _mapRawToAngle(int raw);

        // Calibration limits for the ADC-to-angle mapping.
        const int RAW_MIN = 20;
        const int RAW_MAX = 230;
        const double ANG_MIN = 0.0;
        const double ANG_MAX = 180.0;

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

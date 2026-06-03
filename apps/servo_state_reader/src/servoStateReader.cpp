#include "servoStateReader.hpp"

#include <iostream>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <chrono>

#define I2C_ADDR 0x48

ServoStateReader::ServoStateReader(int num_channels, const std::string& channel_name)
    : _num_channels(num_channels), _channel_name(channel_name) {

    std::cout << "Starting state reader" << std::endl;

    if (!_lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
    }

    // Open I2C bus
    if ((_file = open(_bus_path, O_RDWR)) < 0) {
        perror("Failed to open I2C bus");
        return;
    }

    if (ioctl(_file, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("Failed to acquire bus access/talk to slave");
    }

    std::cout << "Initialised state reader" << std::endl;
}

ServoStateReader::~ServoStateReader() {
    if (_file >= 0) close(_file);
}

void ServoStateReader::pubState() {
    payload_messages::servo_angs msg;

    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();

    // Cast to desired unit (e.g., milliseconds) and get numeric count
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    msg.timestamp = millis;
    msg.num_channels = _num_channels;
    msg.position.resize(_num_channels);

    // std::cout << "[INFO] Raw servo angles: [ "; 
    for (int channel = 0; channel < _num_channels; channel++) {
        // --- ADS7830 SPECIFIC COMMAND LOGIC ---
        uint8_t commandByte = 0;

        // Match the Adafruit channel selection logic
        if (channel % 2 == 0) {
            commandByte |= (0x08 + (channel / 2)); // SINGLE_CH0 base
        } else {
            commandByte |= (0x0C + ((channel - 1) / 2)); // SINGLE_CH1 base
        }

        commandByte <<= 4;   // Shift to high nibble
        commandByte |= (0x03 << 2); // Internal Ref ON, AD Converter ON (PD bits)

        // --- I2C TRANSACTION ---
        if (write(_file, &commandByte, 1) != 1) {
            std::cerr << "Failed to write command to ADS7830" << std::endl;
            continue;
        }


        uint8_t adcValue;
        // ADS7830 only returns ONE byte (8-bit resolution)
        if (read(_file, &adcValue, 1) == 1) {
            msg.position[5-channel] = _mapRawToAngle(
                (int)adcValue,
                _cal[channel].raw_down,
                _cal[channel].raw_up
            );
            // std::cout << (int)adcValue << " ";
        } else {
            msg.position[channel] = 0.0;
            // std::cout << "NaN" << " ";
        }
    }
    // std::cout << " ]" << std::endl << std::flush; 

    _lcm.publish(_channel_name, &msg);

    std::cout << "[INFO] Servo angles: [ "; 
    for (int channel = 0; channel < _num_channels-1; channel++) {
        std::cout << msg.position[channel] << ", "; 
    } 
    std::cout << msg.position[_num_channels-1] << " ]" << std::endl << std::flush; 
}

// double ServoStateReader::_mapRawToAngle(int raw) {
//     // Constrain raw value to calibration limits
//     if (raw < RAW_MIN) raw = RAW_MIN;
//     if (raw > RAW_MAX) raw = RAW_MAX;

//     // Linear map to double
//     return (double)(raw - RAW_MIN) * (ANG_MAX - ANG_MIN) / (double)(RAW_MAX - RAW_MIN) + ANG_MIN;
// }

double ServoStateReader::_mapRawToAngle(int raw, int raw_down, int raw_up) {
    // Constrain to the calibrated range (handle inverted channels naturally)
    int lo = std::min(raw_down, raw_up);
    int hi = std::max(raw_down, raw_up);
    raw = std::max(lo, std::min(hi, raw));

    // Linear interpolation: raw_down -> 0 deg, raw_up -> 180 deg
    return (double)(raw - raw_down) / (double)(raw_up - raw_down) * 180.0;
}

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
    payload_messages::true_servo_angles_t msg;

    // 1. Create a temporary vector to store the raw ADC values
    std::vector<int> raw_adcs(_num_channels, 0);

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

        // Read and discard stale reading
        uint8_t stale;
        read(_file, &stale, 1);  // Discard previous conversion result

        // Now should be fresh 
        uint8_t adcValue;

        // ADS7830 only returns ONE byte (8-bit resolution)
        if (read(_file, &adcValue, 1) == 1) {
            // Note: maintaining your existing 5-channel reverse mapping
            msg.angles[5-channel] = _mapRawToAngle(
                (int)adcValue,
                _cal[channel].raw_down,
                _cal[channel].raw_up,
                _cal[channel].ang_down_rad,
                _cal[channel].ang_up_rad
            );
            // Store the raw value in the same reversed index position
            raw_adcs[5-channel] = (int)adcValue; 
        } else {
            // I recommend applying the reverse index here too to prevent array mismatch
            msg.angles[5-channel] = 0.0; 
            raw_adcs[5-channel] = -1; // Use -1 to visually indicate an I2C read error
        }
    }

    _lcm.publish(_channel_name, &msg);

    // --- FORMATTED PRINTING LOGIC ---
    std::cout << "[INFO] Servo [raw, angles]: [ "; 
    
    // Lock the stream to standard decimal notation and set decimal places
    std::cout << std::fixed << std::setprecision(2); 

    for (int i = 0; i < _num_channels; i++) {
        // Print the pair: [raw, angle]
        // setw(3) for 8-bit ADC (0-255), setw(7) for the formatted float
        std::cout << "[" << std::setw(3) << raw_adcs[i] << ", " 
                  << std::setw(7) << (msg.angles[i] * 180.0f / M_PI) << "]";
        
        // Add a comma for all items except the last one
        if (i < _num_channels - 1) {
            std::cout << ", "; 
        }
    } 
    std::cout << " ]" << std::endl;

}

double ServoStateReader::_mapRawToAngle(int raw, int raw_down, int raw_up, double ang_down_rad, double ang_up_rad) {
    // Constrain to the calibrated range (handles inverted channels naturally)
    int lo = std::min(raw_down, raw_up);
    int hi = std::max(raw_down, raw_up);
    raw = std::max(lo, std::min(hi, raw));

    // Linear interpolation: raw_down -> ang_down_rad, raw_up -> ang_up_rad
    // Outputs radians with 0 = horizontal, positive = above, negative = below.
    return (double)(raw - raw_down) / (double)(raw_up - raw_down) * (ang_up_rad - ang_down_rad) + ang_down_rad;
}

#include "limitSwitchReader.hpp"
#include <iostream>

int main()
{
    try {
        float check_rate = 100.0f; // Hz
        std::string channel_name = "LIMIT_SWITCH_STATES";

        LimitSwitchReader reader(check_rate, channel_name);
        reader.run(); // blocks; publishes on change
    }
    catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


/*
// read_gpio.cpp
// Reads physical pins 36, 38, and 40 on the Raspberry Pi 5 and prints
// whether each is HIGH or LOW.
//
// Physical pin -> BCM GPIO mapping:
//   Pin 36 -> GPIO 16
//   Pin 38 -> GPIO 20
//   Pin 40 -> GPIO 21
//
// Uses libgpiod v2 (required for Raspberry Pi 5 / RP1 chip).
//
// Install dependency:
//   sudo apt install libgpiod-dev
//
// Compile:
//   g++ -o read_gpio read_gpio.cpp -lgpiod
//
// Run:
//   ./read_gpio

#include <gpiod.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>

// On the Raspberry Pi 5 the user-facing GPIOs are on gpiochip4 (the RP1).
// From kernel 6.6+ this is remapped so gpiochip0 always points to user pins,
// but gpiochip4 works on all current Raspberry Pi OS Bookworm releases.
static const char* CHIP_PATH = "/dev/gpiochip4";
static const char* CONSUMER  = "read_gpio";

struct PinInfo {
    int         physicalPin;   // physical board pin number (for display)
    unsigned int bcmLine;      // BCM / gpiod line offset
};

int main()
{
    // Physical pin 36 = BCM 16
    // Physical pin 38 = BCM 20
    // Physical pin 40 = BCM 21
    const std::vector<PinInfo> pins = {
        { 36, 16 },
        { 38, 20 },
        { 40, 21 },
    };

    // ── Open the GPIO chip ──────────────────────────────────────────────────
    gpiod_chip* chip = gpiod_chip_open(CHIP_PATH);
    if (!chip) {
        std::cerr << "Error: could not open " << CHIP_PATH
                  << ". Are you running as root or in the 'gpio' group?\n";
        return EXIT_FAILURE;
    }

    // ── Build a request for all three lines ────────────────────────────────
    gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        std::cerr << "Error: gpiod_line_settings_new() failed.\n";
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        std::cerr << "Error: gpiod_line_config_new() failed.\n";
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }

    // Add each line offset to the config
    for (const auto& p : pins) {
        unsigned int offset = p.bcmLine;
        if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0) {
            std::cerr << "Error: could not add line " << offset << " to config.\n";
            gpiod_line_config_free(line_cfg);
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip);
            return EXIT_FAILURE;
        }
    }

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        std::cerr << "Error: gpiod_request_config_new() failed.\n";
        gpiod_line_config_free(line_cfg);
        gpiod_line_settings_free(settings);
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);

    // ── Request the lines ───────────────────────────────────────────────────
    gpiod_line_request* request =
        gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    if (!request) {
        std::cerr << "Error: could not request GPIO lines. "
                  << "Check that no other process holds them.\n";
        gpiod_chip_close(chip);
        return EXIT_FAILURE;
    }

    // ── Read and print each pin value ───────────────────────────────────────
    std::cout << "Raspberry Pi 5 GPIO pin states\n";
    std::cout << "================================\n";

    for (const auto& p : pins) {
        gpiod_line_value val =
            gpiod_line_request_get_value(request, p.bcmLine);

        std::string state;
        if (val == GPIOD_LINE_VALUE_ACTIVE) {
            state = "HIGH (1)";
        } else if (val == GPIOD_LINE_VALUE_INACTIVE) {
            state = "LOW  (0)";
        } else {
            state = "ERROR reading value";
        }

        std::cout << "  Physical pin " << p.physicalPin
                  << " (BCM GPIO " << p.bcmLine << "): "
                  << state << "\n";
    }

    // ── Cleanup ─────────────────────────────────────────────────────────────
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);

    return EXIT_SUCCESS;
}
*/
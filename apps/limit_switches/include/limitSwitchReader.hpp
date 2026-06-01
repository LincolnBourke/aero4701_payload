/*
    Defines a class for reading the state of the 3 limit switches on the payload.

    - Opens the GPIO lines once on construction and holds them for the lifetime
      of the object (avoids rebuilding the request on every poll).
    - Polls at a configurable rate; only publishes over LCM when at least one
      switch state has changed since the last poll.
*/

#ifndef LIMIT_SWITCH_READER_H
#define LIMIT_SWITCH_READER_H

#include "payload_messages/switch_state_t.hpp"

#include <gpiod.h>
#include <lcm/lcm-cpp.hpp>

#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

// == GPIO configuration ========================================================
// Defined inline to avoid multiple-definition errors when included in multiple
// translation units.

// On the Raspberry Pi 5 the user-facing GPIOs are on gpiochip4 (the RP1).
// From kernel 6.6+ this is remapped so gpiochip0 always points to user pins,
// but gpiochip4 works on all current Raspberry Pi OS Bookworm releases.
inline constexpr const char* CHIP_PATH = "/dev/gpiochip4";
inline constexpr const char* CONSUMER  = "limit_switch_reader";

struct PinInfo {
    int          physicalPin; // physical board pin (for logging/debug)
    unsigned int bcmLine;     // BCM GPIO number used as the gpiod line offset
};

// Physical pin 36 = BCM 16
// Physical pin 38 = BCM 20
// Physical pin 40 = BCM 21
inline constexpr std::array<PinInfo, 3> PINS = {{
    { 36, 16 },
    { 38, 20 },
    { 40, 21 },
}};

// == Class definition ==========================================================

class LimitSwitchReader
{
public:
    /**
     * @param check_rate   Polling frequency in Hz.
     * @param channel_name LCM channel to publish switch states on.
     *
     * @throws std::runtime_error if LCM or GPIO initialisation fails.
     */
    LimitSwitchReader(float check_rate, const std::string& channel_name);
    ~LimitSwitchReader();

    // Non-copyable: owns raw GPIO handles.
    LimitSwitchReader(const LimitSwitchReader&)            = delete;
    LimitSwitchReader& operator=(const LimitSwitchReader&) = delete;

    /**
     * Blocking spin loop. Polls GPIO at _check_rate Hz and publishes over LCM
     * whenever any switch state changes.
     */
    void run();

    // Expose LCM so the caller can call lcm.handle() in its own loop if preferred.
    lcm::LCM lcm;

private:
    /**
     * Reads current GPIO states. Publishes over LCM only if any value differs
     * from _prev_states. Updates _prev_states on change.
     */
    void _checkAndPublish();

    float       _check_rate;
    std::string _channel_name;

    gpiod_chip*         _chip;
    gpiod_line_request* _request; // held open for the lifetime of the object

    // Last-known states for each pin (indexed to match PINS).
    // Initialised to -1 so the first poll always triggers a publish.
    std::array<int, PINS.size()> _prev_states;
};

#endif // LIMIT_SWITCH_READER_H
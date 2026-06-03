#include "limitSwitchReader.hpp"

#include <stdexcept>
#include <thread>

// == Constructor ===============================================================
LimitSwitchReader::LimitSwitchReader(float check_rate, const std::string& channel_name)
    : _check_rate(check_rate)
    , _channel_name(channel_name)
    , _chip(nullptr)
    , _request(nullptr)
{
    std::cout << "Starting Limit Switch Reader on channel '"
              << _channel_name << "' at " << _check_rate << " Hz\n";

    // Validate LCM 
    if (!lcm.good()) {
        throw std::runtime_error("Failed to initialise LCM");
    }

    // Open GPIO chip
    _chip = gpiod_chip_open(CHIP_PATH);
    if (!_chip) {
        throw std::runtime_error(
            std::string("Could not open GPIO chip at ") + CHIP_PATH +
            ", are you running as root or in the 'gpio' group?");
    }

    // Configure lines as inputs
    gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        gpiod_chip_close(_chip);
        throw std::runtime_error("gpiod_line_settings_new() failed");
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);

    gpiod_line_config* line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        gpiod_line_settings_free(settings);
        gpiod_chip_close(_chip);
        throw std::runtime_error("gpiod_line_config_new() failed");
    }

    for (const auto& p : PINS) {
        unsigned int offset = p.bcmLine;
        if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0) {
            gpiod_line_config_free(line_cfg);
            gpiod_line_settings_free(settings);
            gpiod_chip_close(_chip);
            throw std::runtime_error(
                "Could not add BCM line " + std::to_string(p.bcmLine) + " to config");
        }
    }
    gpiod_line_settings_free(settings); // no longer needed after adding to config

    gpiod_request_config* req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        gpiod_line_config_free(line_cfg);
        gpiod_chip_close(_chip);
        throw std::runtime_error("gpiod_request_config_new() failed");
    }
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);

    // Request all lines in one call, held open until the destructor.
    _request = gpiod_chip_request_lines(_chip, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);

    if (!_request) {
        gpiod_chip_close(_chip);
        throw std::runtime_error(
            "Could not request GPIO lines — is another process holding them?");
    }

    // Initialise previous states to -1 so the first poll always publishes.
    _prev_states.fill(-1);
    std::cout << "GPIO lines acquired successfully\n";
}

// == Destructor ================================================================
LimitSwitchReader::~LimitSwitchReader()
{
    if (_request) {
        gpiod_line_request_release(_request);
    }
    if (_chip) {
        gpiod_chip_close(_chip);
    }
}

// == run(); blocking poll loop ================================================
void LimitSwitchReader::run()
{
    using clock    = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    const duration period(1.0 / _check_rate);

    std::cout << "Polling limit switches (Ctrl-C to stop)...\n";

    while (true) {
        auto t0 = clock::now();

        _checkAndPublish();

        // Sleep for the remainder of the period to maintain the target rate.
        auto elapsed = clock::now() - t0;
        auto sleep_for = period - elapsed;
        if (sleep_for > duration::zero()) {
            std::this_thread::sleep_for(sleep_for);
        }
    }
}

// == _checkAndPublish() ========================================================
void LimitSwitchReader::_checkAndPublish() {
    std::array<int, PINS.size()> current_states;
    bool changed = false;

    for (std::size_t i = 0; i < PINS.size(); ++i) {
        gpiod_line_value val = gpiod_line_request_get_value(_request, PINS[i].bcmLine);

        if (val == GPIOD_LINE_VALUE_ERROR) {
            std::cerr << "Error reading BCM GPIO " << PINS[i].bcmLine << "\n";
            current_states[i] = _prev_states[i]; // keep previous on error
        } 
        else {
            current_states[i] = (val == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
        }

        if (current_states[i] != _prev_states[i]) {
            changed = true;
        }
    }

    // == Build and publish LCM message =========================================
    payload_messages::switch_state_t msg{};
    msg.switch1 = current_states[0];
    msg.switch2 = current_states[1];
    msg.switch3 = current_states[2];

    auto t0 = std::chrono::steady_clock::now();
    msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        t0.time_since_epoch()
    ).count();

    lcm.publish(_channel_name, &msg);

    // Log only on change
    if (changed) {
        for (std::size_t i = 0; i < PINS.size(); ++i) {
            if (current_states[i] != _prev_states[i]) {
                std::cout << "  Pin " << PINS[i].physicalPin
                        << " (BCM " << PINS[i].bcmLine << "): "
                        << (_prev_states[i] < 0 ? "?" : std::to_string(_prev_states[i]))
                        << " -> " << current_states[i] << "\n";
            }
        }
        std::cout << "-----------\n";
        _prev_states = current_states;
    }
}
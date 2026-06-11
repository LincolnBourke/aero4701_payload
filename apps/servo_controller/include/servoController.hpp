/*
    Defines a class for controlling servo motors via a Pololu Maestro
    serial servo controller. Subscribes to LCM servo angle messages
    and drives each servo to the commanded target.
*/

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <vector>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "payload_messages/servo_targets_t.hpp"
#include "payload_messages/servo_activation_t.hpp"

class ServoController
{
    public:
        ServoController(const char* device, int num_motors, const std::string& channel_name);
        ~ServoController();

        // Opens the serial port so servo targets can be written to hardware.
        void activateServos();

        // Sends target=0 to all channels, then closes the serial port. Safe to call when already inactive.
        void neutraliseServos();

        // Handles incoming servo angle messages and writes targets to hardware.
        void handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
                               const std::string& chan,
                               const payload_messages::servo_targets_t* msg);

        // Handles incoming servo activation messages.
        void handleServoActivationMsg(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const payload_messages::servo_activation_t* msg);

        // Public LCM instance used by main to pump the message loop.
        lcm::LCM lcm;

    private:
        // Sends a Pololu compact protocol SET_TARGET command over serial.
        void _maestroSetTarget(unsigned char channel, unsigned short target);

        // File descriptor for the serial port.
        int _fd = -1;

        // Number of servo motors managed by this controller.
        int _num_motors;

        std::string _device;
        std::string _channel_name;

        // Last commanded positions for each motor.
        std::vector<double> _positions;
};

#endif

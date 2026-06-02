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

class ServoController
{
    public:
        ServoController(const char* device, int num_motors, const std::string& channel_name);
        ~ServoController();

        // Handles incoming servo angle messages and writes targets to hardware.
        void handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
                               const std::string& chan,
                               const payload_messages::servo_targets_t* msg);

        // Sends all servos to their neutral (zero-angle) position.
        void neutraliseServos();

        // Public LCM instance used by main to pump the message loop.
        lcm::LCM lcm;

    private:
        // Sends a Pololu compact protocol SET_TARGET command over serial.
        void _maestroSetTarget(unsigned char channel, unsigned short target);

        // File descriptor for the serial port.
        int _fd = -1;

        // Number of servo motors managed by this controller.
        int _num_motors;

        std::string _channel_name;

        // Last commanded positions for each motor.
        std::vector<double> _positions;
};

#endif

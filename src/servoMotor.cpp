#include "servoMotor.hpp"

ServoMotor::ServoMotor() 
{
    // TODO: update to initialise with the true position of the servo motor
    angle = 0.0f;
};

ServoMotor::~ServoMotor(){};

float ServoMotor::getAngle()
{
    return angle;
};
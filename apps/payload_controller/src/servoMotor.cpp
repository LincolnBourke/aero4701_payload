#include "servoMotor.hpp"

ServoMotor::ServoMotor() 
{
    // TODO: update to initialise with the true position of the servo motor
    angle = -0.615f;
};

ServoMotor::~ServoMotor(){};

float ServoMotor::getAngle()
{
    return angle;
};

void ServoMotor::setAngle(float angle)
{
    this->angle = angle;
}
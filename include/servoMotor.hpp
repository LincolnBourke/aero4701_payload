/*
    Defines a class for representing the servo motors of the Stewart platform.
*/

#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

class ServoMotor
{
    private: 
        // The angle of the servo motor relative to the horizontal.
        // Angles above the horizontal are positive. 
        float angle; 

    public: 
        ServoMotor();
        ~ServoMotor();

        float getAngle();
};

#endif
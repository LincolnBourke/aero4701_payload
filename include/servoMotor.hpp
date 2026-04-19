/*
    Defines a class for representing the servo motors of the Stewart platform.
*/

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

        // Used solely for testing, the angle should be set by motor feedback
        void setAngle(float angle);
};
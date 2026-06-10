#include "servoController.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <array>

#define MAGIC_OFFSET 3000
#define MAGIC_OFFSET_CORRECTION 700

ServoController::ServoController(const char* device, int num_motors, const std::string& channel_name)
    : _num_motors(num_motors), _channel_name(channel_name) {

    std::cout << "Starting Servo Controller..." << std::endl;

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
    }

    // Correct LCM Subscription syntax
    lcm.subscribe(_channel_name, &ServoController::handleServoAngMsg, this);

    // Use the class member _fd, don't redeclare 'int _fd' here!
    _fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd == -1) {
        perror("Error opening serial port");
        return;
    }

    struct termios options;
    tcgetattr(_fd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(_fd, TCSANOW, &options);

    // Initialize vectors to the correct size
    _positions.resize(_num_motors, 0.0);

    std::cout << "Initialized Servo Controller on " << device << std::endl;
}

ServoController::~ServoController() {
    if (_fd != -1) close(_fd);
}

void ServoController::handleServoAngMsg(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, const payload_messages::servo_targets_t* msg) 
{
    (void)rbuf; // Cast to void to avoid unused parameter compiler warnings 
    (void)chan;

    // Assume fixed number of servos
    int count = 6;

    printf("Servo controller received message on %s, writing servo targets...", chan.c_str());
    for (int i = 0; i < count; i++)
    {
        std::cout << " s" << i << ": " << msg->angles[i] * 180 / M_PI;
    }
    std::cout << std::endl;

    /* 
    Calibration measurements from Maestro Control Center 

    Servo 0
        Up:         917.50
        Horizontal: 1880.00 
        ADC limit:  2224.00 

    Servo 1: 
        Up:         2189.00
        Horizontal: 1300.00
        ADC limit:  2224.00 
    

    Servo 2: 
        Up:         877.50
        Horizontal: 1841.75
        ADC limit:  2224.00

    Servo 3: 
        Up:         2166.75
        Horizontal: 1117.00
        ADC limit:  2224.00

    Servo 4: 
        Up:         854.75
        Horizontal: 1784.50
        ADC limit:  2224.00

    Servo 5: 
        Up:         2184.00
        Horizontal: 1271.25
        ADC limit:  2224.00

    */

    std::array<float, 6> max_pwm = {917.50, 2189.0, 877.5, 2166.75, 854.75, 2184.0};
    std::array<float, 6> flat_pwm = {1880.0, 1300.0, 1841.75, 1117.0, 1784.5, 1271.25};
    // const float limit_pwm = 2224.0;

    // calculate how many pwm units there are per radian
    std::array<float, 6> units_per_radian = {0};
    for (int i = 0; i < count; i++)
    {
        units_per_radian[i] = std::abs(max_pwm[i] - flat_pwm[i]) / (M_PI/2);
    }

    // const int EVEN_MIN = 816;
    // const int EVEN_MAX = 3008;
    // const int ODD_MIN = 384;
    // const int ODD_MAX = 2112;
    // int even_centre = (EVEN_MAX + EVEN_MIN) / 2;
    // int odd_centre = (ODD_MAX + ODD_MIN) / 2;
    // int even_units_per_radian = (EVEN_MAX - EVEN_MIN) / M_PI;
    // int odd_units_per_radian = (ODD_MAX - ODD_MIN) / M_PI;

    // Extract servo targets and convert to Maestro units
    std::cout << "Maestro targets:";
    for (int i = 0; i < count; i++)
    {
        float angle_rad = msg->angles[i];
        
        // Convert the angle in radians to a PWM signal 
        unsigned short maestro_target;
        if (i % 2 == 1) // Odd - range max is up
        {
            // maestro_target = mid_point + float_target * half_range / (M_PI/2) + MAGIC_OFFSET;
            // maestro_target = odd_centre + angle_rad * odd_units_per_radian;
            maestro_target = flat_pwm[i] + angle_rad * units_per_radian[i];
        }
        else // Even - range max is down
        {
            // maestro_target = mid_point - float_target * half_range / (M_PI/2) + MAGIC_OFFSET + MAGIC_OFFSET_CORRECTION;
            // maestro_target = even_centre - angle_rad * even_units_per_radian;
            maestro_target = flat_pwm[i] - angle_rad * units_per_radian[i]; 
        }
        std::cout << " s" << i << ": " << maestro_target;
        // Write to hardware
        _maestroSetTarget(i, maestro_target);
    }
    std::cout << std::endl;
}

// void ServoController::neutraliseServos()
// {
//     std::cout << "[INFO] Neutralising all servos." << std::endl;

//     const int EVEN_MIN = 816,  EVEN_MAX = 3008;
//     const int ODD_MIN  = 384,  ODD_MAX  = 2112;
//     int even_centre = (EVEN_MAX + EVEN_MIN) / 2;
//     int odd_centre  = (ODD_MAX  + ODD_MIN)  / 2;

//     for (int i = 0; i < _num_motors; i++)
//     {
//         unsigned short centre = (i % 2 == 1) ? odd_centre : even_centre;
//         _maestroSetTarget(i, centre);
//     }
// }

void ServoController::neutraliseServos()
{
    std::cout << "[INFO] Disabling all servos." << std::endl;

    for (int i = 0; i < _num_motors; i++)
    {
        _maestroSetTarget(i, 0);
    }
}

void ServoController::_maestroSetTarget(unsigned char channel, unsigned short target) {
    if (_fd == -1) return;

    target = target * 4; // Converting milliseconds to quarter microseconds

    unsigned char command[] = {
        0x84,
        channel,
        static_cast<unsigned char>(target & 0x7F),
        static_cast<unsigned char>((target >> 7) & 0x7F)
    };

    if (write(_fd, command, sizeof(command)) == -1) {
        perror("Serial write failed");
    }
}

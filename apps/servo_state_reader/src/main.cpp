#include "servoStateReader.hpp"
#include <unistd.h>

int main()
{
    // std::cout << "Hello, World!" << std::endl;

    ServoStateReader reader(6, "SERVO_STATE");

    while (true)
    {
        reader.pubState();
        // 10 seconds is quite long for a control loop;
        // usleep(10000) would give you 100Hz
        usleep(10000);
    }

    return 0;
}

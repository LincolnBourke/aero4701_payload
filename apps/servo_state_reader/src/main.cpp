#include "servoStateReader.hpp"
#include <unistd.h>

int main()
{
    // std::cout << "Hello, World!" << std::endl;

    ServoStateReader reader(6, "SERVO_STATE");

    while (true)
    {
        reader.pubState();
        // usleep(10000) would give you 100Hz
        usleep(20000);
    }

    return 0;
}

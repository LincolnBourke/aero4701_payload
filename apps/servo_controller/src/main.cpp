#include "servoController.hpp"
#include <iostream>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_shutdown{false};

static void handleSigint(int /*sig*/)
{
    g_shutdown.store(true);
}

int main()
{
    std::signal(SIGINT, handleSigint);

    ServoController controller("/dev/ttyAMA0", 6, "SERVO_TARGETS");
    std::cout << "Waiting for LCM messages..." << std::endl;
    while (!g_shutdown.load())
    {
        controller.lcm.handleTimeout(500);
    }

    std::cout << "[INFO] Shutdown requested." << std::endl;
    controller.neutraliseServos();

    return 0;
}
#include "payload_cont_to_cam_msg_t.hpp"
#include "cam_msg_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <iostream>

class CameraResponder
{
public:
    CameraResponder() : lcm()
    {
        lcm.subscribe("PAYLOAD_CAM", &CameraResponder::handleContMsg, this);
    }

    void run()
    {
        std::cout << "[INFO] Dummy camera responder listening on PAYLOAD_CAM" << std::endl;
        while (true) { lcm.handle(); }
    }

private:
    lcm::LCM lcm;

    void handleContMsg(const lcm::ReceiveBuffer*,
                       const std::string& channel,
                       const payload_messages::payload_cont_to_cam_msg_t* msg)
    {
        std::cout << "[INFO] Received from " << channel
                  << " state=" << (int)msg->cont_state << " - replying success" << std::endl;

        payload_messages::cam_msg_t reply;
        reply.cam_status = true;
        lcm.publish("CAM_PAYLOAD", &reply);
    }
};

int main()
{
    CameraResponder responder;
    responder.run();
    return 0;
}

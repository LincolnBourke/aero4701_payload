// New dummy based on updated payloadController to test integration with camera without trajectory functionality 
// Dummy payload controller modelled on new PayloadController with camera functionality
// Platform/trajectory functionality replaced with sleep timers to emulate functionality
// Uses two LCM channels for controller --> camera, camera --> controller
//
// Run instructions from camera repo:
// g++ -std=c++17 dummyPayloadController.cpp -o dummyPayloadController $(pkg-config --cflags --libs lcm)

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <filesystem>

#include <lcm/lcm-cpp.hpp>
#include "payload_messages/payload_cont_to_cam_msg_t.hpp"
#include "payload_messages/cam_msg_t.hpp"

// States for the core controller state machine — must match payloadController.hpp
typedef enum State {
    IDLE,               // Waits for command to start an experiment
    READ_TRAJECTORY,    // Reads the trajectory file & converts to interpolated servo angles
    CALIBRATE_SERVOS,   // Uses limit switches to calibrate servos
    CALIBRATE_CAMERA,   // Moves platform to let camera node calibrate
    DEPLOY,             // Deploys the docking port to the starting position
    RUNNING,            // Runs the experiment
    SAVE_RESULTS,       // Saves experiment data and tells the camera node to do the same
    TERMINATE_RUN,      // Moves the platform back to the home position
    ERROR,              // Publishes an erroneous run result
} state_t;

// LCM channel names — must match payloadController.cpp
static const char* CH_CONT_TO_CAM = "PAYLOAD_CAM";   // controller --> camera
static const char* CH_CAM_TO_CONT = "CAM_PAYLOAD";   // camera --> controller

// Timeout constants (ms) — must match payloadController.cpp
static const int CAM_WAIT_TIMEOUT_CALIB_MS   = 60000;   // 1 min for calibration
static const int CAM_WAIT_TIMEOUT_SAVE_MS    = 40000;   // 30s processing + 10s buffer
static const int CAM_WAIT_TIMEOUT_DEFAULT_MS = 10000;   // 10s for all other states

// Debug mode flag — must match payloadController.cpp
static const bool DEBUG_MODE = true;


// LCM message handler
class LcmHandler
{
public:
    bool cam_status_received = false;
    bool cam_status          = false;

    void handleCamMsg(const lcm::ReceiveBuffer*, const std::string&,
                      const payload_messages::cam_msg_t* msg)
    {
        cam_status_received = true;
        cam_status          = msg->cam_status;
        std::cout << "[INFO] Received cam_msg_t: cam_status=" << std::boolalpha << msg->cam_status
                  << " (raw=" << (int)msg->cam_status << ")" << std::endl;
    }

    void reset() { cam_status_received = false; cam_status = false; }
};


// Dummy PayloadController
class DummyPayloadController
{
public:
    DummyPayloadController() : lcm_(), handler_()
    {
        if (!lcm_.good())
        {
            std::cout << "[ERROR] LCM object not good." << std::endl;
        }
        lcm_.subscribe(CH_CAM_TO_CONT, &LcmHandler::handleCamMsg, &handler_);
    }

    void run()
    {
        state_t state = IDLE;

        while (true)
        {
            switch (state)
            {
            // ----------------------------------------------------------------
            case IDLE:
                // Auto-start for testing — real controller waits for RUN_COMMAND
                std::cout << "[INFO] State: IDLE - auto-starting in 1s..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                state = READ_TRAJECTORY;
                std::cout << "[INFO] State set to READ_TRAJECTORY." << std::endl;
                break;

            // ----------------------------------------------------------------
            case READ_TRAJECTORY:
                // Simulating trajectory read — real controller calls buildTrajectory()
                std::cout << "[INFO] State: READ_TRAJECTORY - simulating 1s trajectory read..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                state = CALIBRATE_SERVOS;
                std::cout << "[INFO] State set to CALIBRATE_SERVOS." << std::endl;
                break;

            // ----------------------------------------------------------------
            case CALIBRATE_SERVOS:
                // Simulating servo calibration — real controller uses limit switches
                std::cout << "[INFO] State: CALIBRATE_SERVOS - simulating 2s servo calibration..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                state = CALIBRATE_CAMERA;
                std::cout << "[INFO] State set to CALIBRATE_CAMERA." << std::endl;
                break;

            // ----------------------------------------------------------------
            case CALIBRATE_CAMERA:
                std::cout << "[INFO] State: CALIBRATE_CAMERA" << std::endl;

                publishCameraCommand(CALIBRATE_CAMERA, DEBUG_MODE);

                // Wait for camera to report complete (1 min timeout for calibration)
                if (!waitForCamStatus(CAM_WAIT_TIMEOUT_CALIB_MS))
                {
                    state = ERROR;
                    std::cout << "[INFO] State set to ERROR." << std::endl;
                    break;
                }

                state = DEPLOY;
                std::cout << "[INFO] State set to DEPLOY." << std::endl;
                break;

            // ----------------------------------------------------------------
            case DEPLOY:
                // Simulating platform deploy — real controller calls deployPlatformStep()
                std::cout << "[INFO] State: DEPLOY - simulating 5s platform deploy..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));

                publishCameraCommand(DEPLOY, DEBUG_MODE);

                // Wait for camera to report complete
                if (!waitForCamStatus(CAM_WAIT_TIMEOUT_DEFAULT_MS))
                {
                    state = TERMINATE_RUN;
                    std::cout << "[INFO] State set to TERMINATE_RUN." << std::endl;
                    break;
                }

                state = RUNNING;
                std::cout << "[INFO] State set to RUNNING." << std::endl;
                break;

            // ----------------------------------------------------------------
            case RUNNING:
                std::cout << "[INFO] State: RUNNING" << std::endl;

                // 1s buffer for camera python to be ready
                std::this_thread::sleep_for(std::chrono::seconds(1));

                publishCameraCommand(RUNNING, DEBUG_MODE);

                // Simulating trajectory execution — real controller calls trackTrajectoryStep()
                std::cout << "[INFO] Simulating 30s experiment..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(30));
                
                // 3s buffer for camera python to be ready
                std::this_thread::sleep_for(std::chrono::seconds(3));

                state = SAVE_RESULTS;
                std::cout << "[INFO] State set to SAVE_RESULTS." << std::endl;
                break;

            // ----------------------------------------------------------------
            case SAVE_RESULTS:
            {
                std::cout << "[INFO] State: SAVE_RESULTS" << std::endl;
                
                std::string results_dir = "outputs/experiment_results";
                std::filesystem::create_directories(results_dir);

                // Write one zero record to initialise the file before Python appends
                std::ofstream results_file(results_dir + "/experiment_results.bin", std::ios::binary);
                if (!results_file.is_open())
                {
                    std::cout << "[ERROR] Failed to open results file." << std::endl;
                    state = ERROR;
                    break;
                }
                float zeros[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                results_file.write(reinterpret_cast<const char*>(zeros), sizeof(zeros));
                results_file.close();

                // 1s buffer for camera python to be ready
                std::this_thread::sleep_for(std::chrono::seconds(1));

                publishCameraCommand(SAVE_RESULTS, DEBUG_MODE);

                // Wait for camera to report complete
                if (!waitForCamStatus(CAM_WAIT_TIMEOUT_SAVE_MS))
                {
                    state = ERROR;
                    std::cout << "[INFO] State set to ERROR." << std::endl;
                    break;
                }

                state = IDLE;
                std::cout << "[INFO] State set to IDLE." << std::endl;
                break;
            }

            // ----------------------------------------------------------------
            case TERMINATE_RUN:
                // Simulating platform retract — real controller calls retractPlatform()
                std::cout << "[INFO] State: TERMINATE_RUN - simulating 3s platform retract..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));

                publishCameraCommand(TERMINATE_RUN, DEBUG_MODE);

                // Wait for camera to report complete
                if (!waitForCamStatus(CAM_WAIT_TIMEOUT_DEFAULT_MS))
                {
                    state = ERROR;
                    std::cout << "[INFO] State set to ERROR." << std::endl;
                    break;
                }

                state = IDLE;
                std::cout << "[INFO] State set to IDLE." << std::endl;
                break;

            // ----------------------------------------------------------------
            case ERROR:
                std::cout << "[ERROR] State: ERROR - publishing to camera and returning to IDLE." << std::endl;

                // Publish ERROR state to camera (no wait — best effort)
                publishCameraCommand(ERROR, DEBUG_MODE);

                state = IDLE;
                std::cout << "[INFO] State set to IDLE." << std::endl;
                break;

            // ----------------------------------------------------------------
            default:
                std::cout << "[ERROR] Invalid state." << std::endl;
                state = IDLE;
                break;
            }
        }
    }

private:
    lcm::LCM   lcm_;
    LcmHandler handler_;

    void publishCameraCommand(state_t state, bool debug_mode)
    {
        payload_messages::payload_cont_to_cam_msg_t msg;
        msg.cont_state = static_cast<int8_t>(state);
        msg.debug_mode = debug_mode;
        lcm_.publish(CH_CONT_TO_CAM, &msg);
        std::cout << "[INFO] Published to " << CH_CONT_TO_CAM
                  << ": cont_state=" << static_cast<int>(state)
                  << " debug_mode=" << debug_mode << std::endl;
    }

    bool waitForCamStatus(int timeout_ms)
    {
        handler_.reset();
        auto start = std::chrono::steady_clock::now();

        while (!handler_.cam_status_received)
        {
            lcm_.handleTimeout(100);

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();

            if (elapsed >= timeout_ms)
            {
                std::cout << "[ERROR] Timed out waiting for camera response." << std::endl;
                return false;
            }
        }

        if (!handler_.cam_status)
        {
            std::cout << "[ERROR] Camera reported failure (cam_status=false)." << std::endl;
            return false;
        }

        std::cout << "[INFO] Camera reported success." << std::endl;
        return true;
    }
};


int main()
{
    std::cout << "[INFO] Starting dummy payload controller..." << std::endl;
    DummyPayloadController controller;
    controller.run();
    return 0;
}

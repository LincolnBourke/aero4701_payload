// dummy payload controller modelled on original with extra camera functionality (publishing, subscribing, CALIBRATE_CAM state, sleep timers)
// platform/trajectory functionality removed from original, but can be slotted back in the order from the photo on discord
// - Some platform/trajectory steps replaced with 3-5s timers to emulate functionality, so they should be removed too
// To test it all from one script it also includes a test LcmHandler, so the functions for that here should be commbined with the existing version
// Uses two LCM channels for controller --> camera, camera --> controller

// Note for testing, it starts in IDLE then transitions to SETUP after 1s

// Run instructions from camera repo: 
// g++ -std=c++17 payload_comp.cpp -o payload_comp $(pkg-config --cflags --libs lcm)
// Run from base of repo ~/Documents/AERO4701_event_camera_emulation with: ./scripts/comp_integration/pi/dummyPayloadController

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <filesystem>


#include <lcm/lcm-cpp.hpp>
#include "exlcm/payload_cont_to_cam_msg_t.hpp"
#include "exlcm/cam_msg_t.hpp"

// States for the core controller state machine
typedef enum State {
    IDLE,           // Waits for command to start an experiment
    SETUP,          // Placeholder setup step
    CALIBRATE_CAM,  // Focus and calibrate camera
    DEPLOY,         // Deploy platform (simulated) + start camera
    RUNNING,        // Run experiment (simulated)
    SAVE_RESULTS,   // Save results + wait for camera
    TERMINATE_RUN,  // Retract platform (simulated) + stop camera
    ERROR,          // Publish error to camera + return to IDLE
} state_t;


// Auto generated LCM channels and LcmHandler from claude for testing it all in one script
// LCM channel names
// Need to be two separate channels, or else controller attempts to receive its own published message
static const char* CH_CONT_TO_CAM = "PAYLOAD_CAM";   // controller --> camera
static const char* CH_CAM_TO_CONT = "CAM_PAYLOAD";   // camera --> controller

// Timeout constants (ms)
static const int CAM_WAIT_TIMEOUT_CALIB_MS  = 120000;  // 2 min for calibration - need this to wait enough time for camera to calibrate 
static const int CAM_WAIT_TIMEOUT_EXP_MS = 40000;  // 30s for experiment + 10s buffer - can remove depending how it relies on platform trajectory to also finish
static const int CAM_WAIT_TIMEOUT_SAVE_MS = 40000;  // 30s for processing + 10s buffer - need extra time to process results and save pose estimates

static const int CAM_WAIT_TIMEOUT_DEFAULT_MS = 10000;  // 10s for all other states


// LCM message handler
class LcmHandler
{
public:
    bool cam_status_received = false;
    bool cam_status          = false;
 
    void handleCamMsg(const lcm::ReceiveBuffer*, const std::string&,
                      const exlcm::cam_msg_t* msg)
    {
        cam_status_received = true;
        cam_status          = msg->cam_status;
        std::cout << "[INFO] Received cam_msg_t: cam_status=" << std::boolalpha << msg->cam_status
                  << " (raw=" << (int)msg->cam_status << ")" << std::endl;
    }
 
    // Clear flag before waiting for a new message
    void reset() { cam_status_received = false; cam_status = false; }
};
 
 
// Dummy TestPayloadController (modelled off original PayloadController)
class TestPayloadController
{
public:
    TestPayloadController() : lcm_(), handler_()
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
                // Note: removed checkRunCommand for testing. Just autostart instead
                std::cout << "[INFO] State: IDLE - auto-starting in 1s..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                state = SETUP;
                std::cout << "[INFO] State set to SETUP." << std::endl;
                break;
 
            // ----------------------------------------------------------------
            case SETUP:
                std::cout << "[INFO] State: SETUP." << std::endl;
                state = CALIBRATE_CAM;
                std::cout << "[INFO] State set to CALIBRATE_CAM." << std::endl;
                break;
 
            // ----------------------------------------------------------------
            case CALIBRATE_CAM:
                std::cout << "[INFO] State: CALIBRATE_CAM" << std::endl;
 
                // Publish CALIBRATE_CAM state to camera with debug_mode = true
                // Note: change this true to a debug mode flag
                publishCameraCommand(CALIBRATE_CAM, /*debug_mode=*/true);
 
                // Wait for camera to report complete (2 min timeout for calibration)
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
                std::cout << "[INFO] State: DEPLOY" << std::endl;
                std::cout << "[INFO] Simulating 5s platform deploy..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));
 
                // Note: this would go inside platform_deployed block
                // Publish DEPLOY state to camera
                publishCameraCommand(DEPLOY, /*debug_mode=*/false);
 
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
 
                // Need 1s buffer for camera python to be ready
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // Publish RUNNING state to camera
                publishCameraCommand(RUNNING, /*debug_mode=*/false);
 
                std::cout << "[INFO] Simulating 30s experiment..." << std::endl;
 
                // Wait for camera to report complete
                if (!waitForCamStatus(CAM_WAIT_TIMEOUT_EXP_MS))
                {
                    state = TERMINATE_RUN;
                    std::cout << "[INFO] State set to TERMINATE_RUN." << std::endl;
                    break;
                }
 
                state = SAVE_RESULTS;
                std::cout << "[INFO] State set to SAVE_RESULTS." << std::endl;
                break;
 
            // ----------------------------------------------------------------
            case SAVE_RESULTS:
            {
                std::cout << "[INFO] State: SAVE_RESULTS" << std::endl;
 
                // Need 1s buffer for camera python to be ready
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // Create results binary file
                std::string results_dir = "outputs/experiment_results";
                std::filesystem::create_directories(results_dir);
 
                std::ofstream results_file(results_dir + "/experiment_results.bin", std::ios::binary);
                if (!results_file.is_open())
                {
                    std::cout << "[ERROR] Failed to open results file." << std::endl;
                    state = ERROR;
                    break;
                }
 
                // Publish SAVE_RESULTS state to camera
                publishCameraCommand(SAVE_RESULTS, /*debug_mode=*/false);
 
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
                std::cout << "[INFO] State: TERMINATE_RUN" << std::endl;
                std::cout << "[INFO] Simulating 3s platform retract..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
 
                // Publish TERMINATE_RUN state to camera
                publishCameraCommand(TERMINATE_RUN, /*debug_mode=*/false);
 
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
                std::cout << "[ERROR] State: ERROR." << std::endl;
                std::cout << "[ERROR] Publishing to camera and returning to IDLE." << std::endl;
 
                // Publish ERROR state to camera (no wait — best effort)
                publishCameraCommand(ERROR, /*debug_mode=*/false);
 
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
 
    // Publish current controller state + debug_mode to camera
    void publishCameraCommand(state_t state, bool debug_mode)
    {
        exlcm::payload_cont_to_cam_msg_t msg;
        msg.cont_state = static_cast<int8_t>(state);
        msg.debug_mode = debug_mode;
        lcm_.publish(CH_CONT_TO_CAM, &msg);
        std::cout << "[INFO] Published to " << CH_CONT_TO_CAM
                  << ": cont_state=" << static_cast<int>(state)
                  << " debug_mode=" << debug_mode << std::endl;
    }
 
    // Block until cam_msg_t received or timeout_ms elapsed
    // Returns true if cam_status == true, false on error or timeout
    bool waitForCamStatus(int timeout_ms)
    {
        handler_.reset();
        auto start = std::chrono::steady_clock::now();
 
        while (!handler_.cam_status_received)
        {
            lcm_.handleTimeout(100); // poll every 100ms
 
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
 
 
// Main
int main()
{
    std::cout << "[INFO] Starting test payload controller..." << std::endl;
    TestPayloadController controller;
    controller.run();
    return 0;
}

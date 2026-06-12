#ifndef PAYLOAD_CONFIG_H
#define PAYLOAD_CONFIG_H

// #define TRAJECTORY_FILE_PATH         "data/docking_trajectory.csv"
// #define RESULTS_FILEPATH             "data/test_obc_nominal/results.csv"
// #define DEBUG_RESULTS_FILEPATH       "data/test_obc_debug/debug_mode_focus.jpeg"
// #define TRAJECTORY_SETTINGS_FILEPATH "data/test_obc_nominal/trajectory_settings.csv"
// #define CAMERA_SETTINGS_FILEPATH     "data/test_obc_nominal/scalar_settings.csv"

// Used for the UART hardware testing 
// #define TRAJECTORY_FILE_PATH         "data/docking_trajectory_short.csv"
// #define RESULTS_FILEPATH             "data/hardware_test/results.csv"
// #define DEBUG_RESULTS_FILEPATH       "data/hardware_test/debug_mode_focus.jpeg"
// #define TRAJECTORY_SETTINGS_FILEPATH "data/hardware_test/trajectory_settings.csv"
// #define CAMERA_SETTINGS_FILEPATH     "data/hardware_test/scalar_settings.csv"

// Nominal operation
// #define TRAJECTORY_FILE_PATH         "data/nominal/trajectory_settings.csv" // Payload controller reads the trajectory file from here
#define TRAJECTORY_FILE_PATH         "data/hardware_test/trajectory_settings.csv"
#define RESULTS_FILEPATH             "data/hardware_test/results.csv" // OBC bridge reads results file from here
#define DEBUG_RESULTS_FILEPATH       "data/nominal/debug_mode_focus.jpeg" // OBC bridge reads debug image from here
#define TRAJECTORY_SETTINGS_FILEPATH "data/nominal/trajectory_settings.csv" // OBC bridge writes trajectory here
#define CAMERA_SETTINGS_FILEPATH     "data/nominal/scalar_settings.csv" // OBC bridge writes camera settings here


#define NUM_TIMESTEPS 150

#endif

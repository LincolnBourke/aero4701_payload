# # Function-, camera-based program with LCM functions
# ##########################################################################################################################
## LIBRARIES
import cv2 as cv
import glob
import cameraHelper as h
import lcmHelper as lcm_h
import os

## PARAMETERS
CAMERA_SETTINGS_PATH = "apps/camera/camera_settings/pi_camera_settings.txt"
BOOT_COUNT_FILE      = "/home/slave2/Documents/service_test/boot_count.txt"

## FLAGS
SHOW_CAMERA_FEED    = False   # Display live camera feed widget during capture
SAVE_DEBUG_IMAGES   = True   # Save calibration_test, baseline_pose annotated images and event data
DELETE_IMAGES       = False  # Delete all saved images (calibration + experiment) after each run

## PAYLOAD CONTROLLER STATE ENUM
from enum import IntEnum

class PayloadState(IntEnum):
    IDLE                 = 0
    READ_TRAJECTORY      = 1
    CALIBRATE_SERVOS     = 2
    CALIBRATE_CAMERA     = 3
    DEPLOY               = 4
    RUNNING              = 5
    RETURN               = 6
    SAVE_RESULTS         = 7
    TERMINATE_RUN        = 8
    ERROR                = 9
    DEBUG                = 10


## Read boot count (read-only - the service script owns incrementing it)
def get_boot_count(counter_file):
    if os.path.exists(counter_file):
        with open(counter_file, "r") as f:
            return int(f.read().strip())
    return 0

## CAMERA CLASS
class Camera:
    """
    Manages camera state and experiment lifecycle.
    Instantiated once at startup; handler methods are called by the main loop
    in response to payload controller state messages over LCM.
    """

    def __init__(self):
        # Camera handle
        self.picam2               = None
        # Camera configuration
        self.params               = None
        self.saved_cam_settings   = None
        self.debug_mode           = True # change with flag from payload controller
        # Calibration results
        self.mtx                  = None
        self.dist                 = None
        # Chessboard data
        self.objpoints_3boards    = None
        self.ROIS                 = None
        # Output directories
        # TODO not really necessary but leave for now?
        self.output_dir           = output_dir
        self.calib_folder         = None
        self.baseline_folder      = None
        self.baseline_pose_folder = None
        # Results
        self.hist_records         = None

        # Dispatch table — maps PayloadState int8 values to handler methods
        self.state_handlers = {
            PayloadState.CALIBRATE_CAMERA:  self.handle_calibrate_cam,
            PayloadState.DEPLOY:            self.handle_deploy,
            PayloadState.RUNNING:           self.handle_running,
            PayloadState.SAVE_RESULTS:      self.handle_save_results,
            PayloadState.TERMINATE_RUN:     self.handle_terminate_run,
            PayloadState.ERROR:             self.handle_error,
            PayloadState.DEBUG:             self.handle_debug,
            PayloadState.IDLE:              None,
            PayloadState.READ_TRAJECTORY:   None,
            PayloadState.CALIBRATE_SERVOS:  None,
            PayloadState.RETURN:            None,
        }

    def handle_state(self, state, debug_mode):
        """Look up and call the handler for the given PayloadState int8 value."""
        self.debug_mode = debug_mode
        handler = self.state_handlers.get(state)
        # Run relevant method (if not None)
        if handler is not None:
            handler()

    ## STATE HANDLER METHODS

    def handle_calibrate_cam(self):
        """
        CALIBRATE_CAM: Focus and calibrate camera.
        Sets up directories and parameters, attempts camera calibration,
        saves settings, and publishes result to controller.
        """
        
        print("[INFO] STATE: Calibrate\n")
        # Setup directories
        # self.output_dir, self.calib_folder, self.baseline_folder, self.baseline_pose_folder = h.setup_directories()
        self.output_dir, self.calib_folder, self.baseline_folder, self.baseline_pose_folder = h.setup_directories(self.output_dir)

        # Calibration parameters
        CHESSBOARD, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS = h.setup_calib_parameters()

        # Ground truth chessboard corner coordinates
        self.objpoints_3boards = h.get_cboard_gt(L, CHESSBOARD, SQUARE_SIZE)

        # Define ROIs of chessboards for calibration
        # TODO fix settings on calibration
        self.ROIS = h.define_rois()
        # h.test_draw_rois(image_path="outputs/debug_mode_focus.jpeg", ROIS=self.ROIS)

        # Prepare for opening and calibrating camera
        CALIB_FLAG     = False
        CALIB_ATTEMPTS = 0
        picam2_        = None

        # Attempt calibration until successful or reached max attempts
        while not CALIB_FLAG and CALIB_ATTEMPTS < MAX_CALIB_ATTEMPTS:
            print("Starting calibration...\n")

            # Read parameters in from file
            self.params = h.prep_pi_cam_params(camera_file=CAMERA_SETTINGS_PATH)

            # Open camera and set focus
            # picam2_ = h.open_picam(self.params, picam2_, debug_mode=self.debug_mode)
            picam2_ = h.open_picam(self.params, picam2_, debug_mode=self.debug_mode, output_dir=self.output_dir)
            if picam2_ is None:
                print("Camera opening and focus failed\n")
                CALIB_ATTEMPTS += 1
                continue

            # Take short video and save to calibration folder
            # h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=5.0)
            h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=5.0, calib_folder=self.calib_folder)

            # Load images
            images = glob.glob(f"{self.calib_folder}/*.jpeg")
            if len(images) < 5:
                print("Not enough frames, retrying...\n")
                CALIB_ATTEMPTS += 1
                if picam2_ is not None:
                    picam2_ = h.close_camera(picam2_)
                continue

            # Detect chessboards
            # objpoints, imgpoints, img_size = h.detect_cboard_calib(images, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)
            objpoints, imgpoints, img_size = h.detect_cboard_calib(images, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES, debug_folder=f"{self.output_dir}/calibration_test")
            if len(objpoints) < 3:
                print("Not enough valid detections, retrying...")
                CALIB_ATTEMPTS += 1
                if picam2_ is not None:
                    picam2_ = h.close_camera(picam2_)
                continue

            # Calibrate camera
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)
            print("Camera matrix:\n", mtx)
            self.mtx   = mtx
            self.dist  = dist
            CALIB_FLAG = True
            CALIB_ATTEMPTS += 1

        # Successful calibration
        if CALIB_FLAG:
            # Save settings
            print("Camera calibration complete\n")
            self.saved_cam_settings = h.extract_applied_settings(picam2_)

            # Cleanly close camera
            h.close_camera(picam2_)
        # Unsuccessful calibration
        else:
            print("Calibration failed\n")
            if picam2_ is not None:
                h.close_camera(picam2_)

        # Publish calibration status to payload controller
        lcm_h.publish_cam_msg(cam_status=CALIB_FLAG)

    def handle_deploy(self):
        """
        DEPLOY: Open camera with calibrated settings ready for the experiment.
        Publishes OK to controller when camera is ready, or ERROR on failure.
        """
        print("[INFO] STATE: Deploy\n")
        
        # Reopen camera with saved settings
        picam2_ = h.open_picam_for_exp(self.params, self.picam2, self.saved_cam_settings)

        # Check camera opened ok
        if picam2_ is None:
            print("Camera opening and focus failed for experiment\n")
            lcm_h.publish_cam_msg(cam_status=False)
            return

        self.picam2 = picam2_

        # Publish ok to controller
        lcm_h.publish_cam_msg(cam_status=True)

    def handle_running(self):
        """
        RUNNING: Run the experiment (take video).
        Publishes ok to controller on success, or ERROR on failure.
        """
        print("[INFO] STATE: Running\n")
        # Run experiment
        self.hist_records = h.save_exp_video(self.picam2, display_widget=SHOW_CAMERA_FEED, save_debug_images=SAVE_DEBUG_IMAGES, exp_time=30.0, WINDOW_SIZE=1, output_dir=self.output_dir)

        # Check experiment was ok
        if self.hist_records is None:
            print("Experiment failed\n")
            lcm_h.publish_cam_msg(cam_status=False)
            return


    def handle_save_results(self):
        """
        SAVE_RESULTS: Baseline processing, save experiment data, cleanup.
        Publishes OK to controller when done.
        """
        print("[INFO] STATE: Save Results\n")
        # Cycle through images in baseline, estimate poses, save to file
        # h.process_baseline_data(self.objpoints_3boards, self.mtx, self.dist, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)
        h.process_baseline_data(
            self.objpoints_3boards, self.mtx, self.dist, self.ROIS,
            save_debug_images=SAVE_DEBUG_IMAGES,
            baseline_folder=self.baseline_folder,
            pose_folder=self.baseline_pose_folder,
            results_dir=f"{self.output_dir}/experiment_results"
        )

        # Save histogram results
        # h.save_exp_results(self.hist_records)
        h.save_exp_results(self.hist_records, results_dir=f"{self.output_dir}/experiment_results")
        
        # Turn off camera
        if self.picam2 is not None:
            h.close_camera(self.picam2)
            self.picam2 = None

        # Cleanup
        # h.cleanup_images(DELETE_IMAGES)
        h.cleanup_images(DELETE_IMAGES, output_dir=self.output_dir)

        # Publish ok to controller
        lcm_h.publish_cam_msg(cam_status=True)
        print("\nExperiment complete. Waiting for next calibration trigger...\n")

    def handle_terminate_run(self):
        """
        TERMINATE_RUN: Turn off camera cleanly.
        Publishes OK to controller when done.
        """
        print("[INFO] STATE: Terminate Run\n")
        # Turn off camera
        if self.picam2 is not None:
            h.close_camera(self.picam2)
            self.picam2 = None

        lcm_h.publish_cam_msg(cam_status=True)

    def handle_error(self):
        """
        ERROR: Received error from controller.
        Turn off camera if open.
        """
        print("[INFO] STATE: Error\n")
        # Turn off camera
        if self.picam2 is not None:
            h.close_camera(self.picam2)
            self.picam2 = None

    def handle_debug(self):
        """
        DEBUG: Focus camera and save debug image.
        """
        
        print("[INFO] STATE: DEBUG\n")

        # Read parameters in from file
        self.params = h.prep_pi_cam_params(camera_file=CAMERA_SETTINGS_PATH)

        picam2_ = None

        # Open camera, set focus and take debug image
        picam2_ = h.open_picam(self.params, picam2_, debug_mode=True, output_dir="data/test_obc_debug")
        if picam2_ is None:
            print("[WARN] Camera opening and focus failed\n")
            lcm_h.publish_cam_msg(cam_status=False)

        # Otherwise (success case) close camera and publish success
        if picam2_ is not None:
            h.close_camera(picam2_)

        # Publish calibration status to payload controller
        lcm_h.publish_cam_msg(cam_status=True)
        
## MAIN

print("Starting camera program...")

# Added for startup functionality
boot_count = get_boot_count(BOOT_COUNT_FILE)
output_dir = f"outputs/boot_{boot_count:03d}"
print(f"[INFO] Boot count: {boot_count}, output dir: {output_dir}")

camera = Camera()

# Waits for state messages from the payload controller and dispatches to the
# appropriate handler method. Controller drives all transitions.
while True:
    msg = lcm_h.wait_for_payload_comp_msg()
    camera.handle_state(msg.cont_state, msg.debug_mode)


#########################
# ~ # Checking reprojection error after calibration
# ~ # h.check_repoj_error(objpoints, rvecs, tvecs, mtx, dist, imgpoints)


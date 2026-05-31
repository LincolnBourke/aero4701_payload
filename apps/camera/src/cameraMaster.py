# # Function-, camera-based program with LCM functions
# ##########################################################################################################################
## LIBRARIES
import cv2 as cv
import glob
import cameraHelper as h
import lcmHelper as lcm_h

# TODO put helper functions into class definition?
# TODO make header for class definition?

## FLAGS
SHOW_CAMERA_FEED    = True   # Display live camera feed widget during capture
SAVE_DEBUG_IMAGES   = True   # Save calibration_test, baseline_pose annotated images and event data
DELETE_IMAGES       = False  # Delete all saved images (calibration + experiment) after each run

# TODO check on integration
## PAYLOAD CONTROLLER STATE ENUM
from enum import IntEnum

class PayloadState(IntEnum):
    IDLE          = 0
    SETUP         = 1
    CALIBRATE_CAM = 2
    DEPLOY        = 3
    RUNNING       = 4
    SAVE_RESULTS  = 5
    TERMINATE_RUN = 6
    ERROR         = 7


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
        self.debug_mode           = False
        # Calibration results
        self.mtx                  = None
        self.dist                 = None
        # Chessboard data
        self.objpoints_3boards    = None
        self.ROIS                 = None
        # Output directories
        # TODO not really necessary but leave for now?
        self.output_dir           = None
        self.calib_folder         = None
        self.baseline_folder      = None
        self.baseline_pose_folder = None
        # Results
        self.hist_records         = None

        # Dispatch table — maps PayloadState int8 values to handler methods
        self.state_handlers = {
            PayloadState.CALIBRATE_CAM:  self.handle_calibrate_cam,
            PayloadState.DEPLOY:         self.handle_deploy,
            PayloadState.RUNNING:        self.handle_running,
            PayloadState.SAVE_RESULTS:   self.handle_save_results,
            PayloadState.TERMINATE_RUN:  self.handle_terminate_run,
            PayloadState.ERROR:          self.handle_error,
            PayloadState.IDLE:           None,
            PayloadState.SETUP:          None,
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
        # TODO not really necessary but leave for now?
        self.output_dir, self.calib_folder, self.baseline_folder, self.baseline_pose_folder = h.setup_directories()

        # Calibration parameters
        # TODO Delete MAX_BOARDS
        # TODO fix settings
        CHESSBOARD, MAX_BOARDS, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS = h.setup_calib_parameters()

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
            self.params = h.prep_pi_cam_params()

            # Open camera and set focus
            picam2_ = h.open_picam(self.params, picam2_, debug_mode=self.debug_mode)
            if picam2_ is None:
                print("Camera opening and focus failed\n")
                CALIB_ATTEMPTS += 1
                continue

            # Take short video and save to calibration folder
            h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=20.0)

            # Load images
            images = glob.glob("outputs/calibration/*.jpeg")
            if len(images) < 5:
                print("Not enough frames, retrying...\n")
                CALIB_ATTEMPTS += 1
                if picam2_ is not None:
                    picam2_ = h.close_camera(picam2_)
                continue

            # Detect chessboards
            objpoints, imgpoints, img_size = h.detect_cboard_calib(images, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)
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

            # Cleanly close calibration camera
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
        self.hist_records = h.save_exp_video(self.picam2, display_widget=SHOW_CAMERA_FEED, save_debug_images=SAVE_DEBUG_IMAGES, exp_time=30.0, WINDOW_SIZE=1)

        # Check experiment was ok
        if self.hist_records is None:
            print("Experiment failed\n")
            lcm_h.publish_cam_msg(cam_status=False)
            return

        # Publish ok to controller
        lcm_h.publish_cam_msg(cam_status=True)

    def handle_save_results(self):
        """
        SAVE_RESULTS: Baseline processing, save experiment data, cleanup.
        Publishes OK to controller when done.
        """
        print("[INFO] STATE: Save Results\n")
        # Cycle through images in baseline, estimate poses, save to file
        h.process_baseline_data(self.objpoints_3boards, self.mtx, self.dist, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)

        # Save histogram results
        h.save_exp_results(self.hist_records)
        
        # Turn off camera
        if self.picam2 is not None:
            h.close_camera(self.picam2)
            self.picam2 = None

        # Cleanup
        h.cleanup_images(DELETE_IMAGES)

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


## MAIN

print("Starting camera program...")

camera = Camera()

# Waits for state messages from the payload controller and dispatches to the
# appropriate handler method. Controller drives all transitions.
while True:
    msg = lcm_h.wait_for_payload_comp_msg()
    camera.handle_state(msg.cont_state, msg.debug_mode)


#########################
# ~ # Checking reprojection error after calibration
# ~ # h.check_repoj_error(objpoints, rvecs, tvecs, mtx, dist, imgpoints)
















# # # Function-based program with camera program with simplified main
# # ##########################################################################################################################
# ## LIBRARIES
# import cv2 as cv
# import glob
# import cameraHelper as h
# import lcm_helper as lcm_h

# # TODO put helper functions into class definition
# # TODO make header for class definition

# ## FLAGS
# SHOW_CAMERA_FEED    = True   # Display live camera feed widget during capture
# SAVE_DEBUG_IMAGES   = True   # Save calibration_test, baseline_pose annotated images and event data
# DELETE_IMAGES       = False  # Delete all saved images (calibration + experiment) after each run
# # TODO remove and read off from LCM message 
# DEBUG_MODE          = True   # Save a focused frame to outputs/ after autofocus

# # TODO check on integration
# ## PAYLOAD CONTROLLER STATE ENUM
# from enum import IntEnum

# class PayloadState(IntEnum):
#     IDLE          = 1
#     SETUP         = 2
#     CALIBRATE_CAM = 3
#     DEPLOY        = 4
#     RUNNING       = 5
#     SAVE_RESULTS  = 6
#     TERMINATE_RUN = 7
#     ERROR         = 8


# ## CAMERA CLASS
# class Camera:
#     """
#     Manages camera state and experiment lifecycle.
#     Instantiated once at startup; handler methods are called by the main loop
#     in response to payload controller state messages over LCM.
#     """

#     def __init__(self):
#         # Camera handle
#         self.picam2               = None
#         # Camera configuration
#         self.params               = None
#         self.saved_cam_settings   = None
#         # Calibration results
#         self.mtx                  = None
#         self.dist                 = None
#         # Chessboard data
#         self.objpoints_3boards    = None
#         self.ROIS                 = None
#         # Output directories
#         # TODO not really necessary but leave for now?
#         self.output_dir           = None
#         self.calib_folder         = None
#         self.baseline_folder      = None
#         self.baseline_pose_folder = None
#         # Results
#         self.hist_records         = None

#         # Dispatch table — maps PayloadState int8 values to handler methods
#         self.state_handlers = {
#             PayloadState.CALIBRATE_CAM:  self.handle_calibrate_cam,
#             PayloadState.DEPLOY:         self.handle_deploy,
#             PayloadState.RUNNING:        self.handle_running,
#             PayloadState.SAVE_RESULTS:   self.handle_save_results,
#             PayloadState.TERMINATE_RUN:  self.handle_terminate_run,
#             PayloadState.ERROR:          self.handle_error,
#             PayloadState.IDLE:           None,
#             PayloadState.SETUP:          None,
#         }

#     # TODO add debug mode
#     def handle_state(self, state):
#         """Look up and call the handler for the given PayloadState int8 value."""
#         handler = self.state_handlers.get(state)
#         # Run relevant method (if not None)
#         if handler is not None:
#             handler()

#     ## STATE HANDLER METHODS

#     def handle_calibrate_cam(self):
#         """
#         CALIBRATE_CAM: Focus and calibrate camera.
#         Sets up directories and parameters, attempts camera calibration,
#         saves settings, and publishes result to controller.
#         """
#         # Setup directories
#         # TODO not really necessary but leave for now?
#         self.output_dir, self.calib_folder, self.baseline_folder, self.baseline_pose_folder = h.setup_directories()

#         # Calibration parameters
#         # TODO Delete MAX_BOARDS
#         # TODO fix settings
#         CHESSBOARD, MAX_BOARDS, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS = h.setup_calib_parameters()

#         # Ground truth chessboard corner coordinates
#         self.objpoints_3boards = h.get_cboard_gt(L, CHESSBOARD, SQUARE_SIZE)

#         # Define ROIs of chessboards for calibration
#         # TODO fix settings on calibration
#         self.ROIS = h.define_rois()
#         # h.test_draw_rois(image_path="outputs/debug_mode_focus.jpeg", ROIS=self.ROIS)

#         # Prepare for opening and calibrating camera
#         CALIB_FLAG     = False
#         CALIB_ATTEMPTS = 0
#         picam2_        = None

#         # Attempt calibration until successful or reached max attempts
#         while not CALIB_FLAG and CALIB_ATTEMPTS < MAX_CALIB_ATTEMPTS:
#             print("Starting calibration...\n")

#             # Read parameters in from file
#             self.params = h.prep_pi_cam_params()

#             # Open camera and set focus
#             # TODO read debug mode from LCM message 
#             picam2_ = h.open_picam(self.params, picam2_, debug_mode=DEBUG_MODE)
#             if picam2_ is None:
#                 print("Camera opening and focus failed\n")
#                 CALIB_ATTEMPTS += 1
#                 continue

#             # Take short video and save to calibration folder
#             h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=20.0)

#             # Load images
#             images = glob.glob("outputs/calibration/*.jpeg")
#             if len(images) < 5:
#                 print("Not enough frames, retrying...\n")
#                 CALIB_ATTEMPTS += 1
#                 if picam2_ is not None:
#                     picam2_ = h.close_camera(picam2_)
#                 continue

#             # Detect chessboards
#             objpoints, imgpoints, img_size = h.detect_cboard_calib(images, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)
#             if len(objpoints) < 3:
#                 print("Not enough valid detections, retrying...")
#                 CALIB_ATTEMPTS += 1
#                 if picam2_ is not None:
#                     picam2_ = h.close_camera(picam2_)
#                 continue

#             # Calibrate camera
#             ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)
#             print("Camera matrix:\n", mtx)
#             self.mtx   = mtx
#             self.dist  = dist
#             CALIB_FLAG = True
#             CALIB_ATTEMPTS += 1

#         # Successful calibration
#         if CALIB_FLAG:
#             # Save settings
#             print("Camera calibration complete\n")
#             self.saved_cam_settings = h.extract_applied_settings(picam2_)

#             # Cleanly close calibration camera
#             h.close_camera(picam2_)
#         # Unsuccessful calibration
#         else: 
#             print("Calibration failed\n")
#             if picam2_ is not None:
#                 h.close_camera(picam2_)

#         # Publish calibration status to payload controller
#         # TODO fix publishing
#         lcm_h.publish_cam_msg(cam_calib_complete=CALIB_FLAG)

#     def handle_deploy(self):
#         """
#         DEPLOY: Open camera with calibrated settings ready for the experiment.
#         Publishes OK to controller when camera is ready, or ERROR on failure.
#         """
#         # Reopen camera with saved settings
#         picam2_ = h.open_picam_for_exp(self.params, self.picam2, self.saved_cam_settings)

#         # Check camera opened ok
#         if picam2_ is None:
#             print("Camera opening and focus failed for experiment\n")
#             # TODO fix publishing
#             lcm_h.publish_cam_msg(exp_complete=False)
#             return

#         self.picam2 = picam2_

#         # Publish ok to controller
#         # TODO fix publishing
#         lcm_h.publish_cam_msg(deploy_complete=True)

#     def handle_running(self):
#         """
#         RUNNING: Run the experiment (take video).
#         Publishes ok to controller on success, or ERROR on failure.
#         """
#         # Run experiment
#         self.hist_records = h.save_exp_video(self.picam2, display_widget=False, save_debug_images=False, exp_time=20.0, WINDOW_SIZE=1)
#         # exp_success = h.save_exp_video(self.picam2, display_widget=SHOW_CAMERA_FEED, save_debug_images=SAVE_DEBUG_IMAGES, exp_time=20.0)

#         # Check experiment was ok
#         if self.hist_records is None:
#             print("Experiment failed\n")
#             # TODO fix publishing
#             lcm_h.publish_cam_msg(exp_complete=False)
#             return

#         # Publish ok to controller
#         # TODO fix publishing
#         lcm_h.publish_cam_msg(running_complete=True)

#     def handle_save_results(self):
#         """
#         SAVE_RESULTS: Baseline processing, save experiment data, cleanup.
#         Publishes OK to controller when done.
#         """
#         # Cycle through images in baseline, estimate poses, save to file
#         h.process_baseline_data(self.objpoints_3boards, self.mtx, self.dist, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES)

#         # Save histogram results
#         h.save_exp_results(self.hist_records)

#         # Cleanup
#         h.cleanup_images(DELETE_IMAGES)

#         # Publish ok to controller
#         # TODO Fix publishing
#         lcm_h.publish_cam_msg(exp_complete=True)
#         print("\nExperiment complete. Waiting for next calibration trigger...\n")

#     def handle_terminate_run(self):
#         """
#         TERMINATE_RUN: Turn off camera cleanly.
#         Publishes OK to controller when done.
#         """
#         # Turn off camera
#         if self.picam2 is not None:
#             h.close_camera(self.picam2)
#             self.picam2 = None

#         # TODO fix publishing
#         lcm_h.publish_cam_msg(terminate_complete=True)

#     def handle_error(self):
#         """
#         ERROR: Received error from controller.
#         Turn off camera if open.
#         """
#         # Turn off camera
#         if self.picam2 is not None:
#             h.close_camera(self.picam2)
#             self.picam2 = None


# ## MAIN

# print("Starting camera program...")

# camera = Camera()

# # Waits for state messages from the payload controller and dispatches to the
# # appropriate handler method. Controller drives all transitions.
# while True:
#     msg   = lcm_h.wait_for_payload_comp_msg()
#     state = msg.state 

#     camera.handle_state(state)


#########################
# ~ # Checking reprojection error after calibration
# ~ # h.check_repoj_error(objpoints, rvecs, tvecs, mtx, dist, imgpoints)







# # Function-based program with camera program
# ##########################################################################################################################
# ## LIBRARIES
# import cv2 as cv
# import glob
# import cameraHelper as h
# import lcm_helper as lcm_h


# ## FLAGS
# SHOW_CAMERA_FEED    = True   # Display live camera feed widget during capture
# SAVE_DEBUG_IMAGES   = True   # Save calibration_test, baseline_pose annotated images and event data
# DELETE_IMAGES       = False  # Delete all saved images (calibration + experiment) after each run
# DEBUG_MODE          = True   # Save a focused frame to outputs/ after autofocus


# ## CAMERA CLASS

# class Camera:
#     """
#     Manages camera state and experiment lifecycle.
#     Instantiated once at startup; handler methods are called by the main loop
#     in response to payload controller state messages over LCM.
#     """

#     def __init__(self):
#         # Camera handle
#         self.picam2               = None
#         # Camera configuration
#         self.params               = None
#         self.saved_cam_settings   = None
#         # Calibration results
#         self.mtx                  = None
#         self.dist                 = None
#         # Chessboard data
#         self.objpoints_3boards    = None
#         self.ROIS                 = None
#         # Output directories
#         self.output_dir           = None
#         self.calib_folder         = None
#         self.baseline_folder      = None
#         self.baseline_pose_folder = None

#     ## STATE HANDLER METHODS

#     def handle_calibrate_cam(self):
#         """
#         CALIBRATE_CAM: Focus and calibrate camera.
#         Sets up directories and parameters, attempts camera calibration,
#         saves settings, and publishes result to controller.
#         """
#         # Setup directories
#         (
#             self.output_dir,
#             self.calib_folder,
#             self.baseline_folder,
#             self.baseline_pose_folder,
#         ) = h.setup_directories()

#         # Calibration parameters
#         CHESSBOARD, MAX_BOARDS, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS = h.setup_calib_parameters()

#         # Ground truth chessboard corner coordinates
#         self.objpoints_3boards = h.get_cboard_gt()

#         # Define ROIs of chessboards for calibration
#         self.ROIS = h.define_rois()
#         # h.test_draw_rois(image_path="outputs/debug_mode_focus.jpeg", ROIS=self.ROIS)

#         # Prepare for opening/calibrating camera
#         CALIB_FLAG     = False
#         CALIB_ATTEMPTS = 0
#         picam2_        = None

#         # Attempt calibration until successful or reached max attempts
#         while not CALIB_FLAG and CALIB_ATTEMPTS < MAX_CALIB_ATTEMPTS:
#             print("Starting calibration...\n")

#             # Read parameters in from file
#             self.params = h.prep_pi_cam_params()

#             # Open camera and set focus
#             picam2_ = h.open_picam(self.params, picam2_, debug_mode=DEBUG_MODE)
#             if picam2_ is None:
#                 print("Camera opening and focus failed\n")
#                 CALIB_ATTEMPTS += 1
#                 continue

#             # Take short video and save to calibration folder
#             h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=20.0)

#             # Load images
#             images = glob.glob("outputs/calibration/*.jpeg")
#             if len(images) < 5:
#                 print("Not enough frames, retrying...\n")
#                 CALIB_ATTEMPTS += 1
#                 if picam2_ is not None:
#                     picam2_ = h.close_camera(picam2_)
#                 continue

#             # Detect chessboards
#             objpoints, imgpoints, img_size = h.detect_cboard_calib(
#                 images, self.ROIS, save_debug_images=SAVE_DEBUG_IMAGES
#             )
#             if len(objpoints) < 3:
#                 print("Not enough valid detections, retrying...")
#                 CALIB_ATTEMPTS += 1
#                 if picam2_ is not None:
#                     picam2_ = h.close_camera(picam2_)
#                 continue

#             # Calibrate camera
#             ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
#                 objpoints, imgpoints, img_size, None, None
#             )
#             print("Camera matrix:\n", mtx)
#             self.mtx   = mtx
#             self.dist  = dist
#             CALIB_FLAG = True
#             CALIB_ATTEMPTS += 1

#         # Check calibration worked ok
#         if not CALIB_FLAG:
#             print("Calibration failed\n")
#             if picam2_ is not None:
#                 h.close_camera(picam2_)
#                 self.picam2 = None
#             lcm_h.publish_cam_msg(cam_calib_complete=False)
#             return

#         # Save calibration settings
#         print("Camera calibration complete\n")
#         self.saved_cam_settings = h.extract_applied_settings(picam2_)

#         # Cleanly close calibration camera
#         h.close_camera(picam2_)
#         self.picam2 = None

#         # Tell payload computer camera calibration status
#         lcm_h.publish_cam_msg(cam_calib_complete=CALIB_FLAG)

#     def handle_deploy(self):
#         """
#         DEPLOY: Open camera with calibrated settings ready for the experiment.
#         Publishes OK to controller when camera is ready, or ERROR on failure.
#         """
#         # Reopen camera with saved settings
#         picam2_ = h.open_picam_for_exp(self.params, self.picam2, self.saved_cam_settings)

#         # Check camera opened ok
#         if picam2_ is None:
#             print("Camera opening and focus failed for experiment\n")
#             lcm_h.publish_cam_msg(exp_complete=False)
#             return

#         self.picam2 = picam2_

#         # Publish OK to controller
#         lcm_h.publish_cam_msg(deploy_complete=True)

#     def handle_running(self):
#         """
#         RUNNING: Run the experiment (take video).
#         Publishes OK to controller on success, or ERROR on failure.
#         """
#         # Run experiment
#         exp_success = h.save_exp_video(
#             self.picam2,
#             display_widget=SHOW_CAMERA_FEED,
#             save_debug_images=SAVE_DEBUG_IMAGES,
#             exp_time=20.0,
#         )

#         # Note: for testing
#         exp_success = True

#         # Check experiment was ok
#         if exp_success is None:
#             print("Experiment failed\n")
#             lcm_h.publish_cam_msg(exp_complete=False)
#             return

#         # Publish OK to controller
#         lcm_h.publish_cam_msg(running_complete=True)

#     def handle_save_results(self):
#         """
#         SAVE_RESULTS: Baseline processing, save experiment data, cleanup.
#         Publishes OK to controller when done.
#         """
#         # Cycle through images in baseline/, estimate poses, save to file
#         h.process_baseline_data(
#             self.objpoints_3boards,
#             self.mtx,
#             self.dist,
#             self.ROIS,
#             save_debug_images=SAVE_DEBUG_IMAGES,
#         )

#         # TODO: save other results

#         # Cleanup
#         h.cleanup_images(DELETE_IMAGES)

#         # Publish OK to controller
#         lcm_h.publish_cam_msg(exp_complete=True)
#         print("\nExperiment complete. Waiting for next calibration trigger...\n")

#     def handle_terminate_run(self):
#         """
#         TERMINATE_RUN: Turn off camera cleanly.
#         Publishes OK to controller when done.
#         """
#         # TODO: turn off camera
#         lcm_h.publish_cam_msg(terminate_complete=True)

#     def handle_error(self):
#         """
#         ERROR: Received error from controller.
#         Turn off camera if open.
#         """
#         # TODO: turn off camera (if not None)
#         pass


# ## MAIN

# print("Starting camera program...")

# camera = Camera()

# # State dispatch table — maps controller state strings to handler methods
# STATE_HANDLERS = {
#     "CALIBRATE_CAM":  camera.handle_calibrate_cam,
#     "DEPLOY":         camera.handle_deploy,
#     "RUNNING":        camera.handle_running,
#     "SAVE_RESULTS":   camera.handle_save_results,
#     "TERMINATE_RUN":  camera.handle_terminate_run,
#     "ERROR":          camera.handle_error,
#     # IDLE and SETUP require no camera action
#     "IDLE":           None,
#     "SETUP":          None,
# }

# # Waits for state messages from the payload controller and dispatches to the
# # appropriate handler method. Controller drives all transitions.
# while True:
#     msg   = lcm_h.wait_for_payload_comp_msg()
#     state = msg.state  # Expected: string matching a key in STATE_HANDLERS

#     handler = STATE_HANDLERS.get(state)

#     if handler is None:
#         # IDLE, SETUP, or unrecognised state — no camera action required
#         continue

#     handler()


# #########################
# # ~ # Checking reprojection error after calibration
# # ~ # h.check_repoj_error(objpoints, rvecs, tvecs, mtx, dist, imgpoints)






# Original + reorganisation comments
###########################################################################################################################################
# ## RUN INSTRUCTIONS
# # source venv/bin/activate

# # TODO change debug focus from a flag to an lcm message 
# # TODO check directories setup 
# # TODO update exerpiment output results
# # TODO update camera settings file 

# # TODO make sure payload comp focuses at mid range
# # TODO update chessboard layout to new configuration for right pose estimates 
# # TODO put flag if statements inside functions not out here 
# # TODO use directory names 

# ## LIBRARIES
# import cv2 as cv
# import glob
# import cameraHelper as h
# import lcm_helper as lcm_h


# ## FLAGS
# SHOW_CAMERA_FEED    = True   # Display live camera feed widget during capture
# SAVE_DEBUG_IMAGES   = True   # Save calibration_test, baseline_pose annotated images and event data
# DELETE_IMAGES       = False  # Delete all saved images (calibration + experiment) after each run
# DEBUG_MODE         = True   # Save a focused frame to outputs/ after autofocus

# # Logging message
# print("Starting camera program...")


# ## OUTER EXPERIMENT LOOP
# # Waits for calibration to be enabled, runs calibration + experiment, then loops for the next experiment
# while True:

#     ## CALIBRATE_CAM START ######################################
#     # Stop program until payload comp msg received and camera is enabled
#     start_calib_cam = False
#     while start_calib_cam == False:
#         msg = lcm_h.wait_for_payload_comp_msg()
#         start_calib_cam = msg.cam_enabled

#     # Setup directories
#     output_dir, calib_folder, baseline_folder, baseline_pose_folder = h.setup_directories()

#     # Calibration parameters
#     CHESSBOARD, MAX_BOARDS, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS = h.setup_calib_parameters()

#     # Ground truth chessboard corner coordinates
#     objpoints_3boards = h.get_cboard_gt()

#     # Define ROIS of chessboards for calibration
#     ROIS = h.define_rois()
#     # h.test_draw_rois(image_path="outputs/debug_mode_focus.jpeg", ROIS=ROIS)

#     # Prepare for opening/calibrating camera
#     CALIB_FLAG = False
#     CALIB_ATTEMPTS = 0
#     picam2_ = None

#     # Attempt calibration until successful or reached max attempts
#     while not CALIB_FLAG and CALIB_ATTEMPTS < MAX_CALIB_ATTEMPTS:
#         print("Starting calibration...\n")

#         # Read parameters in from file
#         params = h.prep_pi_cam_params()

#         # Open camera and set focus
#         picam2_ = h.open_picam(params, picam2_, debug_mode=DEBUG_MODE)
#         if picam2_ == None:
#             print("Camera opening and focus failed\n")
#             CALIB_ATTEMPTS += 1
#             continue

#         # Take short video and save to calibration folder
#         h.save_calib_video_picam(picam2_, SHOW_CAMERA_FEED, calib_time=20.0)

#         # Load images
#         images = glob.glob(f"outputs/calibration/*.jpeg")
#         if len(images) < 5:
#             print("Not enough frames, retrying...\n")
#             CALIB_ATTEMPTS += 1
#             if picam2_ is not None:
#                 picam2_ = h.close_camera(picam2_)
#             continue

#         # Detect chessboards
#         objpoints, imgpoints, img_size = h.detect_cboard_calib(images, ROIS, save_debug_images=SAVE_DEBUG_IMAGES)
#         if len(objpoints) < 3:
#             print("Not enough valid detections, retrying...")
#             CALIB_ATTEMPTS += 1
#             if picam2_ is not None:
#                 picam2_ = h.close_camera(picam2_)
#             continue

#         # Calibrate camera
#         ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)

#         print("Camera matrix:\n", mtx)

#         CALIB_FLAG = True
#         CALIB_ATTEMPTS += 1
    
#     # Check calibration worked ok
#     if CALIB_FLAG == False:
#         print("Calibration failed\n")
#         if picam2_ is not None:
#             picam2_ = h.close_camera(picam2_)
#         continue

#     # Save calibration settings
#     print("Camera calibration complete\n")
#     saved_cam_settings = h.extract_applied_settings(picam2_)

#     # Cleanly close calibration camera
#     picam2_ = h.close_camera(picam2_)

#     # Tell payload computer camera calibration status
#     lcm_h.publish_cam_msg(cam_calib_complete=CALIB_FLAG)

#     ## CALIBRATE_CAM END ######################################


#     # Wait for payload computer confirmation to start experiment
#     start_exp = False
#     while start_exp == False:
#         msg = lcm_h.wait_for_payload_comp_msg()
#         start_exp = msg.exp_enabled

#     ## DEPLOY START ######################################
#     # Reopen camera with saved settings
#     picam2_ = h.open_picam_for_exp(params, picam2_, saved_cam_settings)

#     # Check camera opened ok
#     if picam2_ is None:
#         print("Camera opening and focus failed for experiment\n")
#         lcm_h.publish_cam_msg(exp_complete=False)
#         continue
    
#     ## DEPLOY END ######################################

#     ## RUNNING START ######################################
#     # Run experiment
#     exp_success = h.save_exp_video(picam2_, display_widget = SHOW_CAMERA_FEED, save_debug_images = SAVE_DEBUG_IMAGES, exp_time=20.0)

#     # Note: for testing
#     exp_success = True

#     # Check experiment was ok
#     if exp_success == None:
#         print("Experiment failed\n")
#         lcm_h.publish_cam_msg(exp_complete=False)
#         continue

#     ## RUNNING END ######################################


#     ## SAVE_RESULTS START ######################################
#     # Cycle through images in baseline/, estimate poses, save to file
#     h.process_baseline_data(objpoints_3boards, mtx, dist, ROIS, save_debug_images=SAVE_DEBUG_IMAGES)

#     # TODO save other results

#     ## Cleanup
#     h.cleanup_images(DELETE_IMAGES)

#     ## SAVE_RESULTS END ######################################

#     ## TERMINATE_RUN START ######################################
#     # TODO turn off camera

#     ## TERMINATE_RUN END ######################################

#     ## ERROR START ######################################
#     # TODO turn off camera

#     ## ERROR END ######################################




#     ## FINISH
#     # Tell payload computer experiment is finished
#     lcm_h.publish_cam_msg(exp_complete=True)

#     print("\nExperiment complete. Waiting for next calibration trigger...\n")



# #########################
# # ~ # Checking reprojection error after calibration
# # ~ # h.check_repoj_error(objpoints, rvecs, tvecs, mtx, dist, imgpoints)


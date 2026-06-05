import numpy as np
import cv2 as cv
import time
import shutil
import os
import glob
import csv


from picamera2 import Picamera2
from libcamera import controls
from event_camera_emulation.emulator import EventCameraEmulator

# Setup save directories
# ~ def setup_directories():
    # ~ output_dir = "outputs"
    # ~ os.makedirs(output_dir, exist_ok=True)
    # ~ calib_folder = "outputs/calibration"
    # ~ os.makedirs(calib_folder, exist_ok=True)
    # ~ baseline_folder = "outputs/baseline"
    # ~ os.makedirs(baseline_folder, exist_ok=True)
    # ~ baseline_pose_folder = "outputs/baseline_pose"
    # ~ os.makedirs(baseline_pose_folder, exist_ok=True)

    # ~ return output_dir, calib_folder, baseline_folder, baseline_pose_folder
    
def setup_directories(output_dir="outputs"):
    os.makedirs(output_dir, exist_ok=True)
    calib_folder = f"{output_dir}/calibration"
    baseline_folder = f"{output_dir}/baseline"
    baseline_pose_folder = f"{output_dir}/baseline_pose"
    os.makedirs(calib_folder, exist_ok=True)
    os.makedirs(baseline_folder, exist_ok=True)
    os.makedirs(baseline_pose_folder, exist_ok=True)
    
    return output_dir, calib_folder, baseline_folder, baseline_pose_folder

# Define board and other parameters
def setup_calib_parameters():
    CHESSBOARD = (5, 3)
    SQUARE_SIZE = 0.00225       # in metres
    L = 0.025981                # distance between centres of checkerboards, for triangular positions of boards
    MAX_CALIB_ATTEMPTS = 3
    
    return CHESSBOARD, SQUARE_SIZE, L, MAX_CALIB_ATTEMPTS

# Define ROIS centred in the image
# TODO Could avoid hardcoding, or update
# ~ def define_rois(width=640, height=480, scale=0.5):
    # ~ # Scale controls ROI size relative to image size
    # ~ w = int(width * scale)
    # ~ h = int(height * scale)

    # ~ cx = width // 2
    # ~ cy = height // 2

    # ~ # Offsets for triangular layout
    # ~ dx = int(width * 0.2)
    # ~ dy = int(height * 0.2)

    # ~ top_extra = int(height * 0.1)   # extra upward movement
    # ~ bottom_extra = int(height * 0.1)   # extra downward movement

    # ~ ROIS = [
        # ~ # top board
        # ~ (cx - w//2, cy - h//2 - dy - top_extra,
         # ~ cx + w//2, cy + h//2 - dy - top_extra),

        # ~ # bottom-left board
        # ~ (cx - w//2 - dx, cy - h//2 + dy + bottom_extra,
         # ~ cx + w//2 - dx, cy + h//2 + dy + bottom_extra),

        # ~ # bottom-right board
        # ~ (cx - w//2 + dx, cy - h//2 + dy + bottom_extra,
         # ~ cx + w//2 + dx, cy + h//2 + dy + bottom_extra),
    # ~ ]

    # ~ return ROIS
    
def define_rois():
    """Hardcoded ROIs tuned for 640x480."""
    return [
        (160,  11, 480, 266),  # Board 0 (top)
        ( 32, 194, 377, 484),  # Board 1 (bot-left)
        (258, 189, 608, 479),  # Board 2 (bot-right)
    ]


# Test function to draw on ROIS onto image for validation
def test_draw_rois(image_path, ROIS, output_dir="outputs", name="roi_debug.png"): 

    img = cv.imread(image_path)
    
    h, w = img.shape[:2]
    print(h, w)

    if img is None:
        raise ValueError(f"Could not read image: {image_path}")

    vis = img.copy()

    for i, roi in enumerate(ROIS):
        x1, y1, x2, y2 = roi

        # draw rectangle
        cv.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 3)

        # label
        cv.putText(vis, f"Board {i}", (x1 + 10, y1 + 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)

    out_path = os.path.join(output_dir, name)
    cv.imwrite(out_path, vis)

    print(f"Saved ROI test visualisation to: {out_path}")

# Rotation matrix to rotate checkerboard coordinates for triangular positioning
def rotation_towards(origin):
    direction = origin / np.linalg.norm(origin)
    z = np.array([0, 0, 1])

    # Align board normal with outward vector
    v = np.cross(z, direction)
    s = np.linalg.norm(v)
    c = np.dot(z, direction)

    if s == 0:
        return np.eye(3)

    vx = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

    R = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
    return R

# Extract checkerboard ground truth coordinates
def get_cboard_gt(L=0.025981, CHESSBOARD=(5, 3), SQUARE_SIZE=0.00225):
    # Define 3 chessboard corner coordinates 
    h = np.sqrt(3)/2 * L

    board_centres = np.array([
        [0,  2*h/3, 0],        # top
        [-L/2, -h/3, 0],       # bottom-left
        [ L/2, -h/3, 0]        # bottom-right
    ])

    # Define ground truth corner coordinates for 3 boards 
    objpoints_3boards = []
    grid = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
    grid = np.hstack([grid * SQUARE_SIZE, np.zeros((grid.shape[0], 1))])

    for centre in board_centres:
        R = rotation_towards(centre)

        rotated = (R @ grid.T).T
        translated = rotated + centre

        objpoints_3boards.append(translated.astype(np.float32))

    return objpoints_3boards


# ~ def detect_cboard_calib(images, ROIS, CHESSBOARD=(5, 3), SQUARE_SIZE=0.00225, save_debug_images=False):
    # ~ debug_folder = "outputs/calibration_test"
    # ~ if save_debug_images:
        # ~ shutil.rmtree(debug_folder, ignore_errors=True)
        # ~ os.makedirs(debug_folder, exist_ok=True)

def detect_cboard_calib(images, ROIS, CHESSBOARD=(5, 3), SQUARE_SIZE=0.00225, save_debug_images=False, debug_folder="outputs/calibration_test"):
    if save_debug_images:
        shutil.rmtree(debug_folder, ignore_errors=True)
        os.makedirs(debug_folder, exist_ok=True)

    print("Calibrating camera...")

    if ROIS is None:
        raise ValueError("ROIs must be defined for multi-board calibration")

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    num_boards = len(ROIS)

    objpoints = [[] for _ in range(num_boards)]
    imgpoints  = [[] for _ in range(num_boards)]

    # Flat grid model — same for every board during calibration
    objp_base = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
    objp_base[:, :2] = np.mgrid[0:CHESSBOARD[0],
                                 0:CHESSBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    save_idx = 0

    for fname in images:
        img  = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        debug_img = img.copy()

        for board_id, roi in enumerate(ROIS):
            x1, y1, x2, y2 = roi
            
            # Clamp ROI to image bounds
            h_img, w_img = gray.shape
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w_img, x2)
            y2 = min(h_img, y2)

            # Crop to ROI so OpenCV only sees this board
            roi_gray = gray[y1:y2, x1:x2]
            ret, corners = cv.findChessboardCorners(roi_gray, CHESSBOARD, None)
            if not ret:
                continue

            # Remap corners to full image coordinates
            corners[:, 0, 0] += x1
            corners[:, 0, 1] += y1

            corners = cv.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

            objpoints[board_id].append(objp_base.copy())
            imgpoints[board_id].append(corners)

            if save_debug_images:
                cv.drawChessboardCorners(debug_img, CHESSBOARD, corners, ret)
                cv.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if save_debug_images:
            cv.imwrite(f"{debug_folder}/calib_{save_idx:04d}.png", debug_img)
        save_idx += 1

    img_size = gray.shape[::-1]

    # Flatten per-board lists into single lists for calibrateCamera
    obj_flat = [pts for board in objpoints for pts in board]
    img_flat = [pts for board in imgpoints  for pts in board]

    return obj_flat, img_flat, img_size


# Extract the camera settings from input file
def prep_pi_cam_params(camera_file="camera_settings/pi_camera_settings.txt"):
    params = {}

    with open(camera_file, "r") as f:
        for line in f:
            line = line.strip()

            # Skip empty lines and comments
            if not line or line.startswith("#"):
                continue

            key, value = line.split("=", 1)
            params[key.strip()] = value.strip()

    # Convert numeric fields
    params["width"]       = int(params.get("width", 640))
    params["height"]      = int(params.get("height", 480))
    params["fps"]         = float(params.get("fps", 5.0))
    params["threshold"]   = float(params.get("threshold", 30.0))
    params["warmup"]      = float(params.get("warmup", 1.0))
    params["grayscale"]   = params.get("grayscale", "false").lower() == "true"

    # Optional overrides - remain None if not set in file
    exposure_us = params.get("exposure_us", "").strip()
    params["exposure_us"] = int(exposure_us) if exposure_us else None

    gain = params.get("gain", "").strip()
    params["gain"] = float(gain) if gain else None

    return params

# Proper check for valid frame on first go
def get_valid_frame(camera):
    frame = camera.capture_array("main")
    if frame is None or frame.size == 0:
        return None

    # Ensure 3-channel BGR
    if len(frame.shape) != 3:
        return None

    if frame.shape[2] == 4:
        frame = cv.cvtColor(frame, cv.COLOR_RGBA2BGR)  
    elif frame.shape[2] == 3:
        frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)  
    else:
        return None

    return frame


def capture_frame(picam2_):
    frame = picam2_.capture_array("main")
    if frame is None or frame.size == 0:
        return None
    if frame.shape[2] == 4:
        return cv.cvtColor(frame, cv.COLOR_RGBA2BGR)
    elif frame.shape[2] == 3:
        return cv.cvtColor(frame, cv.COLOR_RGB2BGR)
    return None
    

# Open pi camera 3 and set focus. Save debug image if enabled.
# ~ def open_picam(params, picam2_, debug_mode=False):
def open_picam(params, picam2_, debug_mode=False, output_dir="outputs"):

    frame_us = int(1_000_000 / params["fps"])

    try:
        picam2_ = Picamera2()

        # ~ config = picam2_.create_video_configuration(
            # ~ main={"format": "BGR888", "size": (params["width"], params["height"])},
            # ~ raw={"size": picam2_.sensor_resolution},
            # ~ controls={"FrameDurationLimits": (frame_us, frame_us)}
        # ~ )
        
        config = picam2_.create_video_configuration(
            main={"format": "BGR888", "size": (params["width"], params["height"])},
            raw={"size": (4608, 2592)},   # full sensor, matches widget mode
            controls={"FrameDurationLimits": (frame_us, frame_us)}
        )
        picam2_.configure(config)
        picam2_.start()

        # Let auto algorithms settle before focusing
        time.sleep(params["warmup"])
        
        # Log the sensor mode being used
        camera_config = picam2_.camera_configuration()
        print(f"[open_picam] [INFO] Raw stream config: {camera_config.get('raw', 'N/A')}")
        print(f"[open_picam] [INFO] Main stream config: {camera_config.get('main', 'N/A')}")
        

        # Try autofocus once, then lock the focus position
        try:
            picam2_.set_controls({"AfMode": controls.AfModeEnum.Auto})
            success = picam2_.autofocus_cycle()
            meta = picam2_.capture_metadata()
            lens_pos = meta.get("LensPosition", None)
            if success and lens_pos is not None:
                picam2_.set_controls({
                    "AfMode": controls.AfModeEnum.Manual,
                    "LensPosition": lens_pos
                })
        except Exception as exc:
            print(f'[open_picam] [ERROR] Could not focus Raspberry Pi camera: {exc}')
            return None

        # Read settled metadata and lock camera state
        meta = picam2_.capture_metadata()
        settled_exposure = int(meta.get("ExposureTime", min(5000, frame_us // 2)))
        settled_gain = float(meta.get("AnalogueGain", 1.0))
        colour_gains = meta.get("ColourGains", None)

        # Use exposure and gain from camera focus, or override if manual provided
        if params["exposure_us"] is not None:
            settled_exposure = params["exposure_us"]
        else:
            settled_exposure = min(settled_exposure, max(1000, int(0.6 * frame_us)))

        if params["gain"] is not None:
            settled_gain = params["gain"]

        lock_controls = {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": settled_exposure,
            "AnalogueGain": settled_gain,
            "FrameDurationLimits": (frame_us, frame_us),
        }

        if colour_gains is not None:
            lock_controls["ColourGains"] = colour_gains

        picam2_.set_controls(lock_controls)

        # Give time to settle
        time.sleep(0.2)

        # Test frame from camera
        previous_image = get_valid_frame(picam2_)
        if previous_image is None:
            print('[open_picam] [ERROR] No valid initial frame returned.')
            return None

        if params["grayscale"]:
            previous_image = cv.cvtColor(previous_image, cv.COLOR_BGR2GRAY)
            previous_image = cv.cvtColor(previous_image, cv.COLOR_GRAY2BGR)

        # Save focused frame to outputs/ for debug mode
        # ~ if debug_mode:
            # ~ os.makedirs("outputs", exist_ok=True)
            # ~ debug_path = "outputs/debug_mode_focus.jpeg"
            # ~ cv.imwrite(debug_path, previous_image)
            # ~ print(f"[open_picam] [DEBUG] Saved post-focus frame to: {debug_path}")

        
        if debug_mode:
            os.makedirs(output_dir, exist_ok=True)
            debug_path = f"{output_dir}/debug_mode_focus.jpeg"
            cv.imwrite(debug_path, previous_image)
            print(f"[open_picam] [DEBUG] Saved post-focus frame to: {debug_path}")

        # Log status
        print(f'[open_picam] [INFO] FPS: {params["fps"]:.1f}, frame time: {frame_us} us')
        print(f'[open_picam] [INFO] Locked exposure: {settled_exposure} us')
        print(f'[open_picam] [INFO] Locked gain: {settled_gain:.3f}')

        return picam2_

    except Exception as exc:
        print(f'[open_picam] [ERROR] Could not access Raspberry Pi camera: {exc}')
        return None


# Save calibration video with pi camera 3
def save_calib_video_picam(picam2_, display_widget = False, calib_time=5.0, calib_folder="outputs/calibration"):
    # Capture calibration frames
    frames = []
    start_time = time.time()
    if display_widget:
        try:
            while time.time() - start_time < calib_time:

                # frame = picam2_.capture_array("main")
                frame = capture_frame(picam2_)
                if frame is None:
                    continue

                # Live display window
                cv.imshow("Calibration Camera", frame)

                # Store frame
                frames.append(frame.copy())

                # Allow OpenCV UI to update + quit key
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[INFO] Early exit triggered")
                    break

                time.sleep(0.01)
        except Exception as exc:
            print(f"[save_calib_video_picam] [ERROR] Widget failed: {exc}\n")
            return None 
        finally:
            cv.destroyAllWindows()
    else: 
        while time.time() - start_time < calib_time:
            frame = capture_frame(picam2_) # picam2_.capture_array("main")
            if frame is None:
                continue
            frames.append(frame.copy())

    # Reset calibration folder
    shutil.rmtree(calib_folder, ignore_errors=True)
    os.makedirs(calib_folder, exist_ok=True)

    # Save frames to calibration folder
    for i, frame in enumerate(frames):
        cv.imwrite(f"{calib_folder}/frame_{i:04d}.jpeg", frame)

    print(f'[save_calib_video_picam] Saved {len(frames)} calibration frames to {calib_folder}\n')

    return picam2_


# Extract the current settings from autofocus for later re-application
def extract_applied_settings(picam2_):
    metadata = picam2_.capture_metadata()
    saved = {}

    # ExposureTime and AnalogueGain are safe to reapply
    for k in ["ExposureTime", "AnalogueGain"]:
        if k in metadata:
            saved[k] = metadata[k]

    # LensPosition is reapplyable but only meaningful with AfMode=Manual
    if "LensPosition" in metadata:
        saved["LensPosition"] = metadata["LensPosition"]

    print("Saved calibration camera settings:", saved)
    print("\n")

    return saved

# Safely close the camera
def close_camera(picam2_):
    try:
        picam2_.stop()
    except Exception:
        pass
    try:
        picam2_.close()
    except Exception:
        pass
    picam2_ = None
    
    return picam2_

# Reopen camera for experiment and apply calibration settings
def open_picam_for_exp(params, picam2_, saved_cam_settings):
    frame_us = int(1_000_000 / params["fps"])
    try:
        picam2_ = Picamera2()
        # ~ config = picam2_.create_video_configuration(
            # ~ main={"format": "BGR888", "size": (params["width"], params["height"])},
            # ~ raw={"size": picam2_.sensor_resolution},
            # ~ controls={"FrameDurationLimits": (frame_us, frame_us)}
        # ~ )
        config = picam2_.create_video_configuration(
            main={"format": "BGR888", "size": (params["width"], params["height"])},
            raw={"size": (4608, 2592)},   # full sensor, matches widget mode
            controls={"FrameDurationLimits": (frame_us, frame_us)}
        )
        
        picam2_.configure(config)
        picam2_.start()
        time.sleep(params["warmup"])

        if saved_cam_settings is not None:
            # Lock focus manually using saved lens position
            lock_controls = {
                "AfMode": controls.AfModeEnum.Manual,
                "AeEnable": False,   # Lock exposure - no auto re-adjustment
                "AwbEnable": False,  # Lock white balance
            }
            # Merge in the saved calibration values
            lock_controls.update(saved_cam_settings)

            # Clamp ExposureTime to fit within frame duration
            if "ExposureTime" in lock_controls:
                max_exp = max(1000, int(0.95 * frame_us))
                lock_controls["ExposureTime"] = min(lock_controls["ExposureTime"], max_exp)

            picam2_.set_controls(lock_controls)
            print("[INFO] Applied calibration-derived settings:", lock_controls)
        else:
            print("[WARN] No saved settings — camera using defaults")

        # Let controls settle
        time.sleep(0.2)  

        # Verify a valid frame before proceeding (and set as previous)
        test_frame = get_valid_frame(picam2_)
        if test_frame is None:
            print("[open_picam_for_exp] [ERROR] No valid initial frame.")
            return None

        if params["grayscale"]:
            test_frame = cv.cvtColor(test_frame, cv.COLOR_BGR2GRAY)
            test_frame = cv.cvtColor(test_frame, cv.COLOR_GRAY2BGR)

        print(f'[open_picam_for_exp] [INFO] FPS: {params["fps"]:.1f}, frame_us: {frame_us} us')
        print(f'[open_picam_for_exp] [INFO] Applied controls: {saved_cam_settings}\n')
        return picam2_

    except Exception as exc:
        print(f"[open_picam_for_exp] [ERROR] Could not open camera: {exc}\n")
        return None 


import struct


# Saves frames and histograms from experiment 
# ~ def save_exp_video(picam2_, display_widget=False, save_debug_images=False, exp_time=20.0, WINDOW_SIZE=1):
def save_exp_video(picam2_, display_widget=False, save_debug_images=False, exp_time=30.0, WINDOW_SIZE=1, output_dir="outputs"):
    
    print("Starting experiment")
    # Setup debug output folders
    # ~ if save_debug_images:
        # ~ event_frame_dir = "outputs/event_data/frames"
        # ~ event_hist_dir = "outputs/event_data/histograms"
        # ~ shutil.rmtree("outputs/event_data", ignore_errors=True)
        # ~ os.makedirs(event_frame_dir, exist_ok=True)
        # ~ os.makedirs(event_hist_dir, exist_ok=True)
        
    if save_debug_images:
        event_frame_dir = f"{output_dir}/event_data/frames"
        event_hist_dir  = f"{output_dir}/event_data/histograms"
        shutil.rmtree(f"{output_dir}/event_data", ignore_errors=True)
        os.makedirs(event_frame_dir, exist_ok=True)
        os.makedirs(event_hist_dir,  exist_ok=True)

    # Initialise event camera emulator
    e_camera_emulator = EventCameraEmulator()
    frames = []
    frame_idx = 0
    hist_records = []  # list of packed histogram bytes, one per window

    # Wait up to 10s for first frame
    timeout = 10.0
    start_time = time.time()
    frame = None
    while frame is None and (time.time() - start_time) < timeout:
        try:
            # frame = picam2_.capture_array("main")
            frame = capture_frame(picam2_)
        except Exception:
            frame = None
    if frame is None:
        print("[save_exp_video] [ERROR] Experiment failed: no frame received within 10s")
        return None
    else:
        prev_frame = frame
        print("[save_exp_video] [INFO] Read first frame")

    # Inner per-frame processing - shared between widget and non-widget paths
    def process_frame(frame):
        nonlocal prev_frame, event_hist, frame_idx
        frames.append(frame)
        
        # Emulate event camera
        event_image = e_camera_emulator.get_events_image_rgb(
            frame, prev_frame, 30,
            record_off_events=True,
            register_off_events_as_on=False
        )
        visual_event_image = e_camera_emulator.get_visual_events_image(event_image)
        prev_frame = frame

        # Add to spatial histogram
        gray_event = cv.cvtColor(event_image, cv.COLOR_BGR2GRAY) if event_image.ndim == 3 else event_image
        gray_event = gray_event.astype(np.float32)
        if event_hist is None:
            event_hist = np.zeros_like(gray_event, dtype=np.float32)
        event_hist += np.abs(gray_event)

        # Accumulate histogram window
        if frame_idx % WINDOW_SIZE == 0 and frame_idx > 0:
            hist_idx = frame_idx // WINDOW_SIZE
            if hist_idx < 150:
                # ~ # 1 bit version to bin (working)
                # ~ hist_binary = (event_hist > 0).astype(np.uint8)
                # ~ hist_packed = np.packbits(hist_binary, axis=None)
                # ~ hist_records.append(hist_packed.tobytes())
                # ~ # 8 bit version to csv
                # ~ hist_uint8 = np.clip(event_hist, 0, 255).astype(np.uint8)
                # ~ hist_records.append(hist_uint8.flatten())
                # 1 bit to csv
                hist_binary = (event_hist > 0).astype(np.uint8)
                hist_records.append(hist_binary.flatten())

        # Save debug images if enabled
        if save_debug_images:
            cv.imwrite(os.path.join(event_frame_dir, f"event_{frame_idx:04d}.png"), visual_event_image)
            if frame_idx % WINDOW_SIZE == 0 and frame_idx > 0:
                hist_vis = cv.normalize(event_hist, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
                cv.imwrite(os.path.join(event_hist_dir, f"event_hist_vis_{frame_idx:05d}.png"), hist_vis)
        
        # Reset histogram window
        if frame_idx % WINDOW_SIZE == 0 and frame_idx > 0:
            event_hist = np.zeros_like(gray_event, dtype=np.float32)
        frame_idx += 1

    start_time = time.time()
    event_hist = None
    if display_widget:
        try:
            while time.time() - start_time < exp_time:
                frame = capture_frame(picam2_)
                if frame is None:
                    continue
                cv.imshow("Experiment Camera", frame)
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[INFO] Early exit triggered")
                    break
                process_frame(frame)
        except Exception as exc:
            print(f"[save_exp_video] [ERROR] Widget failed: {exc}\n")
            return None
        finally:
            cv.destroyAllWindows()
    else:
        while time.time() - start_time < exp_time:
            frame = capture_frame(picam2_) # picam2_.capture_array("main")
            if frame is None:
                continue
            process_frame(frame)

    # Save baseline images for later pose estimation
    # ~ baseline_folder = "outputs/baseline"
    baseline_folder = f"{output_dir}/baseline"
    shutil.rmtree(baseline_folder, ignore_errors=True)
    os.makedirs(baseline_folder, exist_ok=True)

    # Save RGB frames for later pose estimation
    for i, frame in enumerate(frames):
        cv.imwrite(f"{baseline_folder}/frame_{i:04d}.jpeg", frame)

    print("[save_exp_video] [INFO] Experiment recording complete\n")
    return hist_records


# ~ def process_baseline_data(objpoints_3boards, mtx, dist, ROIS, CHESSBOARD=(5, 3),
                           # ~ baseline_folder="outputs/baseline",
                           # ~ pose_folder="outputs/baseline_pose",
                           # ~ save_debug_images=False):

def process_baseline_data(objpoints_3boards, mtx, dist, ROIS, CHESSBOARD=(5, 3),
                           baseline_folder="outputs/baseline",
                           pose_folder="outputs/baseline_pose",
                           save_debug_images=False,
                           results_dir="outputs/experiment_results"):
    print("Processing baseline frames")
    images = sorted(glob.glob(os.path.join(baseline_folder, "*.jpeg")))
    # print("Found images:", len(images))

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    os.makedirs(pose_folder, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)
    csv_path = os.path.join(results_dir, "experiment_results.csv")
    # results_csv = open(csv_path, "a", newline="")
    # results_writer = csv.writer(results_csv)

    # To store poses
    all_tvecs = []
    all_rvecs = []
    
    for fname in images:
        img  = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        base = os.path.basename(fname)
        name, _ = os.path.splitext(base)

        # To store 3 poses per image (bad naming convention)
        tvecs_all = []
        rvecs_all = []

        for board_id, roi in enumerate(ROIS):
            x1, y1, x2, y2 = roi
            
            # Clamp ROI to image bounds
            h_img, w_img = gray.shape
            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(w_img, x2)
            y2 = min(h_img, y2)

            # Crop to ROI so OpenCV only sees this board
            roi_gray = gray[y1:y2, x1:x2]
            ret, corners = cv.findChessboardCorners(roi_gray, CHESSBOARD, None)
            if not ret:
                continue

            # Remap corners to full image coordinates
            corners[:, 0, 0] += x1
            corners[:, 0, 1] += y1

            corners = cv.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

            obj_model = objpoints_3boards[board_id]
            success, rvec, tvec = cv.solvePnP(obj_model, corners, mtx, dist)
            if success:
                tvecs_all.append(tvec.flatten())
                rvecs_all.append(rvec.flatten())
                if save_debug_images:
                    cv.drawChessboardCorners(img, CHESSBOARD, corners, ret)

        # Write t then r to file
        # if len(tvecs_all) == 0:
        #     # print(f"[WARNING] {fname}: no boards detected, writing zeros.")
        #     # results_file.write(struct.pack("<6f", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        #     results_writer.writerow([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # else:
        #     # if len(tvecs_all) < 3:
        #         # print(f"[WARNING] {fname}: only {len(tvecs_all)}/3 boards detected.")
        #     t_centre = np.mean(tvecs_all, axis=0)
        #     r_centre = np.mean(rvecs_all, axis=0)
        #     # ~ results_file.write(struct.pack("<6f", t_centre[0], t_centre[1], t_centre[2],
        #                                           # ~ r_centre[0], r_centre[1], r_centre[2]))
        #     results_writer.writerow([t_centre[0], t_centre[1], t_centre[2],
        #                  r_centre[0], r_centre[1], r_centre[2]])
    
        # Store t and r (to write out in block later)
        if len(tvecs_all) == 0:
            all_tvecs.append([0.0, 0.0, 0.0])
            all_rvecs.append([0.0, 0.0, 0.0])
        else:
            all_tvecs.append(np.mean(tvecs_all, axis=0).tolist())
            all_rvecs.append(np.mean(rvecs_all, axis=0).tolist())

        if save_debug_images:
            cv.imwrite(os.path.join(pose_folder, f"{name}_multi_pose.png"), img)

    # For binary
    # ~ results_file.close()

    # Write: all positions (tx,ty,tz) then all attitudes (rx,ry,rz), one frame per row
    csv_path = os.path.join(results_dir, "experiment_results.csv")
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        # writer.writerow(["# positions (tx,ty,tz)"])
        for tvec in all_tvecs:
            writer.writerow(tvec)
        # writer.writerow(["# attitudes (rx,ry,rz)"])
        for rvec in all_rvecs:
            writer.writerow(rvec)
    
    print(f"[process_baseline_data] [INFO] Wrote {len(all_tvecs)} position and attitude records\n")

    # For csv
    # results_csv.close()


# Save the experiment results
import csv

def save_exp_results(hist_records, results_dir="outputs/experiment_results"):
    # Setup output folders
    # results_dir = "outputs/experiment_results"
    
    os.makedirs(results_dir, exist_ok=True) # backup
    
    # ~ # Write histogram records to binary file
    # ~ results_file = open(os.path.join(results_dir, "experiment_results.bin"), "ab")
    # ~ for record in hist_records:
        # ~ results_file.write(record)
    # ~ results_file.close()
    
    # ~ print("[save_exp_results] [INFO] Experiment results saved\n")
    
    # Debugging
    # print(f"[save_exp_results] [INFO] hist_records count: {len(hist_records)}")
    # if len(hist_records) > 0:
    #     print(f"[save_exp_results] [INFO] First record shape: {hist_records[0].shape}, dtype: {hist_records[0].dtype}")
    
    # Write histogram to csv
    csv_path = os.path.join(results_dir, "experiment_results.csv")
    # with open(csv_path, "a", newline="") as f:
    #     writer = csv.writer(f)
    #     for record in hist_records:
    #         writer.writerow(record.tolist())
    # Write with hex encoding
    csv_path = os.path.join(results_dir, "experiment_results.csv")
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        # writer.writerow(["# histograms (hex-encoded, one row per window)"])
        for record in hist_records:
            # Pack bits then encode as hex string - compact single-column row
            packed = np.packbits(record)
            writer.writerow([packed.tobytes().hex()])
    print("[save_exp_results] [INFO] Experiment results saved\n")
    

# Add output_dir param so cleanup targets the right boot folder
def cleanup_images(cleanup_enabled=False, output_dir="outputs"):
    if not cleanup_enabled:
        return
    dirs_to_clean = [
        f"{output_dir}/calibration",
        f"{output_dir}/calibration_test",
        f"{output_dir}/baseline",
        f"{output_dir}/baseline_pose",
        f"{output_dir}/event_data/frames",
        f"{output_dir}/event_data/histograms",
    ]
    for folder in dirs_to_clean:
        for file_path in glob.glob(os.path.join(folder, "*")):
            try:
                os.remove(file_path)
            except Exception as e:
                print(f"[WARN] Failed to delete {file_path}: {e}")
    print("Deleted saved images for this run.\n")

# ~ # Delete experiment images, and debug images if produced
# ~ def cleanup_images(cleanup_enabled = False, output_dir="outputs"):
    # ~ if cleanup_enabled == True:
        # ~ # Delete calibration images
        # ~ for file_path in glob.glob(os.path.join("outputs/calibration", "*")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")

        # ~ # Delete calibration test debug images
        # ~ for file_path in glob.glob(os.path.join("outputs/calibration_test", "*")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")

        # ~ # Delete experiment baseline images
        # ~ for file_path in glob.glob(os.path.join("outputs/baseline", "*")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")

        # ~ # Delete baseline pose debug images
        # ~ for file_path in glob.glob(os.path.join("outputs/baseline_pose", "*.png")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")
        
        # ~ # Delete event data frames
        # ~ for file_path in glob.glob(os.path.join("outputs/frames/", "*")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")
        
        # ~ # Delete event data histograms
        # ~ for file_path in glob.glob(os.path.join("outputs/histograms/", "*")):
            # ~ try:
                # ~ os.remove(file_path)
            # ~ except Exception as e:
                # ~ print(f"[WARN] Failed to delete {file_path}: {e}")

        # ~ print("Deleted saved images for this run.\n")

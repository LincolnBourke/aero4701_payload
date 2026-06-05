"""
roi_tuner.py  –  Live Pi Camera 3 feed with adjustable ROI overlays.

Usage:
    python roi_tuner.py

Controls (OpenCV window must be focused):
    w/s        – move selected ROI up / down
    a/d        – move selected ROI left / right
    i/k        – make selected ROI taller / shorter
    j/l        – make selected ROI wider / narrower
    Tab        – cycle selected ROI  (0 → 1 → 2 → 0 …)
    p          – print current ROI values (copy-paste into define_rois)
    q / Esc    – quit

The ROI values printed by 'p' can be pasted directly into define_rois().
"""

import time
import cv2 as cv
import numpy as np
from picamera2 import Picamera2
from libcamera import controls

# ──────────────────────────────────────────────────────────────────────────────
# Helpers copied / adapted from your main program
# ──────────────────────────────────────────────────────────────────────────────

def load_camera_settings(path="camera_settings/pi_camera_settings.txt"):
    params = {
        "width": 640, "height": 480, "fps": 5.0,
        "warmup": 1.0, "grayscale": False,
        "exposure_us": None, "gain": None,
    }
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                k, v = line.split("=", 1)
                params[k.strip()] = v.strip()
    except FileNotFoundError:
        print(f"[WARN] Settings file '{path}' not found – using defaults.")

    params["width"]     = int(params["width"])
    params["height"]    = int(params["height"])
    params["fps"]       = float(params["fps"])
    params["warmup"]    = float(params["warmup"])
    params["grayscale"] = str(params.get("grayscale", "false")).lower() == "true"
    exp = params.get("exposure_us")
    params["exposure_us"] = int(exp) if exp not in (None, "None", "") else None
    gain = params.get("gain")
    params["gain"] = float(gain) if gain not in (None, "None", "") else None
    return params


def make_default_rois(width=640, height=480, scale=0.5):
    """Mirrors define_rois() from your main program."""
    w = int(width * scale)
    h = int(height * scale)
    cx, cy = width // 2, height // 2
    dx = int(width * 0.2)
    dy = int(height * 0.2)
    top_extra    = int(height * 0.1)
    bottom_extra = int(height * 0.1)
    return [
        (cx - w//2,      cy - h//2 - dy - top_extra,
         cx + w//2,      cy + h//2 - dy - top_extra),   # top board
        (cx - w//2 - dx, cy - h//2 + dy + bottom_extra,
         cx + w//2 - dx, cy + h//2 + dy + bottom_extra), # bottom-left board
        (cx - w//2 + dx, cy - h//2 + dy + bottom_extra,
         cx + w//2 + dx, cy + h//2 + dy + bottom_extra), # bottom-right board
    ]

def write_roi():
    """Hardcoded ROIs tuned for 640x480."""
    return [
        (160,  11, 480, 266),  # Board 0 (top)
        ( 32, 194, 377, 484),  # Board 1 (bot-left)
        (258, 189, 608, 479),  # Board 2 (bot-right)
    ]

def open_camera(params):
    frame_us = int(1_000_000 / params["fps"])
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"format": "BGR888", "size": (params["width"], params["height"])},
        raw={"size": (4608, 2592)},
        controls={"FrameDurationLimits": (frame_us, frame_us)},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(params["warmup"])

    # Auto-focus once then lock
    try:
        picam2.set_controls({"AfMode": controls.AfModeEnum.Auto})
        picam2.autofocus_cycle()
        meta  = picam2.capture_metadata()
        lpos  = meta.get("LensPosition")
        if lpos is not None:
            picam2.set_controls({
                "AfMode": controls.AfModeEnum.Manual,
                "LensPosition": lpos,
            })
    except Exception as e:
        print(f"[WARN] Autofocus failed: {e}")

    # Lock exposure / gain
    meta     = picam2.capture_metadata()
    exp      = params["exposure_us"] or int(meta.get("ExposureTime", min(5000, frame_us // 2)))
    exp      = min(exp, max(1000, int(0.6 * frame_us)))
    gain     = params["gain"]     or float(meta.get("AnalogueGain", 1.0))
    cg       = meta.get("ColourGains")
    lock     = {
        "AeEnable": False, "AwbEnable": False,
        "ExposureTime": exp, "AnalogueGain": gain,
        "FrameDurationLimits": (frame_us, frame_us),
    }
    if cg:
        lock["ColourGains"] = cg
    picam2.set_controls(lock)
    time.sleep(0.2)
    print(f"[open_camera] Exposure={exp} us  Gain={gain:.3f}")
    return picam2


def capture(picam2):
    frame = picam2.capture_array("main")
    if frame is None or frame.size == 0:
        return None
    if frame.shape[2] == 4:
        return cv.cvtColor(frame, cv.COLOR_RGBA2BGR)
    return cv.cvtColor(frame, cv.COLOR_RGB2BGR)


# ──────────────────────────────────────────────────────────────────────────────
# Drawing
# ──────────────────────────────────────────────────────────────────────────────

COLOURS   = [(0, 255, 0), (0, 200, 255), (255, 100, 0)]   # green, cyan, orange
LABELS    = ["Board 0 (top)", "Board 1 (bot-left)", "Board 2 (bot-right)"]
STEP_MOVE = 5    # pixels per key press (position)
STEP_SIZE = 5    # pixels per key press (size)


def draw_overlay(frame, rois, selected):
    vis = frame.copy()
    h_img, w_img = vis.shape[:2]

    for i, (x1, y1, x2, y2) in enumerate(rois):
        colour    = COLOURS[i % len(COLOURS)]
        thickness = 3 if i == selected else 1
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # Clamp for drawing only
        dx1, dy1 = max(0, x1), max(0, y1)
        dx2, dy2 = min(w_img, x2), min(h_img, y2)

        cv.rectangle(vis, (dx1, dy1), (dx2, dy2), colour, thickness)
        cv.putText(vis, LABELS[i], (dx1 + 5, dy1 + 22),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2, cv.LINE_AA)

        # Cross-hair at centre
        cv.drawMarker(vis, (cx, cy), colour, cv.MARKER_CROSS, 14, 1)

    # ~ # HUD
    # ~ hud = [
        # ~ f"Selected: {selected} – {LABELS[selected]}",
        # ~ "w/s: up/down  a/d: left/right",
        # ~ "i/k: taller/shorter  j/l: wider/narrower",
        # ~ "Tab: next ROI   p: print values   q/Esc: quit",
    # ~ ]
    # ~ for row, txt in enumerate(hud):
        # ~ cv.putText(vis, txt, (8, 18 + row * 20),
                   # ~ cv.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv.LINE_AA)
        # ~ cv.putText(vis, txt, (8, 18 + row * 20),
                   # ~ cv.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0),       3, cv.LINE_AA)
        # ~ cv.putText(vis, txt, (8, 18 + row * 20),
                   # ~ cv.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv.LINE_AA)
    return vis


def print_rois(rois, width, height):
    print("\n" + "=" * 60)
    print("Current ROI values  (copy into define_rois):\n")
    print(f"    # Image size: {width} x {height}")
    print("    ROIS = [")
    for i, (x1, y1, x2, y2) in enumerate(rois):
        print(f"        ({x1}, {y1}, {x2}, {y2}),  # {LABELS[i]}")
    print("    ]")
    print("=" * 60 + "\n")


# ──────────────────────────────────────────────────────────────────────────────
# Main loop
# ──────────────────────────────────────────────────────────────────────────────

def main():
    params   = load_camera_settings()
    W, H     = params["width"], params["height"]
    picam2   = open_camera(params)

    rois     = list(write_roi()) # list(make_default_rois(W, H))   # mutable list of tuples
    selected = 0

    cv.namedWindow("ROI Tuner", cv.WINDOW_NORMAL)
    cv.resizeWindow("ROI Tuner", W, H)

    print("\nROI Tuner started.  Window must be focused for key controls.\n")

    while True:
        frame = capture(picam2)
        if frame is None:
            continue

        if params["grayscale"]:
            frame = cv.cvtColor(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), cv.COLOR_GRAY2BGR)

        vis = draw_overlay(frame, rois, selected)
        cv.imshow("ROI Tuner", vis)

        key = cv.waitKey(1) & 0xFF
        if key in (ord('q'), 27):          # q or Esc
            break

        x1, y1, x2, y2 = rois[selected]

        # ── movement ──────────────────────────────────────────────────────
        if   key == ord('w'):  y1 -= STEP_MOVE; y2 -= STEP_MOVE
        elif key == ord('s'):  y1 += STEP_MOVE; y2 += STEP_MOVE
        elif key == ord('a'):  x1 -= STEP_MOVE; x2 -= STEP_MOVE
        elif key == ord('d'):  x1 += STEP_MOVE; x2 += STEP_MOVE

        # ── resize ────────────────────────────────────────────────────────
        elif key == ord('i'):  y1 -= STEP_SIZE  # taller (expand top)
        elif key == ord('k'):  y1 += STEP_SIZE  # shorter (shrink top)
        elif key == ord('j'):  x1 -= STEP_SIZE  # wider (expand left)
        elif key == ord('l'):  x1 += STEP_SIZE  # narrower (shrink left)

        # ── meta ──────────────────────────────────────────────────────────
        elif key == 9:                         # Tab – next ROI
            selected = (selected + 1) % len(rois)
            print(f"[ROI Tuner] Selected: {selected} – {LABELS[selected]}")
            continue                           # skip the rois write below
        elif key == ord('p'):
            print_rois(rois, W, H)
            continue

        rois[selected] = (x1, y1, x2, y2)

    cv.destroyAllWindows()
    picam2.stop()
    picam2.close()

    # Final printout so values aren't lost on exit
    print_rois(rois, W, H)


if __name__ == "__main__":
    main()


# -*- coding: utf-8 -*-
"""
Created on Fri Dec 19 16:03:16 2025

@author: Adel Lee (SYK)
"""
import math
import serial
import time
import mediapipe as mp
import cv2
import tkinter as tk
from threading import Thread
import numpy as np

# ---------------------------
# Servo setup (angle and variable definition)
# ---------------------------
### Author's Note: Servo start angle/min angle is always in closed position. Left Servos are always mirrored ###

rx_min, rx_max = 72, 108 ## Right Eye X-Direction - Servo 0 (-)R, (+)L
lx_min, lx_max = 72, 108 ## Left Eye X-Direction - Servo 1 (-R), (+)L

ry_min, ry_max = 76.5, 103.5 ## Right Eye Y-Direction - Servo 2 (-)D, (+)U
ly_min, ly_max = 76.5, 103.5 ## Left Eye Y-Direction - Servo 3 (-)U, (+)D

rb_min, rb_max = 20, 90 ## Right Eye Blink - Servo 4 (-)U
lb_min, lb_max = 90, 140 ## Left Eye Blink - Servo 5 (+)U

reb_min, reb_max = 75, 105 ## Right Eyebrow - Servo 6 (-)D, (+)U  - Index 295 track relative to midpoint of eye_width
leb_min, leb_max = 75, 105 ## Left Eyebrow - Servo 7 - (-)U, (+)D - Index 55

m1_min, m1_max = 90, 180 ## Right Mouth Servo - Servo 8 (-)U, (+)D
m2_min, m2_max = 0, 90 ## Left Mouth Servo - Servo 9 (-)D, (+)US

nose_min, nose_max = 81,120 ## Nose Servo - Servo 10 (-)U, (+)D

## Define Horizontal Eye Distance as a scale bar for mm conversion ##
horizontal_eye = 32

SENSITIVITY = 6             # Max deviation from center, Adjust this setting to increase range of angle
SMOOTH_ALPHA = 0.9           # smoothing factor, Adjust this setting if smoothing is less or more needed 

# Calibration storage
calib = {"x_min": None, "x_max": None, "y_min": None, "y_max": None}

# Mediapipe
mp_face_mesh = mp.solutions.face_mesh

selected_indices = [386, 374, 145, 159, 13, 14, 393, 269,
                    33, 133, 362, 263, 468, 473, 282, 442,222,52]

# ---------------------------
# Serial setup (safe) 
# ---------------------------
SERIAL_PORT = 'COM5'   # change this as needed
BAUD = 115200

arduino = None
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2)  # wait for Arduino reset
except Exception as e:
    print(f"Warning: couldn't open serial port {SERIAL_PORT}: {e}")
    arduino = None

def pack_send(packet):
    """Send packet to Arduino with start/end markers and integer angles.
       Packet is list/tuple of numeric values.
    """
    if arduino is None:
        # For debugging without Arduino connected
        print("Would send:", packet)
        return
    # Ensure integers and clamp 0-180 (typical servo range)
    ints = [max(0, min(180, int(round(x)))) for x in packet]
    msg = "<" + ",".join(map(str, ints)) + ">"
    try:
        arduino.write(msg.encode())
    except Exception as e:
        print("Serial write error:", e)
        
# ---------------------------
# Utility Functions
# ---------------------------
# Source - https://stackoverflow.com/a
# Posted by Alok Singhal, modified by community. See post 'Timeline' for change history
# Retrieved 2025-12-29, License - CC BY-SA 4.0, for "def myround" only

def myround(x, base=2):
    return base * round(x/base)

def eu_dist(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


eye_max = {"L": 0.0, "R": 0.0}  # store per eye

def theta14(p1,p2,sf):
    mm_convert=sf*eu_dist(p1,p2)
    eye_ratio = (mm_convert/(0.6*horizontal_eye))*90
    print(eye_ratio)
    if eye_ratio < 30:
        return 90
    elif eye_ratio >= 100:
        return 170
    else:
        return myround(eye_ratio)+90

def scale_factor (coords):
    lcorner, rcorner = coords[33], coords[133]
    eye_width = rcorner[0] - lcorner[0]
    return horizontal_eye/eye_width

##Top lip as anchor/ground for eu-dist calculation.
def mouth(m1, m2, sf):
    mdist = eu_dist(m1, m2)
    mdist_mm = mdist*sf
    mangle = math.degrees(math.asin(mdist_mm/140))*3 ## 140mm is the Length of Current Mouth
    m1angle = m1_min + mangle # Right mouth angle      
    return myround(m1angle)

def eyebrow(brow, eye, sf, mi, ma):
    edist = eu_dist(eye,brow)
    eangle = (myround(edist*sf,base = 5)/(1.2*horizontal_eye))*abs(ma-mi)+mi-16
    if eangle < 90 and eangle > 85:
        return 90
    if eangle > 90:
        return ma
    else:
        return myround(eangle, base =5)
    
def nose(l1, l2, sf):
    ldist = eu_dist(l1, l2)
    ldist_mm = ldist*sf 
    return myround(math.degrees(math.asin(ldist_mm/65))*3 + nose_min) ## 65mm is length of nose pivot to servo (Length of nose linkage)


# ---------------------------
# Landmark Extraction
# ---------------------------

def get_landmarks(frame, face_mesh):
    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb)
    coords = {}
    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            for idx in selected_indices:
                lm = face_landmarks.landmark[idx]
                coords[idx] = (int(lm.x*w), int(lm.y*h))
    return coords

# ---------------------------
# Continuous Calibration
# ---------------------------

def update_calibration(iris_ratio):
    # iris_ratio = (x_ratio, y_ratio)
    x, y = iris_ratio
    if calib["x_min"] is None or x < calib["x_min"]: calib["x_min"] = x
    if calib["x_max"] is None or x > calib["x_max"]: calib["x_max"] = x
    if calib["y_min"] is None or y < calib["y_min"]: calib["y_min"] = y
    if calib["y_max"] is None or y > calib["y_max"]: calib["y_max"] = y

# ---------------------------
# Eye Tracking
# ---------------------------

def process_eye_tracking(coords, prev_angles, eye="left"):
    if eye == "left":
        lcorner, rcorner = coords[33], coords[133]
        iris = coords[468]
    elif eye == "right":
        lcorner, rcorner = coords[362], coords[263]
        iris = coords[473]

    # Avoid division by zero
    eye_width = rcorner[0] - lcorner[0]
    eye_height = rcorner[1] - lcorner[1]
    if eye_width == 0: eye_width = 1
    if eye_height == 0: eye_height = 1

    iris_x_ratio = (iris[0] - lcorner[0]) / eye_width
    iris_y_ratio = (iris[1] - lcorner[1]) / eye_height

    # Update calibration
    update_calibration((iris_x_ratio, iris_y_ratio))

    # Map to servo angles
    if None not in calib.values():
        norm_x = np.interp(iris_x_ratio, [calib["x_min"], calib["x_max"]],
                           [90-SENSITIVITY, 90+SENSITIVITY])
        norm_y = np.interp(iris_y_ratio, [calib["y_min"], calib["y_max"]],
                           [90-SENSITIVITY, 90+SENSITIVITY])
    else:
        norm_x, norm_y = 90, 90

    # Apply smoothing
    smoothed_x = SMOOTH_ALPHA*norm_x + (1-SMOOTH_ALPHA)*prev_angles[0] ## Additional 1.2sf for more exaggerated movements
    smoothed_y = SMOOTH_ALPHA*norm_y + (1-SMOOTH_ALPHA)*prev_angles[1]

    return smoothed_x, smoothed_y

# ---------------------------
# Live Stream
# ---------------------------
def open_video_source(source_type="webcam", video_path=None):
    if source_type == "webcam":
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    elif source_type == "video" and video_path:
        cap = cv2.VideoCapture(video_path, cv2.CAP_FFMPEG)
    else:
        raise ValueError("Invalid video source")

    if not cap.isOpened():
        raise RuntimeError("Could not open video source")

    return cap

stream_running = False

def start_stream(source_type="webcam", video_path=None):
    global stream_running

    if stream_running:
        print("Stream already running")
        return

    stream_running = True

    def run():
        global stream_running
        cap = None

        try:
            cap = open_video_source(source_type, video_path)
            prev_angles = [90, 90]

            with mp_face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.7,
                min_tracking_confidence=0.7
            ) as face_mesh:

                while cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        break

                    coords = get_landmarks(frame, face_mesh)
                    if len(coords) < len(selected_indices):
                        cv2.imshow("Eye Tracking", frame)
                        if cv2.waitKey(1) & 0xFF == 27:
                            break
                        continue

                    # ---- Calculations ----
                    sf = scale_factor(coords)

                    LeyeX, LeyeY = process_eye_tracking(coords, prev_angles, "left")
                    ReyeX, ReyeY = process_eye_tracking(coords, prev_angles, "right")
                    prev_angles = [LeyeX, LeyeY]

                    Lblink_raw = theta14(coords[374], coords[386], sf)
                    Rblink_raw = theta14(coords[159], coords[145], sf)

                    M1_raw = mouth(coords[13], coords[14], sf)

                    Rebrow_raw = eyebrow(coords[282], coords[362], sf, reb_min, reb_max)
                    Lebrow_raw = eyebrow(coords[52], coords[133], sf, leb_min, leb_max)

                    Nose_raw = nose(coords[393], coords[269], sf)

                    # ---- Servo mapping ----
                    packet = [
                        int(np.clip(ReyeX, rx_min, rx_max)),
                        int(np.clip(LeyeX, lx_min, lx_max)),
                        int(np.clip(180 - ReyeY, ry_min, ry_max)),
                        int(np.clip(LeyeY, ly_min, ly_max)),
                        int(np.clip(180 - Rblink_raw, rb_min, rb_max)),
                        int(np.clip(Lblink_raw, lb_min, lb_max)),
                        int(np.clip(Rebrow_raw, reb_min, reb_max)),
                        int(np.clip(180 - Lebrow_raw, leb_min, leb_max)),
                        int(np.clip(M1_raw, m1_min, m1_max)),
                        int(np.clip(180 - M1_raw, m2_min, m2_max)),
                        int(np.clip(Nose_raw, nose_min, nose_max)),
                    ]
                    print(packet)
                    pack_send(packet)

                    # ---- Draw overlay ----
                    for pt in coords.values():
                        cv2.circle(frame, pt, 2, (0, 255, 0), -1)

                    cv2.imshow("Eye Tracking", frame)

                    delay = 3 if source_type == "video" else 1
                    if cv2.waitKey(delay) & 0xFF == 27:
                        break

        except Exception as e:
            print("Stream error:", e)

        finally:
            if cap:
                cap.release()
            cv2.destroyAllWindows()
            stream_running = False
            print("Stream stopped")

    Thread(target=run, daemon=True).start()


# ---------------------------
# Tkinter GUI
# ---------------------------
from tkinter import filedialog
from pathlib import Path

root = tk.Tk() 
root.title("Eye Tracker Stabilized")

def start_webcam():
    start_stream(source_type="webcam")


def start_video():
    path = filedialog.askopenfilename(
        title="Select video file",
        filetypes=[("Video files", "*.mp4 *.avi *.mov")]
    )

    if not path:
        return

    # Convert to absolute path and use forward slashes
    video_path = str(Path(path).resolve()).replace("\\", "/")
    print("Normalized path:", video_path)

    # Try opening it manually first to catch errors
    cap_test = cv2.VideoCapture(video_path, cv2.CAP_ANY)
    if not cap_test.isOpened():
        print("Stream error: Could not open video source")
        return
    cap_test.release()

    # Now start the stream with the safe path
    start_stream(source_type="video", video_path=video_path)


start_btn = tk.Button(root, text="📷 Start Webcam", font=("Arial",14),
                      command=start_webcam)
start_btn.pack(padx=20,pady=10)

video_btn = tk.Button(root, text="🎥 Load Video", font=("Arial",14),
                      command=start_video)
video_btn.pack(padx=20,pady=10)

root.mainloop()
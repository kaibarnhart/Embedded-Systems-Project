#!/usr/bin/env python

import tensorflow as tf
tf.config.threading.set_inter_op_parallelism_threads(4)
tf.config.threading.set_intra_op_parallelism_threads(4)

from model import create_model
import cv2
import sys
import os
import time
import math
import numpy as np
import serial

# Open UART connection to HiFive
ser = serial.Serial("/dev/ttyAMA2", 115200)

# Convert degrees to radians
def deg2rad(deg):
    return deg * math.pi / 180.0

# Convert radians to degrees
def rad2deg(rad):
    return 180.0 * rad / math.pi

# Pick number of CPU threads
if len(sys.argv) > 1:
    NCPU = int(sys.argv[1])
else:
    NCPU = 1

# Load model and weights
model = create_model(input_shape=(66, 200, 3))
model.load_weights("model/model.h5")

# Basic frame settings
NFRAMES = 1000
curFrame = 0
period = 50   # ms between frames
is_periodic = True

# Lists to track timing
cap_time_list = []
prep_time_list = []
pred_time_list = []
tot_time_list = []

print("---------- Processing video for epoch 1 ----------")

# Load video file
vid_path = "epoch-1.avi"
assert os.path.isfile(vid_path)
cap = cv2.VideoCapture(vid_path)

print("Performing inference...")
time_start = time.time()
first_frame = True
count = 0

# Main loop
while True:

    if curFrame < NFRAMES:

        cam_start = time.time()

        # Read video frame
        ret, img = cap.read()
        if not ret:
            break

        prep_start = time.time()

        # Resize + normalize frame
        img = cv2.resize(img, (200, 66))
        img = img / 255.0
        img = np.expand_dims(img, axis=0)

        pred_start = time.time()

        # Run model prediction
        rad = model.predict(img)[0][0]
        deg = rad2deg(rad)

        # Send steering angle to HiFive
        angle_str = str(int(deg)) + "\n"
        ser.write(angle_str.encode())
        print(f"Sent angle: {int(deg)}")

        pred_end = time.time()

        # Timing calculations
        cam_time = (prep_start - cam_start) * 1000
        prep_time = (pred_start - prep_start) * 1000
        pred_time = (pred_end - pred_start) * 1000
        tot_time = (pred_end - cam_start) * 1000

        print(
            "pred: {:0.2f} deg. took: {:0.2f} ms | cam={:0.2f} prep={:0.2f} pred={:0.2f}".format(
                deg, tot_time, cam_time, prep_time, pred_time
            )
        )

        if first_frame:
            first_frame = False
        else:
            tot_time_list.append(tot_time)

        curFrame += 1

        # Keep timing consistent
        wait_time = (period - tot_time) / 1000
        if is_periodic and wait_time > 0:
            time.sleep(wait_time)

        count += 1

    else:
        break

# Close everything
cap.release()
ser.close()

# Print summary stats
fps = curFrame / (time.time() - time_start)
print("completed inference, total frames: {}, average fps: {} Hz".format(curFrame + 1, round(fps, 1)))
print("count:", len(tot_time_list))
print("mean:", np.mean(tot_time_list))
print("max:", np.max(tot_time_list))
print("99.999pct:", np.percentile(tot_time_list, 99.999))
print("99.99pct:", np.percentile(tot_time_list, 99.99))
print("99.9pct:", np.percentile(tot_time_list, 99.9))
print("99pct:", np.percentile(tot_time_list, 99))
print("min:", np.min(tot_time_list))
print("median:", np.median(tot_time_list))
print("stdev:", np.std(tot_time_list))

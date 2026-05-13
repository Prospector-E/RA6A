#!/usr/bin/env python3
"""
One-time iPhone camera calibration.

Shows a live feed. Press SPACE to capture a frame when you see the
checkerboard outlined in green. Capture 15-20 images from different
angles/distances/positions, then press Q to compute calibration.

Saves camera_calibration.yaml for use by later vision code.

Usage:
  python3 calibrate_camera.py
"""

import cv2
import numpy as np
import yaml
import sys

# ---- Config ----
IPHONE_IP = '192.168.100.206'
URL = f'http://{IPHONE_IP}:4747/video'

CHECKERBOARD = (9, 6)        # inner corners (cols, rows)
SQUARE_SIZE_M = 0.026         # 26mm in meters
OUTPUT_FILE = 'camera_calibration.yaml'
MIN_CAPTURES = 15

# ---- Setup ----
# 3D points of checkerboard corners in board frame (Z=0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_M

obj_points = []   # 3D points in real world
img_points = []   # 2D points in image

print(f"Connecting to {URL}...")
cap = cv2.VideoCapture(URL)
if not cap.isOpened():
    print("Failed to open stream!")
    sys.exit(1)

print(f"\nCamera Calibration")
print(f"  Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners")
print(f"  Square size:  {SQUARE_SIZE_M*1000:.0f}mm")
print(f"\nMove the board to different positions and angles.")
print(f"  SPACE = capture (only when board outlined green)")
print(f"  Q     = quit & compute calibration")
print(f"  R     = remove last capture")
print(f"\nNeed at least {MIN_CAPTURES} captures.\n")

img_shape = None

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img_shape = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    display = frame.copy()
    if found:
        # Refine corner positions
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners_refined, found)
        status = "READY - press SPACE"
        color = (0, 255, 0)
    else:
        status = "Searching for board..."
        color = (0, 0, 255)
        corners_refined = None

    cv2.putText(display, f"{status}  |  Captures: {len(obj_points)}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(display, "SPACE=capture  Q=quit  R=remove last",
                (10, display.shape[0]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)

    cv2.imshow('Calibration', display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break
    elif key == ord(' ') and found and corners_refined is not None:
        obj_points.append(objp)
        img_points.append(corners_refined)
        print(f"  Captured #{len(obj_points)}")
    elif key == ord('r') and obj_points:
        obj_points.pop()
        img_points.pop()
        print(f"  Removed last. Total: {len(obj_points)}")

cap.release()
cv2.destroyAllWindows()

if len(obj_points) < MIN_CAPTURES:
    print(f"\nNot enough captures ({len(obj_points)}/{MIN_CAPTURES}). Aborting.")
    sys.exit(1)

print(f"\nComputing calibration from {len(obj_points)} images...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, img_shape, None, None)

# Re-projection error (lower = better; <0.5 is great, <1.0 is OK)
total_err = 0
for i in range(len(obj_points)):
    proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
    err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
    total_err += err
mean_err = total_err / len(obj_points)

print(f"\nCalibration RMS reprojection error: {ret:.4f} pixels")
print(f"Mean reprojection error:            {mean_err:.4f} pixels")
print(f"  (good: <0.5  acceptable: <1.0  poor: >1.0)")

print(f"\nCamera matrix:\n{mtx}")
print(f"\nDistortion coeffs:\n{dist.ravel()}")

data = {
    'image_width': img_shape[0],
    'image_height': img_shape[1],
    'camera_matrix': mtx.tolist(),
    'distortion_coefficients': dist.ravel().tolist(),
    'reprojection_error': float(ret),
    'num_captures': len(obj_points),
    'square_size_m': SQUARE_SIZE_M,
    'checkerboard_size': list(CHECKERBOARD),
}

with open(OUTPUT_FILE, 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print(f"\nSaved to {OUTPUT_FILE}")
print("Done!")

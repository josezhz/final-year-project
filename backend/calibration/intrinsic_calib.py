import numpy as np
import cv2 as cv
import glob
import json

# --- SETTINGS ---
CHECKERBOARD = (7, 5) 
SQUARE_SIZE = 0.239 / 8     # [m]
CAMERA_INDICES = [1, 2, 3]  # List of cameras to process

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Dictionary to store all calibration results
all_calibrations = {}

for cam_idx in CAMERA_INDICES:
    objpoints = [] 
    imgpoints = [] 
    images = glob.glob(f'backend/calibration/calib_img/cam{cam_idx}_*.jpg')

    if not images:
        print(f"No images found for Camera {cam_idx}. Skipping...")
        continue

    print(f"Processing {len(images)} images for Camera {cam_idx}...")

    img_size = None
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]

        ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

    # Run calibration
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    if ret:
        print(f"Calibration Successful for Camera {cam_idx} (RMS: {ret:.4f})")
        
        # Store in nested dictionary
        all_calibrations[f"cam{cam_idx}"] = {
            "camera_matrix": mtx.tolist(),
            "dist_coeff": dist.tolist(),
            "rms": ret,
            "resolution": img_size
        }
    else:
        print(f"Calibration failed for Camera {cam_idx}")

# --- SAVE ALL DATA TO ONE FILE ---
output_filename = "backend/calibration/camera_intrinsics.json"
with open(output_filename, "w") as f:
    json.dump(all_calibrations, f, indent=4)

print(f"\nAll camera parameters saved to {output_filename}")
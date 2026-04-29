import numpy as np
import cv2 as cv
import glob
import json

CHECKERBOARD = (9, 14) 
SQUARE_SIZE = 0.2685 / 15
CAMERA_INDICES = [1, 2, 3]

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

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

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    if ret:
        print(f"Calibration Successful for Camera {cam_idx} (RMS: {ret:.4f})")
        
        all_calibrations[f"cam{cam_idx}"] = {
            "camera_matrix": mtx.tolist(),
            "dist_coeff": dist.tolist(),
            "rms": ret,
            "resolution": img_size
        }
    else:
        print(f"Calibration failed for Camera {cam_idx}")

output_filename = "backend/calibration/camera_intrinsics.json"
with open(output_filename, "w") as f:
    json.dump(all_calibrations, f, indent=4)

print(f"\nAll camera parameters saved to {output_filename}")

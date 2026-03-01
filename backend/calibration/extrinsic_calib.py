import cv2 as cv
from pseyepy import Camera
import numpy as np
import json
from scipy.spatial.transform import Rotation as R

# --- SETTINGS ---
PARAMS_PATH = "MoCap/calibration/camera_intrinsics.json"
OUTPUT_PATH = "MoCap/calibration/camera_extrinsics.json"
MARKER_SIZE = 0.16  
DICT_TYPE = cv.aruco.DICT_4X4_1000
TARGET_CAPTURES = 3

# 1. Load Intrinsics
with open(PARAMS_PATH, "r") as f:
    calib_data = json.load(f)

obj_points = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
], dtype=np.float32)

detector = cv.aruco.ArucoDetector(cv.aruco.getPredefinedDictionary(DICT_TYPE), cv.aruco.DetectorParameters())

def calculate_average_pose(pose_list):
    """Averages a list of 4x4 matrices using Quaternions for rotation."""
    # Average Translation
    t_vecs = [p[:3, 3] for p in pose_list]
    avg_t = np.mean(t_vecs, axis=0)

    # Average Rotation
    rots = [R.from_matrix(p[:3, :3]) for p in pose_list]
    quats = [r.as_quat() for r in rots]
    avg_quat = np.mean(quats, axis=0)
    avg_quat /= np.linalg.norm(avg_quat)
    avg_R = R.from_quat(avg_quat).as_matrix()

    # Reconstruct 4x4
    avg_pose = np.eye(4)
    avg_pose[:3, :3] = avg_R
    avg_pose[:3, 3] = avg_t
    return avg_pose

# Storage for snapshots: {cam_index: [list_of_matrices]}
captured_data = {}

try:
    c = Camera(fps=30, resolution=Camera.RES_LARGE, colour=True)
    num_cams = len(c.ids)
    captured_data = {i: [] for i in range(num_cams)}

    print(f"Ready. Press 'S' to capture ({len(captured_data[0])}/{TARGET_CAPTURES})")

    while len(captured_data[0]) < TARGET_CAPTURES:
        frames, _ = c.read()
        if num_cams == 1: frames = [frames]
        
        current_frame_poses = {} # Temp store for current sync-frame

        for i, raw_frame in enumerate(frames):
            frame = raw_frame.copy()
            h, w = frame.shape[:2]
            
            cam_key = f"cam{i+1}"
            mtx = np.array(calib_data[cam_key]["camera_matrix"], dtype=np.float32)
            dist = np.array(calib_data[cam_key]["dist_coeff"], dtype=np.float32)

            # --- Apply Dynamic Scaling ---
            scale_factor = w / (mtx[0, 2] * 2) 
            if scale_factor != 1.0:
                mtx[0, 0] *= scale_factor
                mtx[1, 1] *= scale_factor
                mtx[0, 2] *= scale_factor
                mtx[1, 2] *= scale_factor

            corners, ids, _ = detector.detectMarkers(frame)

            if ids is not None:
                success, rvec, tvec = cv.solvePnP(obj_points, corners[0], mtx, dist)
                if success:
                    # Calculate T_cam_to_world
                    R_mat, _ = cv.Rodrigues(rvec)
                    T_m2c = np.eye(4)
                    T_m2c[:3, :3] = R_mat
                    T_m2c[:3, 3] = tvec.flatten()
                    T_c2w = np.linalg.inv(T_m2c)
                    
                    current_frame_poses[i] = T_c2w
                    
                    # Visual Feedback
                    cv.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.1)
                    x, y, z = T_c2w[:3, 3]
                    cv.putText(frame, f"Pos: {x:.2f}, {y:.2f}, {z:.2f}", (10, 30), 
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv.imshow(f"Cam {i+1}", frame)

        # Handle Input
        key = cv.waitKey(1) & 0xFF
        if key == ord('s'):
            if len(current_frame_poses) == num_cams:
                for idx, pose in current_frame_poses.items():
                    captured_data[idx].append(pose)
                print(f"Captured {len(captured_data[0])}/{TARGET_CAPTURES}")
            else:
                print("Warning: Not all cameras see the marker!")
        elif key == ord('q'):
            break

    # --- Process and Save Averages ---
    if len(captured_data[0]) == TARGET_CAPTURES:
        final_json = {}
        for i in range(num_cams):
            avg_pose = calculate_average_pose(captured_data[i])
            final_json[f"cam{i+1}"] = {
                "pose_matrix": avg_pose.tolist(),
                "position_xyz": avg_pose[:3, 3].tolist()
            }
        
        with open(OUTPUT_PATH, "w") as f:
            json.dump(final_json, f, indent=4)
        print(f"Successfully saved averages to {OUTPUT_PATH}")

    c.end()
    cv.destroyAllWindows()

except Exception as e:
    print(f"Error: {e}")
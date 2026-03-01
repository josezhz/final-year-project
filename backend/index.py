import cv2 as cv
from pseyepy import Camera
import numpy as np
import json
import math
from itertools import permutations
import asyncio
import websockets

# --- SETTINGS ---
INTRINSICS_PATH = "backend/calibration/camera_intrinsics.json"
EXTRINSICS_PATH = "backend/calibration/camera_extrinsics.json"
MIN_BRIGHTNESS = 50 
MAX_LEDS = 3 

# Physical drone model: Define Front, Back-Left, Back-Right in METERS
DRONE_LED_MODEL = np.array([
    [ 0.025,  0.00,  0.00], # Front LED
    [-0.025, -0.033,  0.00], # Back-Left LED
    [-0.025,  0.033,  0.00]  # Back-Right LED
], dtype=np.float32)

def load_json(path):
    with open(path, "r") as f: return json.load(f)

def triangulate_n_views(proj_mats, pts_2d):
    """Triangulates a 3D point from multiple camera views using DLT."""
    A = []
    for P, (u, v) in zip(proj_mats, pts_2d):
        A.append(u * P[2, :] - P[0, :])
        A.append(v * P[2, :] - P[1, :])
    A = np.array(A)
    _, _, Vt = np.linalg.svd(A)
    X = Vt[-1]
    return X[:3] / X[3]

def get_euler_angles(R):
    """Extracts Yaw, Pitch, and Roll from a rotation matrix."""
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        x, y, z = math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    else:
        x, y, z = math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0
    return np.degrees([z, y, x]) # Yaw, Pitch, Roll

# Load data
intrinsics_data = load_json(INTRINSICS_PATH)
extrinsics_data = load_json(EXTRINSICS_PATH)

try:
    c = Camera(fps=60, resolution=Camera.RES_LARGE, colour=False)
    num_cams = len(c.ids)
    
    # Pre-calculate Projection Matrices P = K * [R|t]
    proj_mats = []
    for i in range(num_cams):
        cam_key = f"cam{i+1}"
        K = np.array(intrinsics_data[cam_key]["camera_matrix"])
        T_c2w = np.array(extrinsics_data[cam_key]["pose_matrix"])
        # We need World-to-Camera, so invert the Cam-to-World pose
        T_w2c = np.linalg.inv(T_c2w)
        proj_mats.append(K @ T_w2c[:3, :])

    print("Tracking active. Press 'q' to quit.")

    while True:
        frames, _ = c.read()
        if num_cams == 1: frames = [frames]
        
        all_cam_leds = []
        for i, raw_frame in enumerate(frames):
            frame = raw_frame.copy()
            _, mask = cv.threshold(frame, MIN_BRIGHTNESS, 255, cv.THRESH_BINARY)
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            leds = []
            for cnt in contours:
                M = cv.moments(cnt)
                if M["m00"] > 2: # Basic noise filter
                    leds.append((M["m10"]/M["m00"], M["m01"]/M["m00"]))
            
            # Keep only top 3 brightest/largest detections
            all_cam_leds.append(leds[:MAX_LEDS])
            cv.imshow(f"Cam {i+1} Raw", frame)

        # Only proceed if every camera sees exactly 3 LEDs
        if all(len(l) == MAX_LEDS for l in all_cam_leds):
            best_error, best_pose = float('inf'), None

            # Test permutations to find which LED correspondence gives best fit
            for p2 in permutations(range(MAX_LEDS)):
                for p3 in permutations(range(MAX_LEDS)):
                    candidate_3d = []
                    for idx in range(MAX_LEDS):
                        pts_2d = [all_cam_leds[0][idx], all_cam_leds[1][p2[idx]], all_cam_leds[2][p3[idx]]]
                        candidate_3d.append(triangulate_n_views(proj_mats, pts_2d))
                    
                    candidate_3d = np.array(candidate_3d, dtype=np.float32)
                    
                    # Use Umeyama or similar to find rigid transformation
                    # Centroid alignment
                    centroid_model = np.mean(DRONE_LED_MODEL, axis=0)
                    centroid_data = np.mean(candidate_3d, axis=0)
                    
                    model_centered = DRONE_LED_MODEL - centroid_model
                    data_centered = candidate_3d - centroid_data
                    
                    # Compute rotation using SVD
                    H = model_centered.T @ data_centered
                    U, S, Vt = np.linalg.svd(H)
                    R = Vt.T @ U.T
                    
                    # Handle reflection case
                    if np.linalg.det(R) < 0:
                        Vt[-1, :] *= -1
                        R = Vt.T @ U.T
                    
                    # Compute translation
                    t = centroid_data - R @ centroid_model
                    
                    # Calculate fitting error
                    transformed = (R @ DRONE_LED_MODEL.T).T + t
                    err = np.mean(np.linalg.norm(candidate_3d - transformed, axis=1))
                    
                    if err < best_error:
                        best_error = err
                        best_pose = (R, t)

            # Accept the pose if the error is low (e.g., < 5cm)
            if best_pose and best_error < 0.05:
                R, t = best_pose
                yaw, pitch, roll = get_euler_angles(R)
                print(f"XYZ: {t} | YPR: {yaw:.1f}, {pitch:.1f}, {roll:.1f} | Error: {best_error:.4f}")

        if cv.waitKey(1) & 0xFF == ord('q'): break

    c.end()
    cv.destroyAllWindows()
except Exception as e:
    print(f"Error: {e}")

async def mocap_server(websocket):
    print("React client connected")
    # Initialize camera INSIDE the handler or pass it in
    try:
        c = Camera(fps=60, resolution=Camera.RES_LARGE, colour=False)
        num_cams = len(c.ids)
        
        # Pre-calculate Projection Matrices (Logic from your script)
        # ... [Insert your Projection Matrix Logic Here] ...

        while True:
            read_result = c.read()
            if not read_result or len(read_result) < 2:
                await asyncio.sleep(0.01)
                continue
            
            frames, _ = read_result
            if num_cams == 1: frames = [frames]
            
            # ... [Insert your LED detection & best_pose logic here] ...

            if best_pose and best_error < 0.05:
                R, t = best_pose
                yaw, pitch, roll = get_euler_angles(R)
                
                payload = {
                    "position": {"x": round(float(t[0]), 4), "y": round(float(t[1]), 4), "z": round(float(t[2]), 4)},
                    "rotation": {"yaw": round(float(yaw), 2), "pitch": round(float(pitch), 2), "roll": round(float(roll), 2)},
                    "error": round(float(best_error), 5)
                }
                await websocket.send(json.dumps(payload))
            
            await asyncio.sleep(1/60) # Match camera FPS

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    except Exception as e:
        print(f"Tracking Error: {e}")
    finally:
        c.end()

async def main():
    # Start the server on localhost:8765
    async with websockets.serve(mocap_server, "localhost", 8765):
        print("Mocap Server running on ws://localhost:8765")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
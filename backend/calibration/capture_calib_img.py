import cv2 as cv
import os
from pseyepy import Camera

# --- CONFIGURATION ---
SAVE_DIR = "backend/calibration/calib_img"
# MUST match the inner corners of your board (squares minus 1)
CHESSBOARD_SIZE = (7, 5) 

if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)


def remove_stale_images(camera_labels, latest_count):
    """Trim old calibration images beyond the last snapshot saved this session."""
    if latest_count <= 0:
        return

    for label in camera_labels.values():
        prefix = f"cam{label}_"
        for filename in os.listdir(SAVE_DIR):
            if not (filename.startswith(prefix) and filename.endswith(".jpg")):
                continue

            suffix = filename[len(prefix):-4]
            if not suffix.isdigit():
                continue

            if int(suffix) > latest_count:
                os.remove(os.path.join(SAVE_DIR, filename))


def delete_last_saved_images(camera_labels, latest_count):
    """Delete the most recent snapshot for every labeled camera."""
    if latest_count <= 0:
        return 0

    deleted = 0
    for label in camera_labels.values():
        filename = f"cam{label}_{latest_count}.jpg"
        filepath = os.path.join(SAVE_DIR, filename)
        if os.path.exists(filepath):
            os.remove(filepath)
            deleted += 1

    return deleted


def main():
    try:
        print("Initializing cameras for labeling...")
        c = Camera(fps=30, resolution=Camera.RES_LARGE, colour=True)
        num_cams = len(c.ids)
        
        if num_cams == 0:
            print("No cameras detected.")
            return

        # --- STEP 1: MANUALLY LABEL EACH CAMERA ---
        camera_labels = {}
        for i in range(num_cams):
            while True:
                frames, _ = c.read()
                frame = frames[i] if isinstance(frames, list) else frames
                display_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
                
                cv.putText(display_frame, f"WHICH CAMERA IS THIS? (ID: {i})", (20, 50), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv.imshow("Labeling Mode", display_frame)
                cv.waitKey(1)
                
                label = input(f"Enter label for Camera Index {i}: ").strip()
                if label:
                    camera_labels[i] = label
                    break
        cv.destroyWindow("Labeling Mode")

        # --- STEP 2: CAPTURE SESSION WITH PATTERN RECOGNITION ---
        print("\n--- Capture Mode ---")
        print("Controls: [Space] to Save, [D] to Delete Last Saved, [Q] to Quit")

        count = 0
        while True:
            frames, _ = c.read()
            if num_cams == 1: frames = [frames]

            for i in range(num_cams):
                frame_bgr = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)
                gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
                
                # --- PATTERN RECOGNITION ---
                # Find corners to show user if the board is clear enough
                ret, corners = cv.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
                
                if ret:
                    # Draw the pattern on the screen (visual feedback only)
                    cv.drawChessboardCorners(frame_bgr, CHESSBOARD_SIZE, corners, ret)
                    status_color = (0, 255, 0) # Green if found
                else:
                    status_color = (0, 0, 255) # Red if not found

                # Add status text to each window
                lbl = camera_labels[i]
                cv.putText(frame_bgr, f"Cam {lbl} - Pattern: {ret}", (10, 30), 
                           cv.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
                
                cv.imshow(f"Camera {lbl}", frame_bgr)

            key = cv.waitKey(1) & 0xFF
            
            if key == ord(' '):  # SPACE to save
                count += 1
                for i in range(num_cams):
                    lbl = camera_labels[i]
                    filename = f"cam{lbl}_{count}.jpg"
                    filepath = os.path.join(SAVE_DIR, filename)
                    # We save the original frame (without the lines drawn on it)
                    img_to_save = cv.cvtColor(frames[i], cv.COLOR_RGB2BGR)
                    cv.imwrite(filepath, img_to_save)
                print(f"Saved snapshot #{count} (Ensure pattern was detected for best results!)")

            elif key == ord('d'):
                deleted = delete_last_saved_images(camera_labels, count)
                if deleted:
                    print(f"Deleted snapshot #{count}")
                    count -= 1
                else:
                    print("No saved snapshot to delete.")

            elif key == ord('q'):
                break

        remove_stale_images(camera_labels, count)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'c' in locals():
            c.end()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()

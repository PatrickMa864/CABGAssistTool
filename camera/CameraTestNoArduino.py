import cv2
import numpy as np

# ----------------------
#   USER CONFIG
# ----------------------
# Known real-world diameter of the circle (in mm).
KNOWN_DIAMETER_MM = 30

# Known distance for calibration (mm), e.g., 45 cm => 450 mm
CALIBRATION_DISTANCE_MM = 450.0

# Camera index (for your Arducam)
CAMERA_INDEX = 1

# Desired capture properties
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 100

# HSV color range(s) for DARK BLUE (example)
LOWER_COLOR_1 = np.array([90, 150, 50])
UPPER_COLOR_1 = np.array([110, 255, 255])
LOWER_COLOR_2 = np.array([110, 150, 50])
UPPER_COLOR_2 = np.array([130, 255, 255])

# Morphological kernel for noise reduction
KERNEL = np.ones((5, 5), np.uint8)

# ----------------------------------------------------
# WORLD REFERENCE FRAME (WRF) TRANSLATION
# Adjust for your actual setup
# ----------------------------------------------------
WRF_OFFSET_X = 5.0
WRF_OFFSET_Y = 5.0
WRF_OFFSET_Z = 5.0


def main():
    # 1) Open your camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Failed to open camera at index {CAMERA_INDEX}.")
        return

    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Camera settings: {int(width)}x{int(height)} at {int(fps)} FPS")
    print("[INFO] Press 'c' to calibrate (ensure circle is at ~45 cm).")
    print("[INFO] Press 'q' to quit.")

    # We'll store the focal length once we calibrate
    focal_length = None

    # Approximate principal point at the center of the image
    cx = width / 2.0
    cy = height / 2.0

    # -- Store the last known positions so we can display them even if detection fails --
    last_cam_coords = None  # (X_cm_cam, Y_cm_cam, Z_cm_cam)
    last_world_coords = None  # (X_wrf, Y_wrf, Z_wrf)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to grab frame.")
            break

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the designated color
        mask1 = cv2.inRange(hsv, LOWER_COLOR_1, UPPER_COLOR_1)
        mask2 = cv2.inRange(hsv, LOWER_COLOR_2, UPPER_COLOR_2)
        mask = mask1 + mask2

        # Morphological ops to reduce noise
        mask = cv2.erode(mask, KERNEL, iterations=1)
        mask = cv2.dilate(mask, KERNEL, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # By default, do not update coords this frame unless we have a detection
        found_coords = False

        if contours:
            # Pick the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # fitEllipse requires at least 5 points
            if len(largest_contour) >= 5:
                ellipse = cv2.fitEllipse(largest_contour)
                # ellipse = ((x_center, y_center), (major_axis, minor_axis), angle)
                (x_center, y_center), (major_axis, minor_axis), angle = ellipse

                # Draw the ellipse
                cv2.ellipse(frame, ellipse, (0, 255, 0), 2)
                center = (int(x_center), int(y_center))
                cv2.circle(frame, center, 3, (0, 255, 0), -1)

                # Estimate "diameter" in pixels as the average of major and minor axes
                diameter_pixels = 0.5 * (major_axis + minor_axis)

                # Filter out very small ellipses
                if diameter_pixels > 10:
                    if focal_length is not None:
                        # Z in mm via pinhole
                        distance_mm = (KNOWN_DIAMETER_MM * focal_length) / diameter_pixels
                        distance_cm = distance_mm / 10.0

                        # X and Y in mm using the pinhole model
                        x_shifted = (x_center - cx)
                        y_shifted = (y_center - cy)
                        X_mm = (x_shifted / focal_length) * distance_mm
                        Y_mm = (y_shifted / focal_length) * distance_mm

                        # Convert to cm
                        X_cm_cam = X_mm / 10.0
                        Y_cm_cam = Y_mm / 10.0
                        Z_cm_cam = distance_cm  # from camera center

                        # Apply WRF offsets
                        X_wrf = X_cm_cam + WRF_OFFSET_X
                        Y_wrf = Y_cm_cam + WRF_OFFSET_Y
                        Z_wrf = Z_cm_cam + WRF_OFFSET_Z

                        # Update last known coords
                        last_cam_coords = (X_cm_cam, Y_cm_cam, Z_cm_cam)
                        last_world_coords = (X_wrf, Y_wrf, Z_wrf)

                        found_coords = True

        # -------------------------
        # Overlay text on the frame
        # -------------------------

        # If we have known coordinates, display them
        if last_cam_coords is not None:
            (xc, yc, zc) = last_cam_coords
            cam_text = f"Camera (cm): X={xc:.1f}, Y={yc:.1f}, Z={zc:.1f}"
            cv2.putText(frame, cam_text, (10, 30),  # top-left corner
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if last_world_coords is not None:
            (xw, yw, zw) = last_world_coords
            wrf_text = f"World (cm):  X={xw:.1f}, Y={yw:.1f}, Z={zw:.1f}"
            cv2.putText(frame, wrf_text, (10, 60),  # below camera coords
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Show the frame
        cv2.imshow("Arducam (World Reference Frame)", frame)

        # Key handling
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # Calibrate (assuming object is ~45 cm from camera)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if len(largest_contour) >= 5:
                    ellipse = cv2.fitEllipse(largest_contour)
                    (x_center, y_center), (major_axis, minor_axis), angle = ellipse
                    diameter_pixels = 0.5 * (major_axis + minor_axis)

                    if diameter_pixels > 10:
                        # focal_length => (perceived_pixels * known_distance_mm) / real_world_mm
                        focal_length = (diameter_pixels * CALIBRATION_DISTANCE_MM) / KNOWN_DIAMETER_MM
                        print(f"[CALIBRATION] Focal length set to {focal_length:.2f}")
                    else:
                        print("[CALIBRATION] Ellipse too small. Move closer.")
                else:
                    print("[CALIBRATION] Not enough points to fit an ellipse. Try again.")
            else:
                print("[CALIBRATION] No object found for calibration.")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

import cv2
import numpy as np
import serial
import time

# ----------------------
#   USER CONFIG
# ----------------------
SERIAL_PORT = 'COM3'  # Change to your Arduino port, e.g. '/dev/ttyACM0' on Linux
BAUD_RATE = 9600

# Known real-world diameter of the red dot (in mm).
KNOWN_DIAMETER_MM = 16.5

# Known distance (in mm) used for calibration, e.g., 30 cm => 300 mm
CALIBRATION_DISTANCE_MM = 300.0

# Camera index (adjust as needed for your system)
CAMERA_INDEX = 1  # Arducam at index 1

# Desired capture properties
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 100

# HSV color range(s) for RED
LOWER_RED_1 = np.array([0, 120, 70])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 120, 70])
UPPER_RED_2 = np.array([180, 255, 255])

# Morphological kernel for noise reduction
KERNEL = np.ones((5, 5), np.uint8)

# Distance thresholds (in cm)
MIN_DISTANCE_CM = 10
MAX_DISTANCE_CM = 20

def main():
    # 1) Open Serial to Arduino
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # wait for Arduino to reset
        print(f"[INFO] Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"[ERROR] Could not open serial port {SERIAL_PORT}: {e}")
        return

    # 2) Open Arducam
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
    print(f"Camera settings: {int(width)}x{int(height)} at {int(fps)} FPS")
    print("Press 'c' to calibrate (ensure red dot is at 30 cm).")
    print("Press 'q' to quit.")

    # Store the focal length once we calibrate
    focal_length = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to grab frame. Check camera connection.")
            break

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create the red color mask
        mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
        mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)
        mask = mask1 + mask2

        # Morphological ops to reduce noise
        mask = cv2.erode(mask, KERNEL, iterations=1)
        mask = cv2.dilate(mask, KERNEL, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        distance_cm = None

        if contours:
            # Grab largest contour (assuming that's our red dot)
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            # Filter out extremely small circles
            if radius > 5:
                center = (int(x), int(y))
                diameter_pixels = 2 * radius

                # Draw a circle on the detected red dot
                cv2.circle(frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(frame, center, 3, (0, 255, 0), -1)  # center dot

                # If calibrated, compute distance
                if focal_length is not None:
                    distance_mm = (KNOWN_DIAMETER_MM * focal_length) / diameter_pixels
                    distance_cm = distance_mm / 10.0
                    cv2.putText(frame,
                                f"Distance: {distance_cm:.2f} cm",
                                (center[0] + 10, center[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # If we have a valid distance, decide whether to turn LED ON/OFF
        if distance_cm is not None:
            if MIN_DISTANCE_CM <= distance_cm <= MAX_DISTANCE_CM:
                # in range
                ser.write(b'1')  # send '1' to Arduino
            else:
                # out of range
                ser.write(b'0')  # send '0' to Arduino
        else:
            # No red dot found -> LED OFF
            ser.write(b'0')

        # Show the frame
        cv2.imshow("Arducam (Calibration + Distance + Arduino LED)", frame)

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # Calibrate using the largest red dot in view
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                if radius > 5:
                    diameter_pixels = 2 * radius
                    # focal_length = (perceived_width_in_pixels * known_distance) / real_world_width
                    focal_length = (diameter_pixels * CALIBRATION_DISTANCE_MM) / KNOWN_DIAMETER_MM
                    print(f"[CALIBRATION] Focal length set to {focal_length:.2f}")
                else:
                    print("[CALIBRATION] Red dot radius too small. Move the dot closer.")
            else:
                print("[CALIBRATION] No red dot found. Please place the dot in view.")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("[INFO] Exiting program.")

if __name__ == "__main__":
    main()

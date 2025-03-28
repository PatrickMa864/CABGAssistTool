import serial 
import numpy as np
import math
import time
import cv2
import threading
import matplotlib

###############################################################################
#                   USER CONFIG FOR CAMERA
###############################################################################
KNOWN_DIAMETER_MM = 10#15#30
CALIBRATION_DISTANCE_MM = 500.0  # mm
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 30

# HSV thresholds for red
LOWER_COLOR_1 = np.array([0, 100, 100])
UPPER_COLOR_1 = np.array([10, 255, 255])
LOWER_COLOR_2 = np.array([150, 100, 100])
UPPER_COLOR_2 = np.array([180, 255, 255])
KERNEL = np.ones((5, 5), np.uint8)

###############################################################################
#                   USER CONFIG FOR ROBOT SERIAL
###############################################################################
SERIAL_PORT = 'COM9'
SERIAL_BAUD = 115200

###############################################################################
#                   GLOBAL VARIABLES
###############################################################################
camera_displacement_z = 0.0   # measured displacement (mm)
focal_length = None           # computed on the fly via calibration
running = True                # main loop flag

# Fixed reference points
heartPt = np.array([347.5, 279.7, 175])  # known dot starting point on heart
dotPt = heartPt.copy()                   # initially zero displacement

# Calibration variables
dot_calibrated = False        # flag indicating calibration done
z_offset_mm = 0.0             # baseline Z from calibration
calibrating = False           # flag for calibration mode
calibration_buffer = []       # to accumulate several Z measurements
calibration_frames = 5        # number of frames to average

# Sending rate (seconds)
send_interval = 0.1  
last_send_time = time.time()

def serial_reader(ser):
    """Background thread to read measurement data from Arduino."""
    global running
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode().strip()
                if line.startswith("M:"):
                    print("Measurement:", line)
            except Exception as e:
                print("Serial read error:", e)
        time.sleep(0.01)

def main():
    global camera_displacement_z, focal_length, running, dotPt, dot_calibrated, z_offset_mm
    global calibrating, calibration_buffer, last_send_time

    # ---------------------------
    # Open Serial Port
    # ---------------------------
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        print(f"[INFO] Opened serial {SERIAL_PORT} at {SERIAL_BAUD}")
    except Exception as e:
        print(f"[ERROR] Could not open serial: {e}")
        return

    # Start background thread for reading Arduino measurements
    reader_thread = threading.Thread(target=serial_reader, args=(ser,))
    reader_thread.daemon = True
    reader_thread.start()

    # ---------------------------
    # Open Camera
    # ---------------------------
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Failed to open camera index {CAMERA_INDEX}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)
    print("[INFO] Press 'c' to calibrate camera (set Z=0), 'z' to zero the gyro, 'q' to quit.")

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cx = width / 2.0
    cy = height / 2.0
    print(f"[INFO] Camera resolution: {int(width)}x{int(height)}")

    while running:
        ret, frame = cap.read()
        if not ret:
            print("[WARNING] Failed to grab frame.")
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_COLOR_1, UPPER_COLOR_1)
        mask2 = cv2.inRange(hsv, LOWER_COLOR_2, UPPER_COLOR_2)
        mask = mask1 + mask2
        mask = cv2.erode(mask, KERNEL, iterations=1)
        mask = cv2.dilate(mask, KERNEL, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Default measured Z (mm)
        Z_mm = 0.0
        diameter_pixels = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if len(largest) >= 5:
                ellipse = cv2.fitEllipse(largest)
                (xc, yc), (major, minor), angle = ellipse
                diameter_pixels = 0.5 * (major + minor)
                # Optional: draw detection for visualization
                cv2.ellipse(frame, ellipse, (0,255,0), 2)
                cv2.circle(frame, (int(xc), int(yc)), 3, (0,255,0), -1)
                if focal_length is not None and diameter_pixels > 10:
                    distance_mm = (KNOWN_DIAMETER_MM * focal_length) / diameter_pixels
                    y_shifted = (yc - cy)
                    Y_mm = (y_shifted / focal_length) * distance_mm
                    Z_mm = -Y_mm  # Invert axis if needed

        # Calibration: accumulate a few Z measurements if in calibration mode.
        if calibrating and (diameter_pixels > 10):
            calibration_buffer.append(Z_mm)
            if len(calibration_buffer) >= calibration_frames:
                z_offset_mm = np.mean(calibration_buffer)
                dotPt = heartPt.copy()
                camera_displacement_z = 0.0
                dot_calibrated = True
                calibrating = False
                calibration_buffer = []
                print(f"[CALIBRATION] Done. Baseline set to {z_offset_mm:.2f} mm. Dot at {dotPt}")

        # If calibrated, update displacement:
        if dot_calibrated:
            camera_displacement_z = Z_mm - z_offset_mm
            dotPt = heartPt.copy()
            dotPt[2] += camera_displacement_z

        # Display overlay
        if focal_length is not None:
            disp_text = f"Dot Z: {camera_displacement_z:.2f} mm"
            cv2.putText(frame, disp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow("Camera Feed", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            running = False
        elif key == ord('c'):
            if contours and len(largest) >= 5 and diameter_pixels > 10:
                focal_length = (diameter_pixels * CALIBRATION_DISTANCE_MM) / KNOWN_DIAMETER_MM
                print(f"[CALIBRATION] focal_length set to {focal_length:.2f}")
                calibrating = True
                calibration_buffer = []
            else:
                print("[CALIBRATION] Circle not found or too small.")
        elif key == ord('z'):
            ser.write(b'z\n')
            print("[INFO] Sent 'z' to Arduino for gyro zeroing.")

        # Send dotPt at fixed intervals
        if time.time() - last_send_time >= send_interval:
            dotPt_str = f"{dotPt[0]},{dotPt[1]},{dotPt[2]}\n"
            ser.write(dotPt_str.encode())
            last_send_time = time.time()

    # Cleanup
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Exiting main function")

if __name__ == "__main__":
    main()

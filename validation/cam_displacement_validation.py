import serial
import numpy as np
import math
import time
import cv2
import threading
import matplotlib
import matplotlib.pyplot as plt
import csv
import os

###############################################################################
#                   USER CONFIG FOR CAMERA
###############################################################################
KNOWN_DIAMETER_MM = 10
CALIBRATION_DISTANCE_MM = 500.0  # mm
CAMERA_INDEX = 1
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
SERIAL_PORT = 'COM3'
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

# Data collection control
collecting_data = False       # <--- NEW: Only collect after pressing 'c' & calibrating

# Sending rate (seconds)
send_interval = 0.1
last_send_time = time.time()

###############################################################################
#                   IDEAL DATA
###############################################################################
theta_degrees = np.array([
    0, 15, 30, 45, 60, 75, 82, 90, 97, 105, 120, 135,
    150, 165, 180, 195, 210, 225, 240, 255, 262,
    270, 277, 285, 300, 315, 330, 345, 360
])
# Corresponding r-values (in mm):
r = np.array([
    11.994, 12.336, 13.445, 15.620, 19.485, 27.322,
    31.465, 32.964, 31.465, 27.322, 19.485,
    15.620, 13.445, 12.336, 11.994,
    12.336, 13.445, 15.620,
    19.485, 27.322,
    31.465,
    32.964,
    31.465,
    27.322,
    19.485,
    15.620,
    13.445,
    12.336,
    11.994
])

f_ideal = 0.1285           # frequency
T_ideal = 1.0 / f_ideal    # ~7.78 seconds per cycle

def main():
    global camera_displacement_z, focal_length, running, dotPt
    global dot_calibrated, z_offset_mm, calibrating, calibration_buffer
    global collecting_data

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
    print("[INFO] Press 'c' to calibrate (and start collecting data), 'q' to quit.")

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cx = width / 2.0
    cy = height / 2.0
    print(f"[INFO] Camera resolution: {int(width)}x{int(height)}")

    # -----------------------------------------------------
    # Data collection arrays (time and z-displacement)
    # -----------------------------------------------------
    time_data = []
    z_data = []
    start_time = None  # We'll set this once we actually start collecting

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
                # If focal_length is known and circle is large enough
                if focal_length is not None and diameter_pixels > 10:
                    distance_mm = (KNOWN_DIAMETER_MM * focal_length) / diameter_pixels
                    y_shifted = (yc - cy)
                    Y_mm = (y_shifted / focal_length) * distance_mm
                    Z_mm = -Y_mm  # Invert axis if needed

        # Calibration: accumulate a few Z measurements if in calibration mode
        if calibrating and (diameter_pixels > 10):
            calibration_buffer.append(Z_mm)
            if len(calibration_buffer) >= calibration_frames:
                # Once we have enough frames, we define the offset:
                z_offset_mm = np.mean(calibration_buffer)
                dotPt = heartPt.copy()
                camera_displacement_z = 0.0
                dot_calibrated = True
                calibrating = False
                calibration_buffer = []
                print(f"[CALIBRATION] Done. Baseline set to {z_offset_mm:.2f} mm. Dot at {dotPt}")

                # -----------------------------------------------------
                # START COLLECTING DATA NOW THAT WE'RE CALIBRATED
                # -----------------------------------------------------
                collecting_data = True
                # Reset your time arrays so they start at 0
                time_data = []
                z_data = []
                start_time = time.time()
                print("[INFO] Calibration complete. Data collection started. Press 'q' to quit.")

        # If calibrated, compute the actual displacement
        if dot_calibrated:
            camera_displacement_z = Z_mm - z_offset_mm
            dotPt = heartPt.copy()
            dotPt[2] += camera_displacement_z
        else:
            # If not calibrated, there's no displacement offset
            camera_displacement_z = 0.0

        # Display overlay
        disp_text = f"Dot Z: {camera_displacement_z:.2f} mm" if dot_calibrated else "Not calibrated"
        cv2.putText(frame, disp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0,0,255), 2)
        cv2.imshow("Camera Feed", frame)
        key = cv2.waitKey(1) & 0xFF

        # -----------------------------------------------------
        # Only collect data if collecting_data == True
        # -----------------------------------------------------
        if collecting_data and start_time is not None:
            current_time = time.time() - start_time
            time_data.append(current_time)
            z_data.append(camera_displacement_z)

        # Keyboard commands
        if key == ord('q'):
            running = False
        elif key == ord('c'):
            # Attempt to set focal_length if we see a valid circle
            if contours and len(largest) >= 5 and diameter_pixels > 10:
                focal_length = (diameter_pixels * CALIBRATION_DISTANCE_MM) / KNOWN_DIAMETER_MM
                print(f"[CALIBRATION] focal_length set to {focal_length:.2f}")
                calibrating = True
                calibration_buffer = []
            else:
                print("[CALIBRATION] Circle not found or too small. Move or refocus.")

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Exiting main function")

    # -----------------------------------------------------
    # Save the measured data to a CSV file (if any)
    # -----------------------------------------------------
    if len(time_data) > 0:
        csv_filename = "z_displacement_data.csv"
        with open(csv_filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Time (s)", "Z displacement (mm)"])
            for t, z in zip(time_data, z_data):
                writer.writerow([t, z])
        print(f"[INFO] Data saved to {os.path.abspath(csv_filename)}")
    else:
        print("[WARNING] No data was collected (did you calibrate?). CSV not saved.")

    # -----------------------------------------------------
    # Plot: Measured vs. Ideal on the same figure
    # -----------------------------------------------------
    if len(time_data) > 0:
        import math

        time_data = np.array([float(t) for t in time_data])
        z_data = np.array([float(z) for z in z_data])
        T_measured = time_data[-1]

        # Shift the original "ideal" r array so minimum is 0
        r_zeroed = r - np.min(r)

        # Single-cycle time array
        time_one_cycle = np.linspace(0, T_ideal, len(r_zeroed))

        # Number of cycles needed
        num_cycles = max(1, math.ceil(T_measured / T_ideal))

        wave_time_list = []
        wave_r_list = []
        for i in range(num_cycles):
            offset = i * T_ideal
            wave_time_list.append(time_one_cycle + offset)
            wave_r_list.append(r_zeroed)
        wave_time_full = np.concatenate(wave_time_list)
        wave_r_full = np.concatenate(wave_r_list)

        # Trim wave to the camera's total time
        mask = wave_time_full <= T_measured
        wave_time_final = wave_time_full[mask]
        wave_r_final = wave_r_full[mask]

        # Plot both
        plt.figure()
        plt.plot(time_data, z_data, label="Measured Z displacement (camera)")
        plt.plot(wave_time_final, wave_r_final, label="Ideal wave (shifted to min=0)")

        plt.xlabel("Time (s)")
        plt.ylabel("Displacement (mm)")
        plt.title("Comparison: Measured vs. Ideal (Multiple Cycles)")
        plt.legend()
        plt.show()
    else:
        print("[WARNING] No data to plot.")

if __name__ == "__main__":
    main()

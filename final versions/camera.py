import serial 
import numpy as np
import math
import time
import cv2
import threading
import matplotlib.pyplot as plt

###############################################################################
#                   USER CONFIG FOR CAMERA
###############################################################################
KNOWN_DIAMETER_MM = 10  # 10 mm (or update as needed)
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

###############################################################################z
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

# Sending rate (seconds)
send_interval = 0.1  
last_send_time = time.time()

# Global variables for plot data collection
plot_collecting = False      # becomes True once 'p' is pressed
start_time_plot = None       # will be set when plotting is activated
measurement_times = []         # time (s) relative to start_time_plot
measurement_dist_tool_dot = [] # from Arduino: distance from tool to dot
measurement_pot_dist = []      # potentiometer distance (mm)
measurement_desired = []       # desired distance (mm)
measurement_heart = []

def parse_measurement_line(line):
    """
    Parse a measurement line from Arduino.
    Expected format:
    M: yaw=..., roll=..., pitch=..., dist_tool_dot=..., PWM=..., error=..., pot_dist=..., Desired_dist=...
    Returns a dictionary with keys: 'dist_tool_dot', 'pot_dist', 'Desired_dist'
    """
    line = line.replace("M:", "").strip()
    parts = line.split(',')
    data = {}
    for part in parts:
        if '=' in part:
            key, val = part.split('=')
            key = key.strip()
            try:
                data[key] = float(val.strip())
            except ValueError:
                pass
    return data

def serial_reader(ser):
    """Background thread to read measurement data from Arduino.
       If plot collection is active, records the measurement values.
    """
    global running, measurement_times, measurement_dist_tool_dot, measurement_pot_dist, measurement_desired, start_time_plot, plot_collecting
    while running:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode().strip()
                # Always print the measurement line for console monitoring
                if line.startswith("M:"):
                    print("Measurement:", line)
                    # If plot collection is enabled, record the data
                    if plot_collecting:
                        data = parse_measurement_line(line)
                        if 'dist_tool_dot' in data and 'pot_dist' in data and 'Desired_dist' in data:
                            current_time = time.time() - start_time_plot  # relative time for plot
                            measurement_times.append(current_time)
                            measurement_dist_tool_dot.append(data['dist_tool_dot'])
                            measurement_pot_dist.append(data['pot_dist'])
                            measurement_desired.append(data['Desired_dist'])
                            measurement_heart.append(camera_displacement_z)
            except Exception as e:
                print("Serial read error:", e)
        time.sleep(0.01)

def plot_data():
    """Plot the recorded measurement data."""
    if not measurement_times:
        print("[PLOT] No measurement data was collected.")
        return

    plt.figure()

    measurement_heart_np = np.array(measurement_heart)
    measurement_pot_np = np.array(measurement_pot_dist)
    measurement_pot_flipped = -measurement_pot_np

# Plot the data
    plt.plot(measurement_times, measurement_heart_np, label='Heart displacement')
    plt.plot(measurement_times, measurement_pot_flipped+60 - measurement_heart_np/1.9415, label='Actual Distance Between Tool and Dot')
    plt.plot(measurement_times, measurement_pot_np, label='Potentiometer Displacement')
    plt.plot(measurement_times, measurement_desired, label='Desired Distance')
    plt.plot()
    plt.xlabel("Time (s)")
    plt.ylabel("Displacement (mm)")
    plt.title("Measurement Data")
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    global camera_displacement_z, focal_length, running, dotPt, dot_calibrated, z_offset_mm
    global calibrating, calibration_buffer, last_send_time, plot_collecting, start_time_plot

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
    print("[INFO] Press 'c' to calibrate camera (set Z=0), 'z' to zero the gyro, 'p' to start collecting plot data, 'q' to quit and show final plot.")

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
                    Z_mm = -Y_mm  # Invert axis if neededq

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

        # Display overlay on the camera feed
        if focal_length is not None:
            disp_text = f"Dot Z: {camera_displacement_z:.2f} mm"
            cv2.putText(frame, disp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.imshow("Camera Feed", frame)
        key = cv2.waitKey(1) & 0xFF

        # Check key presses
        if key == ord('q'):
            running = False  # This will exit the loop and stop data collection
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
        elif key == ord('p'):
            if not plot_collecting:
                plot_collecting = True
                start_time_plot = time.time()  # record the time when plot data collection starts
                # Clear any previous data if needed
                measurement_times.clear()
                measurement_dist_tool_dot.clear()
                measurement_pot_dist.clear()
                measurement_desired.clear()
                print("[PLOT] Started collecting data for plotting.")

        # Send dotPt at fixed intervals
        if time.time() - last_send_time >= send_interval:
            dotPt_str = f"{dotPt[0]},{dotPt[1]},{dotPt[2]}\n"
            ser.write(dotPt_str.encode())
            last_send_time = time.time()

    # Cleanup after loop ends
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Exiting main function")
    
    # If plotting was enabled, display the final plot
    if plot_collecting:
        print("[PLOT] Displaying final plot.")
        plot_data()

if __name__ == "__main__":
    main()

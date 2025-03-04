import serial
import numpy as np
import math
import time
import cv2
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

###############################################################################
#                   USER CONFIG FOR CAMERA
###############################################################################
# Real-world diameter of the red circle in mm
KNOWN_DIAMETER_MM = 30

# Distance used for camera calibration, in mm
CALIBRATION_DISTANCE_MM = 370.0#373.75

# Camera index
CAMERA_INDEX = 0

# Desired capture properties
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 30

# HSV thresholds for red
LOWER_COLOR_1 = np.array([0, 120, 70])
UPPER_COLOR_1 = np.array([10, 255, 255])
LOWER_COLOR_2 = np.array([170, 120, 70])
UPPER_COLOR_2 = np.array([180, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)


###############################################################################
#                   USER CONFIG FOR ROBOT SERIAL
###############################################################################
SERIAL_PORT = 'COM9'
SERIAL_BAUD = 115200

###############################################################################
#                   GLOBAL VARIABLESS
###############################################################################
# We'll store the "vertical displacement" from the camera here,
# so we can update the dot in the robot figure. This is updated per frame.
camera_displacement_z = 0.0  # in "camera units" (cm). We'll scale it for the robot if needed.
focal_length = None           # Computed on the fly with 'c' key
running = True                # Main loop flag
z_offset_mm = 0.0  # Z offset for zeroing
Z_mm=0.0
heartPt = np.array([347.5, 279.7, 175])# known dot starting point on heart
dotcalibrated=False #has dot been calibrated

###############################################################################
#                   HELPER FUNCTIONS
###############################################################################
def JointTransformation(a, alpha, d, theta):
    """
    Standard DH transform matrix.
    T = RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha).
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),               np.cos(alpha),               d],
        [0,              0,                           0,                            1]
    ], dtype=float)

def plotunitvector(ax, pt, ux, uy, uz, scale=5):
    """
    Plots a local coordinate frame at point 'pt' using quivers in x=red, y=green, z=blue.
    """
    x0, y0, z0 = pt
    ax.quiver(x0, y0, z0, scale*ux[0], scale*ux[1], scale*ux[2], color='r', linewidth=2)
    ax.quiver(x0, y0, z0, scale*uy[0], scale*uy[1], scale*uy[2], color='g', linewidth=2)
    ax.quiver(x0, y0, z0, scale*uz[0], scale*uz[1], scale*uz[2], color='b', linewidth=2)

def updateQuiver3(quiver, origin, direction, scale=5):
    """
    Update an existing quiver, which in matplotlib is a Line3DCollection.
    We reassign the 3D segment for it.
    """
    x0, y0, z0 = origin
    dx, dy, dz = scale*direction
    quiver.set_segments([np.array([[x0, y0, z0], [x0 + dx, y0 + dy, z0 + dz]])])

###############################################################################
#                   MAIN
###############################################################################
def main():
    global camera_displacement_z, focal_length, running, z_offset_mm,Z_mm, heartPt

    # ---------------------------
    # 1) Open Serial
    # ---------------------------
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)
        ser.reset_input_buffer()
        print(f"[INFO] Opened serial {SERIAL_PORT} at {SERIAL_BAUD}")
    except Exception as e:
        print(f"[ERROR] Could not open serial: {e}")
        return

    # ---------------------------
    # 2) Open Camera
    # ---------------------------
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Failed to open camera index {CAMERA_INDEX}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

    print("[INFO] Press 'c' in the OpenCV window to calibrate camera.")
    print("[INFO] Press 'z' in the OpenCV window to zero BNO055 sensor.")
    print("[INFO] Press 'q' in the OpenCV window to quit.")

    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"[INFO] Camera resolution: {int(width)}x{int(height)}")

    cx = width / 2.0
    cy = height / 2.0

    # ---------------------------
    # 3) Setup Matplotlib Figure
    # ---------------------------
    # Use an interactive backend (e.g., TkAgg)
    matplotlib.use("TkAgg")

    fig = plt.figure("Live Robot Animation", figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(elev=20., azim=-35)
    ax.set_ylim(0, 800.4)
    ax.set_xlim(0, 400.2)
    ax.set_zlim(0, 400.2)
    plt.title("Robot + Camera (Press 'q' to quit in OpenCV window )")



    # Robot base reference:
    BRF_pt = np.array([118, 296.5, 0])  # base reference
    plotunitvector(ax, BRF_pt,
                   np.array([1, 0, 0]),
                   np.array([0, 1, 0]),
                   np.array([0, 0, 1]), scale=3)

    # Robot link line
    line_robot, = ax.plot([], [], [], 'o-',
                          linewidth=2, markersize=6,
                          color='k', markerfacecolor='r')

    # End-effector orientation quivers
    scale_arrow = 5
    effector_axes = {
        'x': ax.quiver(0, 0, 0, 0, 0, 0, color='r', linewidth=2),
        'y': ax.quiver(0, 0, 0, 0, 0, 0, color='g', linewidth=2),
        'z': ax.quiver(0, 0, 0, 0, 0, 0, color='b', linewidth=2),
    }

    ax.plot([heartPt[0]], [heartPt[1]], [heartPt[2]],
            'ms', markersize=8, markerfacecolor='k')

    # Dot marker above the heart
    dot_marker, = ax.plot([], [], [], 'r^', markersize=8, markerfacecolor='r')

    # ---------------------------
    # 4) Robot link constants
    # ---------------------------
    l01A = 261 + 11.3  # to gyro
    d01 = 0
    l01B = 0
    l45A = 91
    l45B = 0
    l56A = 20
    l56B = 205

    # --------------------------- Heart Reference Point ---------------------------
    dotPt = heartPt.copy()  # Dot starts at heart position

    # We'll do an indefinite loop until user presses 'q' in OpenCV window
    while running:
        # -------------------------------------------------------------------
        # A) READ CAMERA & DETECT RED CIRCLE
        # -------------------------------------------------------------------
        ret, frame = cap.read()
        if not ret:
            print("[WARNING] Failed to grab frame from camera.")
            # We'll continue anyway; maybe next iteration is okay
            # If you prefer, break here:
            # break

        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(hsv, LOWER_COLOR_1, UPPER_COLOR_1)
            mask2 = cv2.inRange(hsv, LOWER_COLOR_2, UPPER_COLOR_2)
            mask = mask1 + mask2

            # Morph reduce noise
            mask = cv2.erode(mask, KERNEL, iterations=1)
            mask = cv2.dilate(mask, KERNEL, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
           
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if len(largest) >= 5:
                    ellipse = cv2.fitEllipse(largest)
                    (xc, yc), (major, minor), angle = ellipse
                    diameter_pixels = 0.5*(major + minor)
                    cv2.ellipse(frame, ellipse, (0,255,0), 2)
                    cv2.circle(frame, (int(xc), int(yc)), 3, (0,255,0), -1)

                    # If we have focal_length and the circle is big enough
                    if focal_length is not None and diameter_pixels > 10:
                        distance_mm = (KNOWN_DIAMETER_MM * focal_length) / diameter_pixels

                        x_shifted = (xc - cx)
                        y_shifted = (yc - cy)
                        X_mm = (x_shifted / focal_length) * distance_mm
                        Y_mm = (y_shifted / focal_length) * distance_mm

                        # If you interpret Y_mm as "forward distance" and want Z to be "up/down"
                        Z_mm = -Y_mm  # Negative if you want to invert the axis

                        # Update dot Z position if calibrated
                        if dot_calibrated:
                            camera_displacement_z = Z_mm - z_offset_mm  # Calculate displacement
                            dotPt[2] = heartPt[2] + camera_displacement_z  # Update dot Z position


                    else:
                        # If not calibrated yet, we can't compute real distances
                        pass

            # Show text: if we have a displacement, print it
            if focal_length is not None:
                disp_text = f"Dot displacement Z={camera_displacement_z:.2f} mm"
                cv2.putText(frame, disp_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            cv2.imshow("Camera Feed - Press q to quit, c to calibrate camera, z to zero gyro", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                running = False
            elif key == ord('c'):
                # Attempt to calibrate
                if contours and len(largest) >= 5 and diameter_pixels > 10:
                    focal_length = (diameter_pixels * CALIBRATION_DISTANCE_MM) / KNOWN_DIAMETER_MM
                    print(f"[CALIBRATION] focal_length={focal_length:.2f}")

                    # Zero the Z offset at the circle's current height:
                    z_offset_mm = Z_mm
                    dotPt = heartPt.copy()  # Reset dot to starting position
                    dot_calibrated = True  # Mark that calibration has been done
                    print(f"[CALIBRATION] Dot set to heart position {dotPt}")
                    
                else:
                    print("[CALIBRATION] Circle not found or too small for calibration.")
            elif key == ord('z'):  # Zero BNO055 when 'Z' is pressed in OpenCV window
                ser.write(b'z\n')
                print("[INFO] Sent 'z' to Arduino for zeroing BNO055.")

        # -------------------------------------------------------------------
        # B) READ SERIAL FROM ARDUINO & PARSE
        # -------------------------------------------------------------------
        data_line = ser.readline().decode('utf-8').strip()
        fields = data_line.split(',')
        if len(fields) >= 8:
            try:
                headingDeg, rollDeg, pitchDeg, sysCal, gyroCal, accelCal, magCal, potExtension = map(float, fields)

                heading = math.radians(headingDeg)
                roll = math.radians(rollDeg)
                pitch = math.radians(pitchDeg)

                # Offsets for L-shape
                theta2_yaw = heading
                theta3_pitch = pitch
                theta4_roll = roll

                # Build DH param table
                dhparams = [
                    [0,        0,          (l01A + d01 + l01B),    0],
                    [0,  np.pi/2,          0,                      theta2_yaw],
                    [0,  np.pi/2,          0,                      theta3_pitch + (np.pi)/2],
                    [0,        0,          0,                      theta4_roll],
                    [l45A,     0,          0,                      0],
                    [0,        0,          l45B,                   0],
                    [0,        0, (potExtension + l56A + l56B),    0]
                ]

                # Partial transforms
                partialT = [np.eye(4)]
                for row in dhparams:
                    a, alpha, d, th = row
                    Tj = JointTransformation(a, alpha, d, th)
                    partialT.append(partialT[-1] @ Tj)

                # Joint positions
                joint_positions = []
                for T in partialT:
                    trans_vec = T[0:3, 3]
                    # Add base offset
                    joint_positions.append(BRF_pt + trans_vec)
                joint_positions = np.array(joint_positions)

                p_ee = joint_positions[-1]  # end-effector

                # Update line
                line_robot.set_data(joint_positions[:, 0], joint_positions[:, 1])
                line_robot.set_3d_properties(joint_positions[:, 2])

                # Orientation
                R_ee = partialT[-1][0:3, 0:3]
                updateQuiver3(effector_axes['x'], p_ee, R_ee[:, 0], scale_arrow)
                updateQuiver3(effector_axes['y'], p_ee, R_ee[:, 1], scale_arrow)
                updateQuiver3(effector_axes['z'], p_ee, R_ee[:, 2], scale_arrow)

                # Dot offset: camera_displacement_z is in cm, we want mm for the robot?
                # If your robot code is in mm, do this:
                dot_z_disp_robot_mm = camera_displacement_z 

                # The dot is heartPt + that displacement
                dotPt = np.array(heartPt)  # copy
                dotPt[2] +=  dot_z_disp_robot_mm

                # Must pass lists to set_data / set_3d_properties
                dot_marker.set_data([dotPt[0]], [dotPt[1]])
                dot_marker.set_3d_properties([dotPt[2]])

                # Title
                tipCoordRel = p_ee - heartPt
                dist_tool_dot = math.sqrt((p_ee[0] - heartPt[0])**2 + (p_ee[1] - heartPt[1])**2 + (p_ee[2] - heartPt[2])**2)

                print(f"roll: {math.degrees(theta4_roll):.4f}, pitch: {math.degrees(theta3_pitch):.4f}, yaw: {math.degrees(theta2_yaw):.4f}, tool_to_dot_dist: {dist_tool_dot}")
                #print(f"roll: {math.degrees(theta4_roll):.5f}, pitch: {math.degrees(theta3_pitch):.5f}, yaw: {math.degrees(theta2_yaw):.5f}, tool_to_dot_dist: {tipCoordRel}")


            except ValueError:
                pass  # if parse fails

        # -------------------------------------------------------------------
        # C) UPDATE MATPLOTLIB
        # -------------------------------------------------------------------
        plt.draw()
        plt.pause(0.001)

        if not running:
            break

    # End while loop
    # Cleanup
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Exiting main function")


if __name__ == "__main__":
    main()

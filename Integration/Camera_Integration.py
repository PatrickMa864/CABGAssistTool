import cv2
import numpy as np
import math
import time
import threading
import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# =========================================================
#                GLOBAL CONFIG & PARAMETERS
# =========================================================

CAMERA_INDEX = 1
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 30

KNOWN_DIAMETER_MM = 8.4
CALIBRATION_DISTANCE_MM = 370.0

LOWER_COLOR_1 = np.array([0, 120, 70])
UPPER_COLOR_1 = np.array([10, 255, 255])
LOWER_COLOR_2 = np.array([170, 120, 70])
UPPER_COLOR_2 = np.array([180, 255, 255])
KERNEL = np.ones((3, 3), np.uint8)

# -----------------------------------------------------
# 1) GLOBAL REFERENCE
#    We want "heart" at (0,0,0). We'll label it so.
# -----------------------------------------------------
heartPt = np.array([0.0, 0.0, 0.0])

# The user wants the camera calibrate's current dot position => (0,20,150).
CALIB_TARGET = np.array([0.0, 20.0, 150.0])

# -----------------------------------------------------
# 2) CAMERA->GLOBAL TRANSFORM
#    Suppose your camera is at X=370 mm, facing negative X.
#    Mapping:
#      x_cam => Y_global
#      y_cam => -Z_global
#      z_cam => -X_global
#    Then we add the offset (370,0,0).
# -----------------------------------------------------
R_cam2glob = np.array([
    [ 0.0,  0.0, -1.0],  # X_g = -z_cam
    [ 1.0,  0.0,  0.0],  # Y_g =  x_cam
    [ 0.0, -1.0,  0.0]   # Z_g = -y_cam
], dtype=float)

WRF_OFFSET_X = 370.0
WRF_OFFSET_Y = 0.0
WRF_OFFSET_Z = 0.0

def raw_camera_to_global(x_cam, y_cam, z_cam):
    """
    Basic transform from camera coords to global coords
    (before applying the user-calibration offset).
    """
    cam_vec = np.array([x_cam, y_cam, z_cam], dtype=float)
    glob_vec = R_cam2glob @ cam_vec
    X_g = glob_vec[0] + WRF_OFFSET_X
    Y_g = glob_vec[1] + WRF_OFFSET_Y
    Z_g = glob_vec[2] + WRF_OFFSET_Z
    return (X_g, Y_g, Z_g)

# We'll store an additional offset that is computed at calibration time
# so that the dot's position is forced to (0,20,150).
camera_cal_offset = np.array([0.0, 0.0, 0.0])
calibrated_once = False

# -----------------------------------------------------
# 3) ROBOT / TOOL: Updated geometry
#    - Base pivot at (160, 0, 0).
#    - 1st link: 270 mm
#    - 2nd link: 90 mm
#    - final prismatic up to 30 mm
# -----------------------------------------------------
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

basePivot = np.array([160.0, 0.0, 0.0])
length_link1 = 270.0
length_link2 = 90.0
prismatic_max = 30.0

# pot scale => if potRaw=100 => ~30 mm
potScale = prismatic_max / 100.0

# -----------------------------------------------------
# 4) SHARED VARIABLES
# -----------------------------------------------------
camera_coordinates = [0.0, 0.0, 0.0]  # final (X, Y, Z) in global after offset
focal_length_container = {"value": None}
camera_lock = threading.Lock()
global_stop = False

# -----------------------------------------------------
# 4A) SMOOTHING + CLAMP
# -----------------------------------------------------
ALPHA = 0.2
MAX_DELTA = 50.0
prev_cam = [None, None, None]

# =========================================================
#                CAMERA THREAD
# =========================================================
def camera_thread():
    """
    Continuously reads from the camera, detects the dot, does pinhole => raw global,
    then applies smoothing & user-defined offset if calibrated.
    Press 'c' to calibrate the focal length,
    Press 'c' again to define the "calibration offset" => dot => (0,20,150).
    Press 'q' to quit.
    """
    global global_stop, prev_cam, camera_cal_offset, calibrated_once

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[ERROR] Can't open camera.")
        global_stop = True
        return

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Camera {int(w)}x{int(h)} @ {int(fps)} FPS")
    print("[INFO] Press 'c' to calibrate lens or set offset, 'q' to quit.")

    cx = w/2.0
    cy = h/2.0

    while not global_stop:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, LOWER_COLOR_1, UPPER_COLOR_1)
        mask2 = cv2.inRange(hsv, LOWER_COLOR_2, UPPER_COLOR_2)
        mask = mask1 + mask2

        # morphological
        mask = cv2.erode(mask, KERNEL, iterations=1)
        mask = cv2.dilate(mask, KERNEL, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if len(largest)>=5:
                ellipse = cv2.fitEllipse(largest)
                (x_c, y_c), (major, minor), angle = ellipse
                cv2.ellipse(frame, ellipse, (0,255,0),2)
                cv2.circle(frame, (int(x_c),int(y_c)),3,(0,255,0),-1)

                diam_px = 0.5*(major+minor)
                if diam_px>5:
                    fl = focal_length_container["value"]
                    if fl is not None:
                        # pinhole => raw global
                        z_cam_mm = (KNOWN_DIAMETER_MM*fl)/diam_px
                        dx_px = x_c-cx
                        dy_px = y_c-cy
                        x_cam_mm = (dx_px/fl)*z_cam_mm
                        y_cam_mm = (dy_px/fl)*z_cam_mm

                        # raw global
                        X_raw, Y_raw, Z_raw = raw_camera_to_global(x_cam_mm, y_cam_mm, z_cam_mm)

                        # smoothing + clamp
                        with camera_lock:
                            if prev_cam[0] is None:
                                # first time
                                prev_cam[0] = X_raw
                                prev_cam[1] = Y_raw
                                prev_cam[2] = Z_raw
                                # apply offset if we have it
                                X_off = X_raw + camera_cal_offset[0]
                                Y_off = Y_raw + camera_cal_offset[1]
                                Z_off = Z_raw + camera_cal_offset[2]
                                camera_coordinates[0] = X_off
                                camera_coordinates[1] = Y_off
                                camera_coordinates[2] = Z_off
                            else:
                                oldX, oldY, oldZ = prev_cam
                                dX = abs(X_raw - oldX)
                                dY = abs(Y_raw - oldY)
                                dZ = abs(Z_raw - oldZ)
                                if (dX<MAX_DELTA and dY<MAX_DELTA and dZ<MAX_DELTA):
                                    newX = ALPHA*X_raw + (1-ALPHA)*oldX
                                    newY = ALPHA*Y_raw + (1-ALPHA)*oldY
                                    newZ = ALPHA*Z_raw + (1-ALPHA)*oldZ
                                    prev_cam[0]=newX
                                    prev_cam[1]=newY
                                    prev_cam[2]=newZ
                                    # then apply the user offset
                                    X_off = newX + camera_cal_offset[0]
                                    Y_off = newY + camera_cal_offset[1]
                                    Z_off = newZ + camera_cal_offset[2]
                                    camera_coordinates[0] = X_off
                                    camera_coordinates[1] = Y_off
                                    camera_coordinates[2] = Z_off

                        txt = f"Cam=({camera_coordinates[0]:.1f},{camera_coordinates[1]:.1f},{camera_coordinates[2]:.1f})"
                        cv2.putText(frame, txt, (int(x_c)+10,int(y_c)-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),2)

        cv2.imshow("ArducamView", frame)
        key = cv2.waitKey(1)&0xFF
        if key==ord('q'):
            global_stop = True
        elif key==ord('c'):
            # If we haven't calibrated the focal length yet, do that,
            # else we define the calibration offset => dot => (0,20,150)
            if fl is None:
                # same code
                if contours and len(largest)>=5 and diam_px>5:
                    new_fl = (diam_px*CALIBRATION_DISTANCE_MM)/KNOWN_DIAMETER_MM
                    focal_length_container["value"] = new_fl
                    print(f"[CALIBRATION] focal_length= {new_fl:.2f}")
                else:
                    print("[CALIBRATION] No valid dot.")
            else:
                # define the offset so that the current camera_coords => (0,20,150)
                with camera_lock:
                    # we want camera_coordinates => CALIB_TARGET
                    # so offset = CALIB_TARGET - (X_off currently)
                    # but note that right now camera_coordinates[] is already offset
                    # so let's compute the rawX, rawY, rawZ from prev_cam
                    if prev_cam[0] is not None:
                        rawX, rawY, rawZ = prev_cam
                        # so the current displayed = raw + old offset
                        # we want new => (0,20,150)
                        # => offset = CALIB_TARGET - raw
                        new_offset = CALIB_TARGET - np.array([rawX, rawY, rawZ])
                        camera_cal_offset = new_offset
                        calibrated_once = True
                        print("[CALIBRATION OFFSET] Now the dot at this moment => (0,20,150).")
                    else:
                        print("[CALIBRATION OFFSET] No prev_cam yet, can't define offset.")

    cap.release()
    cv2.destroyAllWindows()
    print("[INFO] Camera thread ended.")

# =========================================================
# FORWARD KINEMATICS
# =========================================================
def JointTransformation(a, alpha, d, theta):
    return np.array([
        [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
        [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
        [0,               math.sin(alpha),                  math.cos(alpha),                 d],
        [0,               0,                                0,                               1]
    ], dtype=float)

def plotunitvector(ax, pt, ux, uy, uz, scale=5):
    x0,y0,z0 = pt
    ax.quiver(x0,y0,z0, scale*ux[0],scale*ux[1],scale*ux[2], color='r',linewidth=2)
    ax.quiver(x0,y0,z0, scale*uy[0],scale*uy[1],scale*uy[2], color='g',linewidth=2)
    ax.quiver(x0,y0,z0, scale*uz[0],scale*uz[1],scale*uz[2], color='b',linewidth=2)

def updateQuiver3(quiver, origin, direction, scale=5):
    x0,y0,z0 = origin
    dx,dy,dz = scale*direction
    quiver.set_segments([np.array([[x0,y0,z0],[x0+dx,y0+dy,z0+dz]])])

# Tool geometry
basePivot = np.array([160.0, 0.0, 0.0])
length_link1 = 270.0
length_link2 = 90.0
prismatic_max = 30.0
potScale = prismatic_max / 100.0

# =========================================================
#  MAIN LOOP: ARDUINO + PLOTTING
# =========================================================
def live_robot_animation_fast_with_offsets_zero_heartpoint():
    global global_stop

    # 1) Start camera thread
    cam_t = threading.Thread(target=camera_thread, daemon=True)
    cam_t.start()

    # 2) Setup Arduino
    try:
        if not global_stop:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            time.sleep(2)
            ser.reset_input_buffer()
        else:
            return
    except Exception as e:
        print(f"[ERROR] opening serial port: {e}")
        global_stop = True
        return

    # 3) Matplotlib
    plt.ion()
    fig = plt.figure("Live Robot Animation", figsize=(10,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1,1,1))

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.view_init(elev=20., azim=-35)
    ax.set_xlim(-100, 500)
    ax.set_ylim(-100, 300)
    ax.set_zlim(-50, 400)
    ax.set_title("Initializing...")

    def on_key(event):
        if event.key.lower()=='z' and not global_stop:
            ser.write(b'z\n')
            print("[INFO] Sent 'z' to Arduino for zeroing BNO055.")
    fig.canvas.mpl_connect('key_press_event', on_key)

    # 4) Plot the "heart" at (0,0,0)
    ax.plot([heartPt[0]],[heartPt[1]],[heartPt[2]],
            'ms', markersize=8, markerfacecolor='m', label='Heart(0,0,0)')

    # 5) base pivot
    plotunitvector(ax, basePivot, np.array([1,0,0]),
                           np.array([0,1,0]),
                           np.array([0,0,1]),
                           scale=30)

    # 6) Robot line
    line_robot, = ax.plot([], [], [], 'o-', linewidth=2, markersize=6,
                          color='k', markerfacecolor='r', label='Tool')

    eff_axes = {
        'x': ax.quiver(0,0,0, 0,0,0, color='r',linewidth=2),
        'y': ax.quiver(0,0,0, 0,0,0, color='g',linewidth=2),
        'z': ax.quiver(0,0,0, 0,0,0, color='b',linewidth=2),
    }

    # 7) Camera dot
    camera_dot, = ax.plot([], [], [], 'bo', markersize=6, label='CameraDot')

    ax.legend()

    iteration=0
    while not global_stop:
        iteration+=1
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue
            parts=line.split(',')
            if len(parts)<8:
                continue
            headingDeg,rollDeg,pitchDeg,sysCal,gyroCal,accelCal,magCal,potRaw = map(float, parts)
        except:
            continue

        heading = math.radians(headingDeg)
        roll    = math.radians(rollDeg)
        pitch   = math.radians(pitchDeg)

        extension = potRaw*potScale
        if extension>prismatic_max:
            extension=prismatic_max

        # We'll do a custom forward kinematics:
        # 1) T0 = place basePivot
        T0 = np.eye(4)
        T0[0,3]=basePivot[0]
        T0[1,3]=basePivot[1]
        T0[2,3]=basePivot[2]

        # 2) heading about Z
        Th = np.eye(4)
        Th[0,0]=math.cos(heading); Th[0,1]=-math.sin(heading)
        Th[1,0]=math.sin(heading); Th[1,1]= math.cos(heading)

        # 3) link1 = 270 mm up
        Tlink1 = np.eye(4)
        Tlink1[2,3]= length_link1

        # 4) pitch about Y
        Tp= np.eye(4)
        Tp[0,0]=math.cos(pitch);  Tp[0,2]= math.sin(pitch)
        Tp[2,0]=-math.sin(pitch); Tp[2,2]= math.cos(pitch)

        # 5) link2= 90 mm up
        Tlink2= np.eye(4)
        Tlink2[2,3]= length_link2

        # 6) roll about X
        Tr= np.eye(4)
        Tr[1,1]= math.cos(roll);  Tr[1,2]=-math.sin(roll)
        Tr[2,1]= math.sin(roll);  Tr[2,2]= math.cos(roll)

        # 7) prismatic
        Text= np.eye(4)
        Text[2,3]= extension

        T = T0@Th@Tlink1@Tp@Tlink2@Tr@Text

        partials=[T0,
                  T0@Th@Tlink1,
                  T0@Th@Tlink1@Tp@Tlink2,
                  T]

        jpos=[]
        for M in partials:
            jpos.append(M[0:3,3])
        jpos = np.array(jpos)

        line_robot.set_data(jpos[:,0], jpos[:,1])
        line_robot.set_3d_properties(jpos[:,2])

        R_ee= T[0:3,0:3]
        p_ee= T[0:3,3]
        updateQuiver3(eff_axes['x'], p_ee, R_ee[:,0], 30)
        updateQuiver3(eff_axes['y'], p_ee, R_ee[:,1], 30)
        updateQuiver3(eff_axes['z'], p_ee, R_ee[:,2], 30)

        # camera dot
        with camera_lock:
            cX, cY, cZ = camera_coordinates
        camera_dot.set_data([cX],[cY])
        camera_dot.set_3d_properties([cZ])

        if iteration%5==0:
            tipRel = p_ee - heartPt
            new_title = (f"Hdg={headingDeg:.1f}, Rll={rollDeg:.1f}, Pch={pitchDeg:.1f} | "
                         f"Sys={int(sysCal)},Gy={int(gyroCal)},Ac={int(accelCal)},Mg={int(magCal)} | "
                         f"Ext={extension:.1f} mm\n"
                         f"Tool=({p_ee[0]:.1f},{p_ee[1]:.1f},{p_ee[2]:.1f}) | "
                         f"TipRelHeart=({tipRel[0]:.1f},{tipRel[1]:.1f},{tipRel[2]:.1f}) | "
                         f"Cam=({cX:.1f},{cY:.1f},{cZ:.1f})")
            ax.set_title(new_title)
            plt.draw()
            plt.pause(0.001)

    ser.close()
    print("[INFO] Main loop ended. Serial closed.")
    print("[INFO] Exiting program.")


if __name__=="__main__":
    live_robot_animation_fast_with_offsets_zero_heartpoint()

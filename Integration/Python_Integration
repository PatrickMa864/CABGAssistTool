import serial
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def live_robot_animation_fast_with_offsets_zero_heartpoint():
    """
    Python script to:
    1) Connect to an Arduino via serial (which sends heading,roll,pitch,cal statuses,pot).
    2) Parse the 8 CSV values.
    3) Compute forward kinematics with an extra prismatic joint for the potentiometer extension.
    4) Plot and animate the robot's motion in 3D in real-time.
    5) Press 'z' in the figure window to send 'z' to Arduino (for zeroing BNO055).
    """

    # -----------------------------
    # 1) SET UP SERIAL PORT
    # -----------------------------
    portName = 'COM3'  # Update to your serial port
    baudRate = 115200
    try:
        ser = serial.Serial(portName, baudRate, timeout=1)
        time.sleep(2)  # Allow Arduino to reset
    except Exception as e:
        print(f"Error opening serial port {portName}: {e}")
        return

    # Clear any junk in the buffer
    ser.reset_input_buffer()

    # -----------------------------
    # 1A) CREATE FIGURE
    # -----------------------------
    fig = plt.figure("Live Robot Animation", figsize=(10,6))
    fig.canvas.manager.set_window_title("Live Robot Animation")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1,1,1))

    # Add labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set initial view and axis limits
    ax.view_init(elev=20., azim=-35)
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.set_zlim(-50, 100)
    plt.title("Initializing...")

    # -----------------------------
    # Key press callback: Press 'z' to zero BNO055
    # -----------------------------
    def on_key(event):
        if event.key.lower() == 'z':
            ser.write(b'z\n')
            print('Sent "z" to Arduino for zeroing the BNO055.')

    fig.canvas.mpl_connect('key_press_event', on_key)

    # -----------------------------
    # 2) PLOT BASE FRAME
    # -----------------------------
    BRF_pt = np.array([40.2 - 13.5, 21.0, 0])  # base reference
    plotunitvector(ax, BRF_pt, np.array([1,0,0]),
                          np.array([0,1,0]),
                          np.array([0,0,1]), scale=10)

    # -----------------------------
    # Robot link line object
    # -----------------------------
    line_robot, = ax.plot([], [], [], 'o-', linewidth=2, markersize=6,
                          color='k', markerfacecolor='r')

    # -----------------------------
    # 3) QUIVERS FOR END-EFFECTOR ORIENTATION
    # -----------------------------
    scale = 5
    effector_axes = {
        'x': ax.quiver(0,0,0, 0,0,0, color='r', linewidth=2),
        'y': ax.quiver(0,0,0, 0,0,0, color='g', linewidth=2),
        'z': ax.quiver(0,0,0, 0,0,0, color='b', linewidth=2),
    }

    # -----------------------------
    # 4) SET REFERENCE "HEART" POINT
    # -----------------------------
    heartPt = np.array([10, 10, 0])
    ax.plot([heartPt[0]], [heartPt[1]], [heartPt[2]],
            'ms', markersize=10, markerfacecolor='m')

    # -----------------------------
    # 5) ROBOT LINK CONSTANTS (DH setup)
    # -----------------------------
    l01A = 26.1
    d01  = 0
    l01B = 0

    l45A = 10.1
    l45B = 11.5

    # Example for prismatic joint scaling
    potScale = 1 # mm per count if we want 0..51 mm 3.3v 
    # or set potScale= 0.1, 0.2, etc. to match your real hardware

    # -----------------------------
    # MAIN LOOP
    # -----------------------------
    n_iterations = 2000
    for i in range(n_iterations):
        # Read one line from serial
        # Format: heading,roll,pitch,sys,gyro,accel,mag,potValue
        try:
            data_line = ser.readline().decode('utf-8').strip()
            fields = data_line.split(',')
            if len(fields) < 8:
                continue
            fields = list(map(float, fields))
        except:
            continue

        headingDeg, rollDeg, pitchDeg, sysCal, gyroCal, accelCal, magCal, potRaw = fields

        # Convert angles to radians
        heading = math.radians(headingDeg)
        roll    = math.radians(rollDeg)
        pitch   = math.radians(pitchDeg)

        # Convert pot raw reading to extension (prismatic)
        potExtension = potRaw * potScale  # e.g. in mm

        # Offsets for L-shape (from your original code)
        theta2_yaw   = heading
        theta3_pitch = pitch
        theta4_roll  = roll

        # Build DH parameters for the rotational links
        # [a, alpha, d, theta]
        dhparams = [
            [0,       0,         (l01A + d01 + l01B),   0],
            [0,       np.pi/2,   0,                     theta2_yaw + np.pi/2],
            [0,       np.pi/2,   0,                     theta3_pitch + np.pi/4],
            [0,       0,         0,                     theta4_roll],
            [l45A,    0,         l45B,                  0],
        ]

        # Now add the final prismatic link
        # Assume it extends along the Z-axis of the previous frame
        # So we can represent it as: a=0, alpha=0, d=potExtension, theta=0
        # (Adjust alpha if your prismatic axis is different)
        dhparams.append([0, 0, potExtension, 0])

        # 1) Compute partial transforms
        partialT = [np.eye(4)]
        for row in dhparams:
            a, alpha, d, theta = row
            Tj = JointTransformation(a, alpha, d, theta)
            partialT.append(partialT[-1].dot(Tj))

        # 2) Collect joint positions
        joint_positions = []
        for T in partialT:
            trans_vec = T[0:3, 3]
            joint_positions.append(BRF_pt + trans_vec)
        joint_positions = np.array(joint_positions)

        # End-effector = last joint
        p_ee = joint_positions[-1]

        # Update the line object
        line_robot.set_data(joint_positions[:,0], joint_positions[:,1])
        line_robot.set_3d_properties(joint_positions[:,2])

        # End-effector orientation (from final transform)
        R_ee = partialT[-1][0:3, 0:3]
        updateQuiver3(effector_axes['x'], p_ee, R_ee[:,0], scale)
        updateQuiver3(effector_axes['y'], p_ee, R_ee[:,1], scale)
        updateQuiver3(effector_axes['z'], p_ee, R_ee[:,2], scale)

        # Compute tip relative to heart
        tipCoordRel = p_ee - heartPt

        # Update figure title
        new_title = (
            f"Heading={headingDeg:.1f}°, Roll={rollDeg:.1f}°, Pitch={pitchDeg:.1f}°  |  "
            f"Cal: {int(sysCal)},{int(gyroCal)},{int(accelCal)},{int(magCal)} | "
            f"PotRaw={potRaw:.0f}, Ext={potExtension:.1f}\n"
            f"Tip wrt Heart = ({tipCoordRel[0]:.1f}, {tipCoordRel[1]:.1f}, {tipCoordRel[2]:.1f})"
        )
        ax.set_title(new_title)

        # Update plot
        plt.draw()
        plt.pause(0.001)

    # Close serial when done
    ser.close()
    print("Done. Closed serial port.")


def JointTransformation(a, alpha, d, theta):
    """
    Standard DH transform matrix:
    T = [ cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta)
          sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta)
          0            sin(alpha)             cos(alpha)               d
          0            0                      0                        1 ]
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,             np.sin(alpha),                np.cos(alpha),               d              ],
        [0,             0,                            0,                           1              ]
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
    We reassign the 3D segment for it. 'direction' should be a 3-element vector.
    """
    x0, y0, z0 = origin
    dx, dy, dz = scale*direction
    quiver.set_segments([np.array([[x0, y0, z0],[x0+dx, y0+dy, z0+dz]])])


if __name__ == "__main__":
    live_robot_animation_fast_with_offsets_zero_heartpoint()

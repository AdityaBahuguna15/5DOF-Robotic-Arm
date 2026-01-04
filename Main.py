import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from Kinematics import ArmKinematics
from Mapper import ServoMapper
from ArduinoPy import ArduinoInterface
from LiveViz import ArmVisualizer

# --------------------------
# USER SETTINGS
# --------------------------
USE_ARDUINO = True    # <- set True when ready
ARDUINO_PORT = 'COM5'

# --------------------------
# INITIALIZE MODULES
# --------------------------
arm = ArmKinematics()
mapper = ServoMapper()
viz = ArmVisualizer(limits=20)

if USE_ARDUINO:
    ar = ArduinoInterface(port=ARDUINO_PORT, baud=115200)
    ar.connect()

# --------------------------
# MATPLOTLIB SETUP
# --------------------------
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.30)

# initial slider values
cx0, cy0, cz0 = 8.0, 5.0, 2.0
ry0, rz0 = 0.0, 0.0   # degrees

# Create slider axes
ax_x  = plt.axes([0.20, 0.20, 0.65, 0.03])
ax_y  = plt.axes([0.20, 0.16, 0.65, 0.03])
ax_z  = plt.axes([0.20, 0.12, 0.65, 0.03])
ax_ry = plt.axes([0.20, 0.08, 0.65, 0.03])
ax_rz = plt.axes([0.20, 0.04, 0.65, 0.03])

s_x  = Slider(ax_x,  'X',  -10, 15, valinit=cx0)
s_y  = Slider(ax_y,  'Y',  -10, 15, valinit=cy0)
s_z  = Slider(ax_z,  'Z',    0, 20, valinit=cz0)
s_ry = Slider(ax_ry, 'Ry°', -180, 180, valinit=ry0)
s_rz = Slider(ax_rz, 'Rz°', -180, 180, valinit=rz0)

# --------------------------
# MAIN UPDATE FUNCTION
# --------------------------
def update(val):
    # read sliders
    cx, cy, cz = s_x.val, s_y.val, s_z.val
    ry, rz = s_ry.val, s_rz.val

    # rotation
    Ry = arm.Yrot(np.deg2rad(ry))
    Rz = arm.Zrot(np.deg2rad(rz))
    Rcube = Ry @ Rz

    # target transform
    T_target = np.eye(4)
    T_target[:3,:3] = Rcube @ arm.Xrot(np.pi)
    T_target[:3,3]  = [cx, cy, cz]

    # IK solve
    try:
        t1,t2,t3,t4,t5 = arm.inverse_kinematics(T_target, elbow='up')
    except:
        return

    # FK
    Ts = arm.forward_kinematics(t1,t2,t3,t4,t5)

    # build cube
    hs = 1
    vertices = np.array([
        [-hs,-hs,-hs],[hs,-hs,-hs],[hs,hs,-hs],[-hs,hs,-hs],
        [-hs,-hs, hs],[hs,-hs, hs],[hs,hs, hs],[-hs,hs, hs]
    ])
    rv = vertices @ Rcube.T + np.array([cx,cy,cz])
    faces = [
        [rv[0],rv[1],rv[2],rv[3]],
        [rv[4],rv[5],rv[6],rv[7]],
        [rv[0],rv[1],rv[5],rv[4]],
        [rv[2],rv[3],rv[7],rv[6]],
        [rv[1],rv[2],rv[6],rv[5]],
        [rv[0],rv[3],rv[7],rv[4]],
    ]

    # single-window redraw
    viz.draw(ax, Ts, cube_faces=faces, cube_centroid=(cx,cy,cz))
    fig.canvas.draw_idle()

    degs = np.rad2deg([t1, t2, t3, t4, t5])

    s1 = mapper.base(degs[0])
    s2 = mapper.shoulder(degs[1])
    s3 = mapper.elbow(degs[2])
    s4 = mapper.wrist_pitch(degs[3])
    s5 = mapper.wrist_roll(degs[4])

    # --- Send live to Arduino ---
    if USE_ARDUINO:
        try:
            ar.send_angles(s1, s2, s3, s4, s5)
        except:
            print("Warning: could not send servo packet")

# --------------------------
# LINK SLIDERS TO UPDATE
# --------------------------
for slider in [s_x, s_y, s_z, s_ry, s_rz]:
    slider.on_changed(update)

# start
update(None)
plt.show()

if USE_ARDUINO:
    ar.close()

"""
Run from /build:
python ../tools/plotter.py
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg
import os
from matplotlib import gridspec

# Setup UDP Socket
sock = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
sock.bind (('127.0.0.1', 5555))
sock.setblocking (False)

# Data containers
ekf_path = {"x": [], "y": [], "z": []}
gt_path  = {"x": [], "y": [], "z": []}
time_ns = 0

# Setup Plot
plt.ion ()
fig = plt.figure (figsize = (14, 7))
gs = gridspec.GridSpec (2, 2, width_ratios = [2.5, 1])

ax_traj = fig.add_subplot (gs[:, 0], projection = '3d')
ax_l = fig.add_subplot (gs[0, 1])
ax_r = fig.add_subplot (gs[1, 1])

def on_key (event):
    if event.key == 'r':
        print("Resetting trajectories")
        reset()
        plt.draw()

fig.canvas.mpl_connect ('key_press_event', on_key)

ax_l.set_title ("Left Image")
ax_r.set_title ("Right Image")

img_l_artist = None
img_r_artist = None

def reset ():
    global img_l_artist, img_r_artist
    global ekf_path, gt_path
    print ("Resetting")
    ekf_path = {"x": [], "y": [], "z": []}
    gt_path  = {"x": [], "y": [], "z": []}
    
    ax_traj.cla ()
    ax_traj.set_xlabel ('X (m)')
    ax_traj.set_ylabel ('Y (m)')
    ax_traj.set_zlabel ('Z (m)')
    ax_traj.set_title('Live VIO Trajectory')
    
    img_l_artist = None
    img_r_artist = None
    ax_l.cla ()
    ax_r.cla ()
    ax_l.set_title ("Left Image")
    ax_r.set_title ("Right Image")

def update_image (
    ax,
    artist,
    path
):
    if not os.path.exists (path):
        return artist

    img = mpimg.imread (path)

    if artist is None:
        artist = ax.imshow (img)
        ax.axis ("off")
    else:
        artist.set_data (img)

    return artist

def update_plot (
):
    global time_ns, img_l_artist, img_r_artist
    new_data = False

    try:
        while True:
            data, addr = sock.recvfrom (1024)
            v = struct.unpack ('ddddddL', data)

            if (all (val == -1 for val in v[:5])):
                reset ()
                return

            ekf_path["x"].append (v[0])
            ekf_path["y"].append (v[1])
            ekf_path["z"].append (v[2])

            gt_path["x"].append (v[3])
            gt_path["y"].append (v[4])
            gt_path["z"].append (v[5])

            time_ns = v[6]
            new_data = True
    except BlockingIOError:
        pass

    if not new_data:
        return

    # Traj
    ax_traj.cla ()
    # ISSUE: No idea why 0, 0, 0 is plotted
    ax_traj.plot (gt_path["x"], gt_path["y"], gt_path["z"],
                'g-', label = 'Ground Truth', alpha=  0.7)
    ax_traj.plot (ekf_path["x"], ekf_path["y"], ekf_path["z"],
                'r--', label = 'EKF Estimate', linewidth = 2)

    ax_traj.scatter (ekf_path["x"][-1], ekf_path["y"][-1], ekf_path["z"][-1],
                     color = 'red', s = 50)
    ax_traj.scatter (gt_path["x"][-1], gt_path["y"][-1], gt_path["z"][-1],
                     color = 'green', s = 50)

    ax_traj.set_xlabel ('X (m)')
    ax_traj.set_ylabel ('Y (m)')
    ax_traj.set_zlabel ('Z (m)')
    ax_traj.set_title (f'Live VIO Trajectory (t = {time_ns})')
    ax_traj.legend ()

    # Images
    root = "../data/euroc_datasets/machine_hall/MH_01_easy/mav0/"
    left_path  = f"{root}cam0/data/{time_ns}.png"
    right_path = f"{root}cam1/data/{time_ns}.png"


    img_l_artist = update_image (ax_l, img_l_artist, left_path)
    img_r_artist = update_image (ax_r, img_r_artist, right_path)

    plt.draw ()
    plt.pause (0.001)

print ("Matplotlib Visualizer Online. Waiting for C++ data on port 5555...")

try:
    while True:
        update_plot ()
except KeyboardInterrupt:
    print ("Stopping visualizer.")

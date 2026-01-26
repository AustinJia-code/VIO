"""
VIO estimation viewer

pip install numpy matplotlib
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Setup UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5555))
sock.setblocking(False)

# Data containers
ekf_path = {"x": [], "y": [], "z": []}
gt_path = {"x": [], "y": [], "z": []}

# Setup Plot
plt.ion() # Interactive mode
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

def update_plot():
    global ekf_path, gt_path
    new_data = False
    
    # Drain the socket buffer
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            # Unpack 6 doubles (48 bytes)
            v = struct.unpack('dddddd', data)
            
            ekf_path["x"].append(v[0]); ekf_path["y"].append(v[1]); ekf_path["z"].append(v[2])
            gt_path["x"].append(v[3]); gt_path["y"].append(v[4]); gt_path["z"].append(v[5])
            new_data = True
    except BlockingIOError:
        pass

    if new_data:
        ax.cla() # Clear for redraw
        
        # Plot Trajectories
        ax.plot(gt_path["x"], gt_path["y"], gt_path["z"], 'g-', label='Ground Truth', alpha=0.7)
        ax.plot(ekf_path["x"], ekf_path["y"], ekf_path["z"], 'r--', label='EKF Estimate', linewidth=2)
        
        # Plot Current Position (Head)
        if len(ekf_path["x"]) > 0:
            ax.scatter(ekf_path["x"][-1], ekf_path["y"][-1], ekf_path["z"][-1], color='red', s=50)
            ax.scatter(gt_path["x"][-1], gt_path["y"][-1], gt_path["z"][-1], color='green', s=50)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Live VIO Trajectory Comparison')
        ax.legend()
        
        # Keep the view scaled to the data
        plt.draw()
        plt.pause(0.001)

print("Matplotlib Visualizer Online. Waiting for C++ data on port 5555...")

try:
    while True:
        update_plot()
except KeyboardInterrupt:
    print("Stopping visualizer.")
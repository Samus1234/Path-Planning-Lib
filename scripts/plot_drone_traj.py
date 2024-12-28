import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

data = pd.read_csv("data/drone_simulation.csv")

# Extract data columns
time = data["Time"].to_numpy()
x = data["Position_X"].to_numpy()
y = data["Position_Y"].to_numpy()
z = data["Position_Z"].to_numpy()
vx = data["Velocity_X"].to_numpy()
vy = data["Velocity_Y"].to_numpy()
vz = data["Velocity_Z"].to_numpy()
roll = data["Roll"].to_numpy()
pitch = data["Pitch"].to_numpy()
yaw = data["Yaw"].to_numpy()

# Plot Translational Coordinates
plt.figure(figsize=(16, 9))
plt.subplot(211)
plt.plot(time, x, label="$x$", lw=2)
plt.plot(time, y, label="$y$", lw=2)
plt.plot(time, z, label="$z$", lw=2)
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Translational Coordinates")
plt.legend()
plt.grid()

# Plot Rotational Angles
plt.subplot(212)
plt.plot(time, roll, label="Roll ($\\phi$)", lw=2)
plt.plot(time, pitch, label="Pitch ($\\theta$)", lw=2)
plt.plot(time, yaw, label="Yaw ($\\psi$)", lw=2)
plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.title("Rotational Angles")
plt.legend()
plt.grid()
plt.tight_layout()

plt.savefig("plots/states.png")

# 3D Trajectory Plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.plot3D(x, y, z, label="Trajectory", lw=2)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D Drone Trajectory")
plt.legend()
plt.grid()

plt.savefig("plots/3d_trajectory.png")

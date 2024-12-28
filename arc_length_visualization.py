import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

WIDTH = 1920
HEIGHT = 1080
DPI = 100

data = pd.read_csv("data/trajectory_data.csv")

def set_figure_size():
    return (WIDTH / DPI, HEIGHT / DPI)

t = data['time'].to_numpy()

plt.figure(figsize=set_figure_size())
plt.plot(t, data["s"].to_numpy(), lw=2.5, label="s")
plt.title("Arc-length vs. Time", fontsize=20)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Arc-length (m)", fontsize=20)
plt.legend(fontsize=15)
plt.grid()
plt.savefig("plots/arclength_vs_time.png", dpi=DPI)
plt.close()

plt.figure(figsize=set_figure_size())
plt.plot(t, data["x"].to_numpy(), label="x", lw=2.5)
plt.plot(t, data["y"].to_numpy(), label="y", lw=2.5)
plt.plot(t, data["z"].to_numpy(), label="z", lw=2.5)
plt.plot(t, data["x_ref"].to_numpy(), label="x_{ref}", lw=1.5, linestyle='--', c='tab:blue')
plt.plot(t, data["y_ref"].to_numpy(), label="y_{ref}", lw=1.5, linestyle='--', c='tab:orange')
plt.plot(t, data["z_ref"].to_numpy(), label="z_{ref}", lw=1.5, linestyle='--', c='tab:green')
plt.title("Position vs. Time", fontsize=20)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Position (m)", fontsize=20)
plt.legend(fontsize=15)
plt.grid()
plt.savefig("plots/position_vs_time.png", dpi=DPI)
plt.close()

plt.figure(figsize=set_figure_size())
plt.plot(t, data["u1"].to_numpy(), label="u1", lw=2.5)
plt.plot(t, data["u2"].to_numpy(), label="u2", lw=2.5)
plt.plot(t, data["u3"].to_numpy(), label="u3", lw=2.5)
plt.title("Control Inputs vs. Time", fontsize=20)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Control Inputs", fontsize=20)
plt.legend(fontsize=15)
plt.grid()
plt.savefig("plots/control_inputs_vs_time.png", dpi=DPI)
plt.close()

plt.figure(figsize=set_figure_size())
plt.plot(t, data['e'].to_numpy(), lw=2.5)
plt.title("Position Tracking Error vs. Time", fontsize=20)
plt.xlabel("Time (s)", fontsize=20)
plt.ylabel("Tracking Error (m)", fontsize=20)
plt.grid()
plt.savefig("plots/position_tracking_error.png", dpi=DPI)
plt.close()

print("Plots saved")
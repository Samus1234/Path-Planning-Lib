import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

nodes = pd.read_csv("data/lattice_nodes.csv")
edges = pd.read_csv("data/lattice_edges.csv")
primitives = pd.read_csv("data/motion_primitives.csv")

plt.figure(figsize=(10, 10))
plt.scatter(nodes['x'], nodes['y'], c='b', label='Lattice Nodes', alpha=0.8)

for _, edge in edges.iterrows():
    parent_idx = int(edge['parent'])
    child_idx = int(edge['child'])
    control_idx = int(edge['control'])

    x_start, y_start, theta_start = nodes.iloc[parent_idx][['x', 'y', 'theta']]
    x_end, y_end = nodes.iloc[child_idx][['x', 'y']]

    control = primitives.iloc[control_idx]
    velocity = control['velocity']
    steering_angle = control['steering_angle']

    t = np.linspace(0, 1e-1, 50)
    R = 1 / np.tan(steering_angle) if np.abs(steering_angle) > 1e-2 else np.inf
    x_curve, y_curve = [], []

    for dt in t:
        if np.isinf(R):
            x = x_start + dt * velocity * np.cos(theta_start)
            y = y_start + dt * velocity * np.sin(theta_start)
        else:
            x = x_start - R * np.sin(theta_start) + R * np.sin(theta_start + dt * velocity / R)
            y = y_start + R * np.cos(theta_start) - R * np.cos(theta_start + dt * velocity / R)
        x_curve.append(x)
        y_curve.append(y)

    plt.plot(x_curve, y_curve, 'r-', alpha=0.5)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Lattice Visualization with Curved Paths')
plt.legend()
plt.grid(True)
plt.show()

plt.savefig("plots/lattice.png")

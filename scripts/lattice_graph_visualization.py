import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the CSV files
nodes = pd.read_csv("data/lattice_nodes.csv")
edges = pd.read_csv("data/lattice_edges.csv")
primitives = pd.read_csv("data/motion_primitives.csv")

# Normalize costs for coloring
min_cost = edges['cost'].min()
max_cost = edges['cost'].max()
edges['normalized_cost'] = (edges['cost'] - min_cost) / (max_cost - min_cost)

# Create the plot
plt.figure(figsize=(10, 10))
plt.scatter(nodes['y'], nodes['x'], c='b', label='Lattice Nodes', alpha=0.8)

for _, edge in edges.iterrows():
    parent_idx = int(edge['parent'])
    child_idx = int(edge['child'])
    control_idx = int(edge['control'])
    cost_normalized = edge['normalized_cost']

    x_start, y_start, theta_start = nodes.iloc[parent_idx][['x', 'y', 'theta']]
    x_end, y_end = nodes.iloc[child_idx][['x', 'y']]

    control = primitives.iloc[control_idx]
    velocity = control['velocity']
    steering_angle = control['steering_angle']

    t = np.linspace(0, 1e-1, 100)
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

    # Use the colormap for the edge color based on cost
    plt.plot(y_curve, x_curve, color=plt.cm.viridis(cost_normalized), alpha=0.5)

# Add a colorbar for edge costs
sm = plt.cm.ScalarMappable(cmap='winter', norm=plt.Normalize(vmin=min_cost, vmax=max_cost))
sm.set_array([])
plt.colorbar(sm, label='Edge Cost')

# Finalize the plot
plt.xlabel('Y')
plt.ylabel('X')
plt.title('Lattice Visualization with Curved Paths and Edge Costs')
plt.legend()
plt.grid(True)

# Save the plot
plt.savefig("plots/lattice_with_costs.png")

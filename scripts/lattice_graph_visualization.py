import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

nodes = pd.read_csv("data/lattice_nodes.csv")
edges = pd.read_csv("data/lattice_edges.csv")
primitives = pd.read_csv("data/motion_primitives.csv")
path = pd.read_csv("data/optimal_path.csv")

min_cost = edges['cost'].min()
max_cost = edges['cost'].max()
edges['normalized_cost'] = (edges['cost'] - min_cost) / (max_cost - min_cost)

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

    plt.plot(y_curve, x_curve, color=plt.cm.viridis(cost_normalized), alpha=0.5)

path_edges = path.to_numpy()
for i in range(len(path_edges) - 1):
    state_idx_1, primitive_idx = path_edges[i]
    state_idx_2, _ = path_edges[i + 1]
    x_start, y_start = nodes.iloc[state_idx_1][['x', 'y']]
    x_end, y_end = nodes.iloc[state_idx_2][['x', 'y']]
    plt.plot([y_start, y_end], [x_start, x_end], 'r-', lw=2, label='Optimal Path' if i == 0 else "")

start_node = path_edges[0][0]
goal_node = path_edges[-1][0]
plt.scatter(nodes.iloc[start_node]['y'], nodes.iloc[start_node]['x'], c='green', label='Start Node', s=100)
plt.scatter(nodes.iloc[goal_node]['y'], nodes.iloc[goal_node]['x'], c='purple', label='Goal Node', s=100)

sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=min_cost, vmax=max_cost))
sm.set_array([])
plt.colorbar(sm, label='Edge Cost')

plt.xlabel('Y')
plt.ylabel('X')
plt.title('Lattice Visualization with Optimal Path and Edge Costs')
plt.legend()
plt.grid(True)

plt.savefig("plots/lattice_with_optimal_path.png")
plt.show()


import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys

# Relative path to the point cloud csv 
point_cloud_path = "../data/point_cloud2.csv"

# Load the point cloud data
df = pd.read_csv(point_cloud_path, header=None, names=['p_x', 'p_y', 'p_z', 'q_x', 'q_y', 'q_z', 'q_w'])
# df = df[:3000]

# Plot the point cloud
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(df['p_x'], df['p_y'], df['p_z'], c=df['p_z'], cmap='viridis', s=10)

fig.colorbar(scatter, ax=ax, label='z position (mm)')
ax.set_xlabel('x (mm)')
ax.set_ylabel('y (mm)')
ax.set_zlabel('z (mm)')
ax.set_title('Platform Workspace')

plt.show()
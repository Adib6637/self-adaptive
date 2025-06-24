import numpy as np
import pandas as pd
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Simulate observed drone flight data
np.random.seed(42)
n_samples = 100

data = {
    'speed': np.random.uniform(5, 20, n_samples),
    'height': np.random.uniform(10, 50, n_samples),
    'resolution': np.random.uniform(5, 15, n_samples),
    'fps': np.random.uniform(10, 60, n_samples),
    'power_consumption': np.random.uniform(50, 200, n_samples)
}

df = pd.DataFrame(data)

# Apply clustering
kmeans = KMeans(n_clusters=4, random_state=42)
df['cluster'] = kmeans.fit_predict(df[['speed', 'height', 'resolution', 'fps']])

# Identify the cluster with the lowest average power consumption
cluster_avg_power = df.groupby('cluster')['power_consumption'].mean()
lowest_power_cluster = cluster_avg_power.idxmin()

# Get the centroid of the lowest power cluster
centroid = kmeans.cluster_centers_[lowest_power_cluster]

# Print the centroid values
print(f"Centroid of the lowest power cluster (Cluster {lowest_power_cluster}):")
print(f"Speed: {centroid[0]:.2f} m/s")
print(f"Height: {centroid[1]:.2f} m")
print(f"Resolution: {centroid[2]:.2f} MP")
print(f"FPS: {centroid[3]:.2f}")

# Visualize the clusters
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
scatter = ax.scatter(df['speed'], df['height'], df['resolution'], c=df['cluster'], cmap='viridis')

# Add centroid marker
ax.scatter(centroid[0], centroid[1], centroid[2], color='red', s=100, label='Lowest Power Centroid')

ax.set_xlabel('Speed (m/s)')
ax.set_ylabel('Height (m)')
ax.set_zlabel('Resolution (MP)')
ax.set_title('Drone Configuration Clusters')
ax.legend()

plt.show()

# Update constraints based on the centroid of the lowest power cluster
updated_constraints = {
    'speed': (centroid[0] * 0.9, centroid[0] * 1.1),
    'height': (centroid[1] * 0.9, centroid[1] * 1.1),
    'resolution': (centroid[2] * 0.9, centroid[2] * 1.1),
    'fps': (centroid[3] * 0.9, centroid[3] * 1.1)
}

print("Updated constraints based on the lowest power cluster centroid:")
print(updated_constraints)


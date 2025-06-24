import pandas as pd
import numpy as np

# Load data
observed = pd.read_csv('../dataset/speed_power_centroids.csv', header=None, names=['speed', 'power'])
centroids = pd.read_csv('../dataset/speed_power_observed.csv', header=None, names=['speed', 'power'])


# Compute distances to nearest centroid
def min_distance(row, centroids):
    return np.min(np.sqrt((centroids['speed'] - row['speed'])**2 + (centroids['power'] - row['power'])**2))

distances = observed.apply(lambda row: min_distance(row, centroids), axis=1)

# Analyze distribution
print("Distance statistics:")
print(distances.describe())

# Suggest threshold: mean + 2*std
suggested_threshold = distances.mean() + 2 * distances.std()
print(f"Suggested threshold (mean + 2*std): {suggested_threshold:.4f}")

# Or use 95th percentile
percentile_95 = np.percentile(distances, 95)
print(f"Suggested threshold (95th percentile): {percentile_95:.4f}")
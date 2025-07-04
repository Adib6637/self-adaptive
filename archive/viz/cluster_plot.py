import pandas as pd
import matplotlib.pyplot as plt

# 2D centroids and observed data

import pandas as pd
import matplotlib.pyplot as plt

# Load data
observed = pd.read_csv('../dataset/speed_power_centroids.csv', header=None, names=['speed', 'power'])
centroids = pd.read_csv('../dataset/speed_power_observed.csv', header=None, names=['speed', 'power'])
try:
    anomalies = pd.read_csv('../dataset/anomaly_detected.csv', header=None, names=['speed', 'power'])
except FileNotFoundError:
    anomalies = pd.DataFrame(columns=['speed', 'power'])

# Plot observed data
plt.scatter(observed['speed'], observed['power'], label='Observed', alpha=0.5, s=10)
# Plot centroids
plt.scatter(centroids['speed'], centroids['power'], color='red', label='Centroids', marker='x', s=100)
# Plot anomalies
if not anomalies.empty:
    plt.scatter(anomalies['speed'], anomalies['power'], color='orange', label='Anomalies', marker='o', s=40, edgecolors='k')

plt.xlabel('Speed')
plt.ylabel('Power Actuator')
plt.title('Clustering and Anomaly Detection')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
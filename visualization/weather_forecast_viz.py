import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
df = pd.read_csv("d:/OneDrive/FH Dortmund/Sem4/Master-Thesis/Implementation/Simulation/log/log_wind_forecast.csv")
# Select only rows with index 1, 4, 7, ...
df = df.iloc[range(1, len(df), 4)].reset_index(drop=True)
# Plot wind speed and direction over time
fig, ax1 = plt.subplots(figsize=(10, 6))

color = 'tab:blue'
ax1.set_xlabel('Index')
ax1.set_ylabel('Wind Speed (m/s)', color=color)
ax1.plot(df.index, df['v_wind'], color=color, label='Wind Speed')
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()
color = 'tab:orange'
ax2.set_ylabel('Wind Direction (deg)', color=color)
ax2.plot(df.index, df['theta_wind'], color=color, label='Wind Direction')
ax2.tick_params(axis='y', labelcolor=color)

plt.title('Wind Speed and Direction Over Time')
fig.tight_layout()
plt.show()

# Wind rose (polar plot)
plt.figure(figsize=(8, 8))
theta = np.deg2rad(df['theta_wind'])
r = df['v_wind']
ax = plt.subplot(111, polar=True)
c = ax.scatter(theta, r, c=r, cmap='viridis', alpha=0.75)
plt.title('Wind Rose (Direction and Speed)')
plt.colorbar(c, label='Wind Speed (m/s)')
plt.show()
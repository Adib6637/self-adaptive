import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv("d:/OneDrive/FH Dortmund/Sem4/Master-Thesis/Implementation/Simulation/log/log_wind_forecast.csv")

# 1. Histogram of wind speed
plt.figure(figsize=(8, 4))
plt.hist(df['v_wind'], bins=20, color='skyblue', edgecolor='black')
plt.xlabel('Wind Speed (m/s)')
plt.ylabel('Frequency')
plt.title('Histogram of Wind Speed')
plt.grid(True)
plt.tight_layout()
plt.show()

# 2. Histogram of wind direction
plt.figure(figsize=(8, 4))
plt.hist(df['theta_wind'], bins=20, color='salmon', edgecolor='black')
plt.xlabel('Wind Direction (degrees)')
plt.ylabel('Frequency')
plt.title('Histogram of Wind Direction')
plt.grid(True)
plt.tight_layout()
plt.show()

# 3. 2D hexbin plot: wind speed vs wind direction
plt.figure(figsize=(8, 6))
plt.hexbin(df['theta_wind'], df['v_wind'], gridsize=30, cmap='viridis')
plt.xlabel('Wind Direction (degrees)')
plt.ylabel('Wind Speed (m/s)')
plt.title('Hexbin Plot of Wind Speed vs Wind Direction')
plt.colorbar(label='Counts')
plt.tight_layout()
plt.show()
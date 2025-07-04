import pandas as pd
import matplotlib.pyplot as plt

# Load the results
df = pd.read_csv("../log/log_optimization_results.csv")

# 1. Number of drones used per run
drones_used_per_run = df[df['used'] > 0].groupby('counter')['drone'].count()
plt.figure()
plt.plot(drones_used_per_run.index, drones_used_per_run.values, marker='o')
plt.xlabel('Run Counter')
plt.ylabel('Number of Drones Used')
plt.title('Number of Drones Used per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 2. Total energy consumption per run
total_energy_per_run = df.groupby('counter')['energy'].sum()
plt.figure()
plt.plot(total_energy_per_run.index, total_energy_per_run.values, marker='s', color='orange')
plt.xlabel('Run Counter')
plt.ylabel('Total Energy Consumption')
plt.title('Total Energy Consumption per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 3. Total power consumption per run
total_power_per_run = df.groupby('counter')['power'].sum()
plt.figure()
plt.plot(total_power_per_run.index, total_power_per_run.values, marker='^', color='green')
plt.xlabel('Run Counter')
plt.ylabel('Total Power Consumption')
plt.title('Total Power Consumption per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 4. Total operation time per run
total_operation_time_per_run = df.groupby('counter')['operation_time_req'].sum()
plt.figure()
plt.plot(total_operation_time_per_run.index, total_operation_time_per_run.values, marker='x', color='red')
plt.xlabel('Run Counter')
plt.ylabel('Total Operation Time')
plt.title('Total Operation Time per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 5. Total covered area per run
total_covered_area_per_run = df.groupby('counter')['covered_area_true'].sum()
plt.figure()
plt.plot(total_covered_area_per_run.index, total_covered_area_per_run.values, marker='d', color='purple')
plt.xlabel('Run Counter')
plt.ylabel('Total Covered Area')
plt.title('Total Covered Area per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 6. Average charging cycles per run
avg_charging_cycles_per_run = df.groupby('counter')['charging_cycles'].mean()
plt.figure()
plt.plot(avg_charging_cycles_per_run.index, avg_charging_cycles_per_run.values, marker='v', color='brown')
plt.xlabel('Run Counter')
plt.ylabel('Average Charging Cycles')
plt.title('Average Charging Cycles per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 7. Example: Plot speed and altitude for each drone in the last run
last_run = df['counter'].max()
df_last = df[df['counter'] == last_run]
plt.figure()
plt.bar(df_last['drone'], df_last['v_true'], label='Speed')
plt.bar(df_last['drone'], df_last['h'], alpha=0.7, label='Altitude')
plt.xlabel('Drone')
plt.ylabel('Value')
plt.title('Speed and Altitude per Drone (Last Run)')
plt.legend()
plt.tight_layout()
plt.show()
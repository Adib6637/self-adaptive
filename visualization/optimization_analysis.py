import pandas as pd
import matplotlib.pyplot as plt

# Load your CSV file
df = pd.read_csv("../Simulation/log/log_optimization_results.csv")

# Make counter relative (1, 2, 3, ...)
unique_counters = sorted(df['counter'].unique())
counter_map = {old: new for new, old in enumerate(unique_counters, start=1)}
df['counter'] = df['counter'].map(counter_map)

# Get unique counters (runs)
counters = sorted(df['counter'].unique())
num_drones = df['drone'].nunique()

# 1. Plot number of drones used per run
drones_used_per_run = df[df['used'] > 0].groupby('counter')['drone'].count()
plt.figure(figsize=(8, 4))
plt.plot(drones_used_per_run.index, drones_used_per_run.values, marker='o')
plt.xlabel('Run Counter')
plt.ylabel('Number of Drones Used')
plt.title('Number of Drones Used per Run')
plt.grid(True)
plt.tight_layout()
plt.show()

# 2. Plot all numeric columns per drone
numeric_cols = [
    'v', 'v_true', 'h', 'fps', 'pix', 'pix_x', 'pix_y',
    'covered_area_x_t0', 'covered_area_y_t0', 'covered_area_total_t0',
    'covered_area_total', 'covered_area_true', 'number_of_place_covered',
    'covered_distance', 'operation_time', 'pa_consumption', 'ps_consumption',
    'power', 'energy', 'charging_cycles', 'operation_time_req'
]

for col in numeric_cols:
    plt.figure(figsize=(10, 5))
    for drone_id in df['drone'].unique():
        plt.plot(
            df[df['drone'] == drone_id]['counter'],
            df[df['drone'] == drone_id][col],
            marker='o',
            label=f'Drone {drone_id}'
        )
    plt.xlabel('Run Counter')
    plt.ylabel(col.replace('_', ' ').title())
    plt.title(f'{col.replace("_", " ").title()} per Drone')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# 3. Plot total values per run (sum over all drones)
for col in numeric_cols:
    plt.figure(figsize=(8, 4))
    totals = df.groupby('counter')[col].sum()
    plt.plot(totals.index, totals.values, marker='s', color='black')
    plt.xlabel('Run Counter')
    plt.ylabel(f'Total {col.replace("_", " ").title()}')
    plt.title(f'Total {col.replace("_", " ").title()} per Run')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# 4. Plot total energy vs number of drones used per run
# Get total energy per run
energy_per_run = df.groupby('counter')['energy'].sum()
# Get number of drones used per run (already computed)
# drones_used_per_run
fig, ax1 = plt.subplots(figsize=(10, 5))
color = 'tab:blue'
ax1.set_xlabel('Run Counter')
ax1.set_ylabel('Total Energy', color=color)
ax1.plot(energy_per_run.index, energy_per_run.values, marker='o', color=color, label='Total Energy')
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()
color = 'tab:orange'
ax2.set_ylabel('Number of Drones Used', color=color)
ax2.plot(drones_used_per_run.index, drones_used_per_run.values, marker='s', color=color, label='Drones Used')
ax2.tick_params(axis='y', labelcolor=color)

plt.title('Total Energy vs Number of Drones Used per Run')
fig.tight_layout()
plt.grid(True)
plt.show()
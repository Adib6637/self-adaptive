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
plt.bar(drones_used_per_run.index, drones_used_per_run.values)
plt.xlabel('Run Counter')
plt.ylabel('Number of Drones Used')
plt.title('Number of Drones Used per Run')
plt.grid(True, axis='y')
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
    width = 0.8 / len(df['drone'].unique())
    for i, drone_id in enumerate(df['drone'].unique()):
        x = df[df['drone'] == drone_id]['counter'] + i * width - 0.4
        y = df[df['drone'] == drone_id][col]
        plt.bar(x, y, width=width, label=f'Drone {drone_id}')
    plt.xlabel('Run Counter')
    plt.ylabel(col.replace('_', ' ').title())
    plt.title(f'{col.replace("_", " ").title()} per Drone')
    plt.legend()
    plt.grid(True, axis='y')
    plt.tight_layout()
    plt.show()

# 3. Plot total values per run (sum over all drones)
for col in numeric_cols:
    plt.figure(figsize=(8, 4))
    totals = df.groupby('counter')[col].sum()
    plt.bar(totals.index, totals.values, color='black')
    plt.xlabel('Run Counter')
    plt.ylabel(f'Total {col.replace("_", " ").title()}')
    plt.title(f'Total {col.replace("_", " ").title()} per Run')
    plt.grid(True, axis='y')
    plt.tight_layout()
    plt.show()
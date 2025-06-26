import pandas as pd
import matplotlib.pyplot as plt

# --- Configuration ---
CSV_PATHS = [
    "../log/log_optimization_results.csv",
    "../Simulation/log/log_optimization_results.csv"
]
NUMERIC_COLS = [
    #'v', 'v_true', 'h', 'fps', 'pix', 'pix_x', 'pix_y',
    #'covered_area_x_t0', 'covered_area_y_t0', 'covered_area_total_t0',
    #'covered_area_total', 'covered_area_true', 'number_of_place_covered',
    #'covered_distance',
    'operation_time',
    'pa_consumption', 'ps_consumption',
    'power', 'energy', 'charging_cycles',
    'operation_time_req'
]
COUNTER_RANGE = (24, 43)

x_size = 5
y_size = 5

# --- Data Loading ---
df = None
for path in CSV_PATHS:
    try:
        df = pd.read_csv(path)
        break
    except FileNotFoundError:
        continue
if df is None:
    raise FileNotFoundError("CSV file not found in any of the specified paths.")

# --- Preprocessing ---
unique_counters = sorted(df['counter'].unique())
counter_map = {old: new for new, old in enumerate(unique_counters, start=1)}
df['counter'] = df['counter'].map(counter_map)
df = df[df['counter'].between(*COUNTER_RANGE)]
# Only keep counters 1, 4, 7, 10, ...
df = df[(df['counter'] - 1) % 4 == 0]
counters = sorted(df['counter'].unique())
num_drones = df['drone'].nunique()
counter_min = df['counter'].min()

# --- Helper Plot Functions ---
def plot_per_drone(col):
    plt.figure(figsize=(x_size, y_size))
    for drone_id in df['drone'].unique():
        plt.plot(
            df[df['drone'] == drone_id]['counter'] - counter_min,
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

def plot_total_per_run(col):
    plt.figure(figsize=(x_size, y_size))
    totals = df.groupby('counter')[col].sum()
    plt.plot(totals.index - counter_min, totals.values, marker='s', color='black')
    plt.xlabel('Run Counter')
    plt.ylabel(f'Total {col.replace("_", " ").title()}')
    plt.title(f'Total {col.replace("_", " ").title()} per Run')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# --- Analysis & Plots ---

# 1. Number of drones used per run
drones_used_per_run = df[df['used'] > 0].groupby('counter')['drone'].count()
"""
plt.figure(figsize=(8, 4))
plt.plot(drones_used_per_run.index - counter_min, drones_used_per_run.values, marker='o')
plt.xlabel('Run Counter')
plt.ylabel('Number of Drones Used')
plt.title('Number of Drones Used per Run')
plt.grid(True)
plt.tight_layout()
plt.show()
"""


# 2. Numeric columns per drone
for col in NUMERIC_COLS:
    plot_per_drone(col)

# 3. Total values per run
for col in NUMERIC_COLS:
    plot_total_per_run(col)



# 4. Total energy vs number of drones used per run
energy_per_run = df.groupby('counter')['energy'].sum()
fig, ax1 = plt.subplots(figsize=(x_size, y_size))
ax1.set_xlabel('Run Counter')
ax1.set_ylabel('Total Energy', color='tab:blue')
ax1.plot(energy_per_run.index - counter_min, energy_per_run.values, marker='o', color='tab:blue', label='Total Energy')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax2 = ax1.twinx()
ax2.set_ylabel('Number of Drones Used', color='tab:orange')
ax2.plot(drones_used_per_run.index - counter_min, drones_used_per_run.values, marker='s', color='tab:orange', label='Drones Used')
ax2.tick_params(axis='y', labelcolor='tab:orange')
plt.title('Total Energy vs Number of Drones Used per Run')
fig.tight_layout()
plt.grid(True)
plt.show()

"""
# 5. Max operation time and number of drones used per run
max_operation_time_per_run = df.groupby('counter')['operation_time_req'].max()
fig, ax1 = plt.subplots(figsize=(x_size, y_size))
ax1.set_xlabel('Run Counter)
ax1.set_ylabel('Max Operation Time (per Run)', color='tab:blue')
ax1.plot(max_operation_time_per_run.index - counter_min, max_operation_time_per_run.values, marker='o', color='tab:blue', label='Max Operation Time')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax2 = ax1.twinx()
ax2.set_ylabel('Number of Drones Used', color='tab:orange')
ax2.plot(drones_used_per_run.index - counter_min, drones_used_per_run.values, marker='s', color='tab:orange', label='Drones Used')
ax2.tick_params(axis='y', labelcolor='tab:orange')
plt.title('Max Operation Time and Number of Drones Used per Run')
fig.tight_layout()
plt.grid(True)
plt.show()
"""

# 6. Objective expression per run
operation_time_total_per_run = df.groupby('counter')['operation_time_req'].sum()
objective_expr_per_run = (
    energy_per_run
    + 100 * drones_used_per_run
    + 100 * operation_time_total_per_run
)
plt.figure(figsize=(x_size, y_size))
plt.plot(objective_expr_per_run.index - counter_min, objective_expr_per_run.values, marker='D', color='purple')
plt.xlabel('Run Counter')
plt.ylabel('Objective Value')
plt.title('Objective Expression per Run')
plt.grid(True)
plt.tight_layout()
plt.show()
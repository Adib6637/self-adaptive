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
COUNTER_RANGE = (0, 45)
interval = 3

x_size = 5
y_size = 5

XLABEL_FONT_SIZE = 12
YLABEL_FONT_SIZE = 12
TITLE_FONT_SIZE = 14

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
df = df[(df['counter'] - 1) % interval == 0]
counters = sorted(df['counter'].unique())
num_drones = df['drone'].nunique()
counter_min = df['counter'].min()

# --- Helper Plot Functions ---
def plot_per_drone(col):
    plt.figure(figsize=(x_size, y_size))
    filtered_counters = sorted(df['counter'].unique())
    counter_idx_map = {counter: idx+1 for idx, counter in enumerate(filtered_counters)}
    for drone_id in df['drone'].unique():
        drone_df = df[df['drone'] == drone_id]
        x_vals = drone_df['counter'].map(counter_idx_map)
        plt.plot(
            x_vals,
            drone_df[col],
            marker='o',
            label=f'Drone {drone_id}'
        )
    plt.xlabel('Run Index', fontsize=XLABEL_FONT_SIZE)
    plt.ylabel(col.replace('_', ' ').title(), fontsize=YLABEL_FONT_SIZE)
    plt.title(f'{col.replace("_", " ").title()} per Drone', fontsize=TITLE_FONT_SIZE)
    plt.legend()
    plt.grid(True)
    plt.xticks(range(1, len(filtered_counters) + 1))
    plt.tight_layout()
    plt.show()

def plot_total_per_run(col):
    plt.figure(figsize=(x_size, y_size))
    totals = df.groupby('counter')[col].sum()
    filtered_counters = sorted(totals.index)
    x_vals = range(1, len(filtered_counters) + 1)
    plt.plot(x_vals, totals.values, marker='s', color='black')
    plt.xlabel('Run Index', fontsize=XLABEL_FONT_SIZE)
    if col == 'operation_time_req':
        ylabel = 'Total Operation Time'
        title = 'Total Operation Time per Run'
    else:
        ylabel = f'Total {col.replace("_", " ").title()}'
        title = f'Total {col.replace("_", " ").title()} per Run'
    plt.ylabel(ylabel, fontsize=YLABEL_FONT_SIZE)
    plt.title(title, fontsize=TITLE_FONT_SIZE)
    plt.grid(True)
    plt.xticks(range(1, len(filtered_counters) + 1))
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
energy_run_indices = range(1, len(energy_per_run.index) + 1)
drones_run_indices = range(1, len(drones_used_per_run.index) + 1)
fig, ax1 = plt.subplots(figsize=(x_size, y_size))
ax1.set_xlabel('Run Index', fontsize=XLABEL_FONT_SIZE)
ax1.set_ylabel('Total Energy', color='tab:blue', fontsize=YLABEL_FONT_SIZE)
ax1.plot(energy_run_indices, energy_per_run.values, marker='o', color='tab:blue', label='Total Energy')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_xticks(list(energy_run_indices))
ax2 = ax1.twinx()
ax2.set_ylabel('Number of Drones Used', color='tab:orange', fontsize=YLABEL_FONT_SIZE)
ax2.plot(drones_run_indices, drones_used_per_run.values, marker='s', color='tab:orange', label='Drones Used')
ax2.tick_params(axis='y', labelcolor='tab:orange')
ax2.set_xticks(list(drones_run_indices))
plt.title('Total Energy vs Number of Drones Used per Run', fontsize=TITLE_FONT_SIZE)
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
objective_run_indices = range(1, len(objective_expr_per_run.index) + 1)
plt.figure(figsize=(x_size, y_size))
plt.plot(objective_run_indices, objective_expr_per_run.values, marker='D', color='purple')
plt.xlabel('Run Index', fontsize=XLABEL_FONT_SIZE)
plt.ylabel('Objective Value', fontsize=YLABEL_FONT_SIZE)
plt.title('Objective Expression per Run', fontsize=TITLE_FONT_SIZE)
plt.grid(True)
plt.xticks(list(objective_run_indices))
plt.tight_layout()
plt.show()
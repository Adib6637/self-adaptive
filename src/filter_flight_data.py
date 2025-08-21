import pandas as pd
import os
import sys

# Change the working directory to the script's directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Read the high error counters
error_df = pd.read_csv('../log/log_high_error_counters.csv', header=None, names=['counter', 'error'])
counters_to_remove = error_df['counter'].values

# Read the flight data
flight_data = pd.read_csv('../dataset/flight_filtered.csv')

# Remove rows where index + 1 matches the counter values
rows_to_keep = ~flight_data.index.isin([c-1 for c in counters_to_remove])
filtered_flight_data = flight_data[rows_to_keep]

# Save the filtered data
filtered_flight_data.to_csv('../dataset/flight_filtered_clean.csv', index=False)

print(f"Removed {len(counters_to_remove)} rows with high error from the flight data.")
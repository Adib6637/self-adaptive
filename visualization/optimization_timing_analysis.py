import pandas as pd

# Set the range of rows to analyze (by index). Use None for full range.
start = 12-5  
end = 12   

# Load the CSV file (skip the first column, use the second column 'last_runtime')
df = pd.read_csv("../log/log_runtime_results.csv")

df_range = df.iloc[start:end]

# Calculate statistics
avg_time = df_range["last_runtime"].mean()
min_time = df_range["last_runtime"].min()
max_time = df_range["last_runtime"].max()

print(f"Average execution time: {avg_time:.6f} seconds")
print(f"Minimum execution time: {min_time:.6f} seconds")
print(f"Maximum execution time: {max_time:.6f} seconds")
print(f"Number of records: {len(df_range)}")
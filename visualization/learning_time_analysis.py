import pandas as pd

# Load the CSV file (assuming the first column is execution time)
df = pd.read_csv("../log/log_learning_time.csv", header=None, names=["time", "counter"])

# Calculate statistics
avg_time = df["time"].mean()
min_time = df["time"].min()
max_time = df["time"].max()

print(f"Number of records: {len(df)}")
print(f"Average execution time: {avg_time:.6f} seconds")
print(f"Minimum execution time: {min_time:.6f} seconds")
print(f"Maximum execution time: {max_time:.6f} seconds")
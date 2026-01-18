import pandas as pd
import os

# Load the CSV file
case = 1
sub = 1
cat = ''
test = f'c_{case}_{sub}/'
file_path = f'../log/{test}{cat}log_learning_time.csv'
df = pd.read_csv(file_path, header=None, names=["time", "counter"])

# Calculate statistics
avg_time = df["time"].mean()
min_time = df["time"].min()
max_time = df["time"].max()

print(f"Number of records: {len(df)}")
print(f"Average execution time: {avg_time:.6f} seconds")
print(f"Minimum execution time: {min_time:.6f} seconds")
print(f"Maximum execution time: {max_time:.6f} seconds")

# Prepare LaTeX table
latex_table = r"""
\begin{table}[h!]
\centering
\begin{tabular}{l r}
\hline
Statistic & Value \\
\hline
Number of records & %d \\
Average execution time (s) & %.6f \\
Minimum execution time (s) & %.6f \\
Maximum execution time (s) & %.6f \\
\hline
\end{tabular}
\caption{Learning Time Statistics}
\end{table}
""" % (len(df), avg_time, min_time, max_time)

# Save LaTeX table
output_dir = os.path.dirname(file_path)
os.makedirs(output_dir, exist_ok=True)
with open(os.path.join(output_dir, "learning_time_stats.tex"), "w") as f:
    f.write(latex_table)
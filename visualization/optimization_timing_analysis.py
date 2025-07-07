import pandas as pd
import os

# Set the range of rows to analyze (by index). Use None for full range.
start = 12-5  
end = 12   

# Load the CSV file (skip the first column, use the second column 'last_runtime')
case = 6
sub = 0
cat = '3/'
test = f'c_{case}_{sub}/'
file_path = f'../log/{test}{cat}log_runtime_results.csv'
df = pd.read_csv(file_path)

df_range = df.iloc[start:end]

# Calculate statistics
avg_time = df_range["last_runtime"].mean()
min_time = df_range["last_runtime"].min()
max_time = df_range["last_runtime"].max()

print(f"Average execution time: {avg_time:.6f} seconds")
print(f"Minimum execution time: {min_time:.6f} seconds")
print(f"Maximum execution time: {max_time:.6f} seconds")
print(f"Number of records: {len(df_range)}")

# Prepare LaTeX table
latex_table = r"""
\begin{table}[h!]
\centering
\begin{tabular}{l r}
\hline
Statistic & Value \\
\hline
Average execution time (s) & %.6f \\
Minimum execution time (s) & %.6f \\
Maximum execution time (s) & %.6f \\
Number of records & %d \\
\hline
\end{tabular}
\caption{Optimization Timing Statistics}
\end{table}
""" % (avg_time, min_time, max_time, len(df_range))

# Save LaTeX table
output_dir = os.path.dirname(file_path)
os.makedirs(output_dir, exist_ok=True)
with open(os.path.join(output_dir, "optimization_timing_stats.tex"), "w") as f:
    f.write(latex_table)
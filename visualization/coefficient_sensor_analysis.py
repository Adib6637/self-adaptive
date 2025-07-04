import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
file_path = '../log/log_coefficient_actuator.csv'  
df = pd.read_csv(file_path)

plt.figure(figsize=(10, 6))

df.columns = ['eta', 'delta', 'alpha', 'beta', 'counter']

df_subset = df.iloc[:]
# Plotting the loss values
plt.plot(df_subset['eta'], label='eta', color='blue')
plt.plot(df_subset['delta'], label='delta', color='red')
plt.plot(df_subset['alpha'], label='alpha', color='green')
plt.plot(df_subset['beta'], label='beta', color='black')

# Adding titles and labels
plt.title('')#'Loss Value Comparison of Two Models', fontsize=14)
plt.xlabel('Epochs', fontsize=12)
plt.ylabel('Coefficient Value', fontsize=12)
plt.legend()

# Show the plot
plt.show()
